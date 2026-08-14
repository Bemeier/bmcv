#include "midi.h"

#include "midi_realtime.h"
#include "string.h"
#include "sysex.h"
#include "version.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
// The transmit buffer, filled fresh by each midi_send_msgs call. Static because
// USBD_MIDI_SendReport hands the pointer to the USB stack and the transfer
// outlives the call.
static uint8_t buffUsbReport[MIDI_EPIN_SIZE] = {0};

// Host-to-module control traffic. All four are set from the USB interrupt and
// read from the main loop, so they are volatile and nothing else. The identity
// flag is the only one that latches rather than edge-triggers - a DFU request
// is answered by not coming back, so it is never cleared.
static SysexParser sysex_parser;
static volatile bool sysex_dfu_requested      = false;
static volatile bool sysex_identity_requested = false;
static volatile bool midi_clock_flag          = false;
static volatile bool midi_reset_flag          = false;

// SPIKE - see SYSEX_CMD_BENCH_REQ. Counts down to zero and stops on its own.
static volatile uint16_t bench_remaining = 0;
static uint16_t bench_seq                = 0;

// The remote input mailbox, on its way in. Staged here rather than written
// straight into the instance because this runs in the USB interrupt, which can
// preempt input_fold halfway through reading it - and a mailbox that is half
// this update and half the last one is the one thing its design does not
// tolerate. The main loop moves it across between ticks, where nothing can
// interleave. See midi_take_remote_input().
_Static_assert(sizeof(RemoteInput) == SYSEX_REMOTE_INPUT_BYTES, "the mailbox is what sysex.h says it is");

static volatile bool remote_pending = false;
static RemoteInput remote_staged;

// The same arrangement for commands, and for the same reason: decoded in the
// interrupt, moved into the instance by the main loop.
_Static_assert(sizeof(RemoteCommand) == SYSEX_REMOTE_COMMAND_BYTES, "the command mailbox is what sysex.h says it is");

static volatile bool command_pending = false;
static RemoteCommand command_staged;

// Streaming credit. Two counters rather than one that both sides adjust: the
// interrupt only ever increments `asked` and the main loop only ever increments
// `sent`, so neither has to do a read-modify-write the other could land in the
// middle of. Outstanding credit is the difference.
static volatile uint32_t stream_asked = 0;
static uint32_t stream_sent           = 0;

// One recognised message. Called from sysex_feed, which calls it once per
// message in the transfer - a host is free to pack several into one, and a
// second command going unnoticed is what made the stream stutter.
static void on_sysex(SysexCmd cmd, const uint8_t* payload, uint8_t len, void* user)
{
  (void) user;

  switch (cmd)
  {
  case SYSEX_CMD_ENTER_UPDATE:
    sysex_dfu_requested = true;
    break;
  case SYSEX_CMD_IDENTITY_REQ:
    sysex_identity_requested = true;
    break;
  case SYSEX_CMD_BENCH_REQ:
    bench_seq       = 0;
    bench_remaining = SYSEX_BENCH_MESSAGES;
    break;
  case SYSEX_CMD_STREAM_REQ:
    stream_asked++;
    break;
  case SYSEX_CMD_REMOTE_INPUT:
    // The length was checked before the command was recognised, so this cannot
    // decode to anything other than a whole mailbox.
    sysex7_decode((uint8_t*) &remote_staged, payload, len);
    remote_pending = true;
    break;
  case SYSEX_CMD_REMOTE_COMMAND:
    sysex7_decode((uint8_t*) &command_staged, payload, len);
    command_pending = true;
    break;
  default:
    break;
  }
}

// Overrides the __weak stub in the MIDI class, and runs in the USB interrupt.
// It does the least it can get away with: parse, latch, return. Acting on
// ENTER_UPDATE here would tear down the USB stack from inside its own ISR.
void USBD_MIDI_DataInHandler(uint8_t* usb_rx_buffer, uint8_t usb_rx_buffer_length)
{
  sysex_feed(&sysex_parser, usb_rx_buffer, usb_rx_buffer_length, on_sysex, NULL);

  MidiRealtimeEvents rt = midi_realtime_feed(usb_rx_buffer, usb_rx_buffer_length);
  if (rt.clock)
    midi_clock_flag = true;
  if (rt.start)
    midi_reset_flag = true;
}

uint8_t midi_read_clock_trig()
{
  if (!midi_clock_flag)
    return 0;
  midi_clock_flag = false;
  return 1;
}

uint8_t midi_read_reset_trig()
{
  if (!midi_reset_flag)
    return 0;
  midi_reset_flag = false;
  return 1;
}

uint8_t midi_dfu_requested() { return sysex_dfu_requested; }

uint8_t midi_bench_active() { return bench_remaining != 0; }

uint8_t midi_take_remote_input(RemoteInput* dst)
{
  if (!remote_pending)
    return 0;

  // Cleared first: another message landing during the copy is a newer mailbox,
  // and leaving the flag set means it is taken next pass rather than lost. The
  // copy itself can be torn by that, which is exactly what the mailbox's levels
  // are built to survive - see RemoteInput.
  remote_pending = false;
  *dst           = remote_staged;
  return 1;
}

uint8_t midi_take_remote_command(RemoteCommand* dst)
{
  if (!command_pending)
    return 0;

  command_pending = false;
  *dst            = command_staged;
  return 1;
}

/* ---- snapshot streaming -------------------------------------------------- */

// One instance, frozen. The stream encodes straight out of this as it goes, so
// there is no second buffer holding the encoded form - but it does mean the
// copy has to sit still for the ~57 transfers it takes to leave, which is why
// it is a copy and not the live instance.
static uint8_t snapshot[sizeof(BmcvInstance)];
static uint16_t snapshot_len = 0;
static SysexStream snapshot_stream;
static bool snapshot_sending = false;

// There was a watchdog here that flushed the IN endpoint when a transfer had
// gone uncollected for 50ms, meant to recover a module left holding a transfer
// by a host that vanished. It is gone, and the reason is worth keeping.
//
// It was written for a wedge that had been reasoned about but never observed,
// and it did its recovering by calling USBD_LL_FlushEP and then setting the
// class handle's state to MIDI_IDLE - reaching past the HAL to contradict
// bookkeeping the PCD still owned. What it actually produced was a module that
// answered nothing at all, ever: any reply the host was slow to drain got
// flushed out from under it, and the endpoint did not reliably come back.
//
// A speculative fix for an unobserved failure, which caused a total one. If a
// real wedge ever turns up, the thing to reach for is the class's own reset
// path, not a poke at its state behind its back.
void midi_stream_poll(uint32_t now_us, const BmcvInstance* m)
{
  (void) now_us;

  // A burst owns the endpoint while it runs; it is a measurement, and sharing
  // it with snapshots would measure something else.
  if (bench_remaining || !midi_idle())
    return;

  // A host that asked while the module was busy elsewhere, or that asked far
  // more often than it collected, does not get to bank the difference: credit
  // beyond the window is dropped so that what goes out is the module as it is
  // now rather than a queue of frames from a moment ago.
  const uint32_t asked = stream_asked;
  if (asked - stream_sent > SYSEX_STREAM_MAX_CREDITS)
    stream_sent = asked - SYSEX_STREAM_MAX_CREDITS;

  if (!snapshot_sending)
  {
    // Control traffic gets the endpoint first. A snapshot holds it for about
    // eleven milliseconds and cannot be interrupted once begun, so anything
    // that arrives mid-snapshot has to wait for a gap - and a stream the host
    // keeps topped up never leaves one. Starving the identity reply that way
    // makes the module undiscoverable while it is working perfectly, which is
    // how it presented: "no BMCV answered", from a module mid-stream.
    if (sysex_identity_requested)
      return;

    if (asked == stream_sent)
      return; // nobody is waiting for one

    // Taken here, between engine ticks, so the copy is internally consistent.
    // The probe's read cannot say that: it runs while the core does, so a blob
    // can straddle a tick. This is the one thing this transport does better.
    memcpy(snapshot, m, sizeof snapshot);
    snapshot_len = (uint16_t) sizeof snapshot;
    sysex_stream_begin(&snapshot_stream, SYSEX_CMD_SNAPSHOT, snapshot, snapshot_len);
    snapshot_sending = true;
  }

  static uint8_t packets[SYSEX_STREAM_TRANSFER_BYTES];
  const uint8_t n = sysex_stream_next(&snapshot_stream, packets, 0);

  if (n == 0)
  {
    // Finished, and one credit is spent. The next one starts as soon as the
    // host asks again, which it does once it has finished with this one - so
    // the rate is what the host can actually absorb.
    snapshot_sending = false;
    stream_sent++;
    return;
  }

  USBD_MIDI_SendReport(&hUsbDeviceFS, packets, n);
}

uint8_t midi_stream_active() { return snapshot_sending; }

void midi_poll_control()
{
  // SPIKE. First claim on the endpoint while a burst is running, and the reason
  // bmcv.c holds its control changes back meanwhile: what is being measured is
  // how fast this endpoint can be filled, and traffic sharing it would be
  // measured as the transport being slower than it is.
  if (bench_remaining && midi_idle())
  {
    // Static for the same reason buffUsbReport is: USBD_MIDI_SendReport hands
    // the pointer to USBD_LL_Transmit and returns, so the buffer has to outlive
    // the call.
    static uint8_t packets[SYSEX_BENCH_PACKET_BYTES];
    uint8_t len = sysex_bench_message(packets, 0, bench_seq++);

    USBD_MIDI_SendReport(&hUsbDeviceFS, packets, len);
    bench_remaining--;
    return;
  }

  // Not mid-snapshot, for the reason bmcv.c gives about control changes: a
  // second SysEx started inside the first is not something a host can make
  // sense of. The request keeps until the snapshot has finished.
  if (!sysex_identity_requested || !midi_idle() || snapshot_sending)
    return; // the IN endpoint is busy; the request keeps until the next pass

  uint8_t reply[SYSEX_IDENTITY_REPLY_LEN];
  uint8_t len = sysex_identity_reply(reply, 0, FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);

  USBD_MIDI_SendReport(&hUsbDeviceFS, reply, len);
  sysex_identity_requested = false;
}

void midi_send_msgs(const MidiMsg* msgs, uint8_t n)
{
  if (n == 0)
    return;
  if (n > MIDI_MSGS_PER_TRANSFER)
    n = MIDI_MSGS_PER_TRANSFER;

  for (uint8_t i = 0; i < n; i++)
  {
    // Cable 0, and the Code Index Number from the status byte's high nibble:
    // 0xB for a control change, 0xF for a System Real-Time byte, which is what
    // the spec asks for in both cases. A Real-Time message is one byte and the
    // other two go out as the zero padding the class expects.
    buffUsbReport[i * 4 + 0] = (uint8_t) (msgs[i].status >> 4);
    buffUsbReport[i * 4 + 1] = msgs[i].status;
    buffUsbReport[i * 4 + 2] = msgs[i].d1;
    buffUsbReport[i * 4 + 3] = msgs[i].d2;
  }

  USBD_MIDI_SendReport(&hUsbDeviceFS, buffUsbReport, (uint16_t) (n * 4));
}

uint8_t midi_idle() { return USBD_MIDI_GetState(&hUsbDeviceFS) == MIDI_IDLE; }
