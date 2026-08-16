#include "midi.h"

#include "midi_realtime.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
// The transmit buffer, filled fresh by each midi_send_msgs call. Static because
// USBD_MIDI_SendReport hands the pointer to the USB stack and the transfer
// outlives the call.
static uint8_t buffUsbReport[MIDI_EPIN_SIZE] = {0};

// Set from the USB interrupt, read from the main loop.
//
// The clock is a *backlog*, not a flag. Counting it here is only half the fix -
// midi_realtime_feed counts the bytes in one transfer, and this keeps the ones
// the loop has not collected yet, since several transfers can land between two
// engine ticks as easily as several bytes can land in one transfer.
//
// Drained one per tick, which compresses a burst into tick-wide intervals. That is
// the right answer rather than a compromise: the beat grid advances once per
// clock either way, so it stays aligned, and Clock_Trigger's implausible-
// interval guard rejects the compressed intervals as the tempo samples they are
// not. Dropping the clocks instead loses the beats themselves, which nothing
// downstream can recover.
static volatile uint8_t midi_clock_pending = 0;
static volatile bool midi_reset_flag       = false;

// One beat's worth at 24 PPQN. A backlog deeper than this is not a host that
// stuttered, it is one that has gone away or is streaming faster than the
// module can be driven, and replaying it would walk the beat counter forward
// through time that has already passed.
#define MIDI_CLOCK_MAX_PENDING 24u

// Overrides the __weak stub in the MIDI class, and runs in the USB interrupt.
//
// All that is left of it. Everything else a host used to say over this endpoint
// - snapshots, the input mailbox, reset, the bootloader - moved to the vendor
// interface, where it needs no seven-bit encoding, no SysEx framing and no turn
// against the engine's own output. See usblink.h.
//
// What remains is what MIDI is actually for: a clock arriving from a DAW.
void USBD_MIDI_DataInHandler(uint8_t* usb_rx_buffer, uint8_t usb_rx_buffer_length)
{
  MidiRealtimeEvents rt = midi_realtime_feed(usb_rx_buffer, usb_rx_buffer_length);

  if (rt.clocks)
  {
    // Saturating, and safe without a lock because this interrupt is the only
    // producer: the main loop only ever decrements, so it cannot land inside
    // the add and turn it into a different number.
    uint8_t pending = __atomic_load_n(&midi_clock_pending, __ATOMIC_ACQUIRE);
    uint8_t room    = (uint8_t) (MIDI_CLOCK_MAX_PENDING - pending);
    if (pending < MIDI_CLOCK_MAX_PENDING)
      __atomic_add_fetch(&midi_clock_pending, rt.clocks < room ? rt.clocks : room, __ATOMIC_ACQ_REL);
  }

  if (rt.start)
    midi_reset_flag = true;
}

uint8_t midi_read_clock_trig()
{
  if (__atomic_load_n(&midi_clock_pending, __ATOMIC_ACQUIRE) == 0)
    return 0;

  // Sound against the interrupt above for the mirror of its reason: this is the
  // only consumer, so nothing else can take the one just observed.
  __atomic_sub_fetch(&midi_clock_pending, 1u, __ATOMIC_ACQ_REL);
  return 1;
}

uint8_t midi_read_reset_trig()
{
  if (!midi_reset_flag)
    return 0;
  midi_reset_flag = false;
  return 1;
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
