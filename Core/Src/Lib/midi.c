#include "midi.h"

#include "midi_realtime.h"
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

// Overrides the __weak stub in the MIDI class, and runs in the USB interrupt.
// It does the least it can get away with: parse, latch, return. Acting on
// ENTER_UPDATE here would tear down the USB stack from inside its own ISR.
void USBD_MIDI_DataInHandler(uint8_t* usb_rx_buffer, uint8_t usb_rx_buffer_length)
{
  switch (sysex_feed(&sysex_parser, usb_rx_buffer, usb_rx_buffer_length))
  {
  case SYSEX_CMD_ENTER_UPDATE:
    sysex_dfu_requested = true;
    break;
  case SYSEX_CMD_IDENTITY_REQ:
    sysex_identity_requested = true;
    break;
  default:
    break;
  }

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

void midi_poll_control()
{
  if (!sysex_identity_requested || !midi_idle())
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
