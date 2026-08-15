#include "midi.h"

#include "midi_realtime.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
// The transmit buffer, filled fresh by each midi_send_msgs call. Static because
// USBD_MIDI_SendReport hands the pointer to the USB stack and the transfer
// outlives the call.
static uint8_t buffUsbReport[MIDI_EPIN_SIZE] = {0};

// Set from the USB interrupt, read from the main loop, so volatile and nothing
// more.
static volatile bool midi_clock_flag = false;
static volatile bool midi_reset_flag = false;

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
