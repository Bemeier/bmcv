#include "midi.h"

#include "midi_realtime.h"
#include "sysex.h"
#include "version.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t buffUsbReport[MIDI_EPIN_SIZE] = {0};
static uint8_t buffUsbReportNextIndex        = 0;

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

void MIDI_addToUSBReport(uint8_t cable, uint8_t message, uint8_t param1, uint8_t param2)
{
  buffUsbReport[buffUsbReportNextIndex++] = (cable << 4) | (message >> 4);
  buffUsbReport[buffUsbReportNextIndex++] = (message);
  buffUsbReport[buffUsbReportNextIndex++] = (param1);
  buffUsbReport[buffUsbReportNextIndex++] = (param2);

  if (buffUsbReportNextIndex == MIDI_EPIN_SIZE)
  {
    while (USBD_MIDI_GetState(&hUsbDeviceFS) != MIDI_IDLE)
    {
    };
    USBD_MIDI_SendReport(&hUsbDeviceFS, buffUsbReport, MIDI_EPIN_SIZE);
    buffUsbReportNextIndex = 0;
  }
}

uint8_t midi_idle() { return USBD_MIDI_GetState(&hUsbDeviceFS) == MIDI_IDLE; }

void update_midi()
{
  USBD_MIDI_SendReport(&hUsbDeviceFS, buffUsbReport, MIDI_EPIN_SIZE);
  buffUsbReportNextIndex = 0;
}
