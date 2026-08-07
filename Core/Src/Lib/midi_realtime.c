#include "midi_realtime.h"

MidiRealtimeEvents midi_realtime_feed(const uint8_t* packets, uint8_t len)
{
  MidiRealtimeEvents ev = {0};

  for (uint8_t i = 0; (uint16_t) i + 4u <= (uint16_t) len; i += 4)
  {
    if ((packets[i] & 0x0F) != 0x0F)
      continue; // not a Single Byte packet - not System Real-Time

    switch (packets[i + 1])
    {
    case MIDI_RT_CLOCK:
      ev.clock = 1;
      break;
    case MIDI_RT_START:
      ev.start = 1;
      break;
    default:
      break;
    }
  }

  return ev;
}
