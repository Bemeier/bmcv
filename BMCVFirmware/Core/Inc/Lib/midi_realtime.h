#ifndef INC_LIB_MIDI_REALTIME_H_
#define INC_LIB_MIDI_REALTIME_H_

#include <stdint.h>

// What a USB-MIDI transfer means for the clock, with no USB in it - the
// counterpart to sysex.c, for the four System Real-Time status bytes the
// clock cares about rather than the one SysEx stream it does.

#define MIDI_RT_CLOCK 0xF8
#define MIDI_RT_START 0xFA
#define MIDI_RT_CONTINUE 0xFB
#define MIDI_RT_STOP 0xFC

typedef struct
{
  uint8_t clock; // a Clock (0xF8) byte was in this transfer
  uint8_t start; // a Start (0xFA) byte was in this transfer
} MidiRealtimeEvents;

// Feed one USB MIDI OUT transfer: `len` bytes of 4-byte USB-MIDI event
// packets - the same framing sysex_feed reads. System Real-Time messages are
// Code Index Number 0xF ("Single Byte") and arrive whole in one packet, so
// unlike SysEx this needs no state between calls.
//
// Only Clock and Start are reported. Continue must not report as Start: it
// resumes rather than restarts, and reporting it the same way would reset
// phase on every DAW punch-in. Stop needs nothing done - a source that stops
// pulsing already reads as gone once Clock_Poll's own timeout passes, the
// same as an unplugged CV clock.
MidiRealtimeEvents midi_realtime_feed(const uint8_t* packets, uint8_t len);

#endif /* INC_LIB_MIDI_REALTIME_H_ */
