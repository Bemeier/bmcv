#ifndef INC_LIB_MIDI_REALTIME_H_
#define INC_LIB_MIDI_REALTIME_H_

#include <stdint.h>

// What a USB-MIDI transfer means for the clock, with no USB in it: the four
// System Real-Time status bytes, and nothing else on the bus.
//
// It used to share this endpoint with a SysEx parser carrying everything a host
// wanted to say to the module. That has moved to the vendor interface - see
// Core/Inc/Lib/usblink.h - and this is all that reads the MIDI bus now.

#define MIDI_RT_CLOCK 0xF8
#define MIDI_RT_START 0xFA
#define MIDI_RT_CONTINUE 0xFB
#define MIDI_RT_STOP 0xFC

// Clock is a count and Start is a flag, and the difference is not an
// inconsistency: a beat is advanced once per clock byte, so two of them in one
// transfer have to be worth two, while resetting twice is the same as resetting
// once.
//
// This mattered more than it looks. A USB-MIDI transfer carries up to 16 event
// packets and a host packs what it has, so a DAW that falls behind delivers its
// clocks in a batch - and reporting a batch as a single `clock = 1` silently
// dropped the rest, which the sync loop saw as a gap in the beat grid. The same
// mistake, on the same endpoint, is what the SysEx parser this replaced was
// retired for; see docs/live-module.md.
typedef struct
{
  uint8_t clocks; // how many Clock (0xF8) bytes were in this transfer
  uint8_t start;  // whether a Start (0xFA) byte was
} MidiRealtimeEvents;

// Feed one USB MIDI OUT transfer: `len` bytes of 4-byte USB-MIDI event
// packets. System Real-Time messages are Code Index Number 0xF ("Single Byte")
// and arrive whole in one packet, so this needs no state between calls - the
// one thing that made the SysEx parser this replaced awkward.
//
// Only Clock and Start are reported. Continue must not report as Start: it
// resumes rather than restarts, and reporting it the same way would reset
// phase on every DAW punch-in. Stop needs nothing done - a source that stops
// pulsing already reads as gone once Clock_Poll's own timeout passes, the
// same as an unplugged CV clock.
MidiRealtimeEvents midi_realtime_feed(const uint8_t* packets, uint8_t len);

#endif /* INC_LIB_MIDI_REALTIME_H_ */
