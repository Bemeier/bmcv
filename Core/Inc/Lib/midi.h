#ifndef INC_DRIVERS_MIDI_H_
#define INC_DRIVERS_MIDI_H_

#include "midi_out.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include "usbd_midi.h"     // IWYU pragma: keep

// The module on the MIDI bus: control changes out, a clock in.
//
// That is all this is now. Watching and driving the module from a browser used
// to come through here as SysEx and has moved to the vendor interface - see
// usblink.h - which left this file as the musical half it was always meant to
// be.

// How many messages one transfer carries: a USB-MIDI event packet is four
// bytes, so the endpoint's 64 hold sixteen.
#define MIDI_MSGS_PER_TRANSFER (MIDI_EPIN_SIZE / 4)

// Wrap `n` messages as USB-MIDI event packets and send them, up to
// MIDI_MSGS_PER_TRANSFER. Sends exactly what it was given and never waits: the
// caller checks midi_idle() first, and anything that did not fit stays in the
// queue it came from.
//
// Exact-length rather than filling a fixed 64-byte buffer, which is what this
// replaced. That version only transmitted once the buffer was full and its
// flush sent all 64 bytes regardless of how many were written this time, so a
// batch shorter than the last one re-sent the tail of the previous one - a
// duplicated clock or a control change reverting to a stale value.
void midi_send_msgs(const MidiMsg* msgs, uint8_t n);

uint8_t midi_idle();

// A MIDI Clock (0xF8) / Start (0xFA) byte is owed since the last call - see
// midi_realtime.h for why Start and not Continue. Read-and-clear, the same
// contract as adc_read_trig_state(): call once per tick and feed the result to
// InputSample.midi_clock_trig / midi_reset_trig.
//
// The clock keeps a backlog rather than a flag, and hands back one per call.
// Several clocks can land between two ticks - batched into one transfer by a
// host that fell behind, or arriving in consecutive transfers - and each is a
// beat. Draining them one per tick keeps the beat grid aligned; see midi.c for
// why compressing them into 250us intervals is harmless to the tempo estimate.
uint8_t midi_read_clock_trig();
uint8_t midi_read_reset_trig();

#endif /* INC_DRIVERS_MIDI_H_ */
