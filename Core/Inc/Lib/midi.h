#ifndef INC_DRIVERS_MIDI_H_
#define INC_DRIVERS_MIDI_H_

#include "input_fold.h" // IWYU pragma: keep
#include "instance.h"   // IWYU pragma: keep
#include "midi_out.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include "usbd_midi.h"     // IWYU pragma: keep

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

// Non-zero once the host has asked the module to reboot into the ROM DFU
// bootloader. Latched in the USB interrupt and never cleared - the main loop's
// only correct response is fw_update_enter_dfu(), which does not return.
uint8_t midi_dfu_requested();

// Answer any pending host control request that needs the IN endpoint. Cheap
// enough to call every pass; it does nothing until there is something to say.
void midi_poll_control();

/* ---- driving and watching the module over its own USB port --------------- */
//
// The same two directions the debug probe offers, over the MIDI endpoint the
// module already enumerates - no probe on the programming header. See
// docs/plans/midi-transport.md.

// Collect a remote input mailbox if one arrived since the last call. Call from
// the main loop between ticks, never from the tick: this is what keeps a USB
// interrupt from rewriting the mailbox while input_fold is reading it.
// Returns non-zero when `dst` was written.
uint8_t midi_take_remote_input(RemoteInput* dst);

// The same for a reset / forget-storage command. Call from the main loop, and
// hand what it writes to bmcv_instance_take_command.
uint8_t midi_take_remote_command(RemoteCommand* dst);

// Push snapshots of `instance` out while a host keeps asking for them. Copies
// it when a message starts, so call this between ticks and the copy is
// internally consistent - unlike a probe's read, which runs while the core
// does. Does nothing until a SYSEX_CMD_STREAM_REQ arrives and stops
// SYSEX_STREAM_TIMEOUT_US after the last one.
void midi_stream_poll(uint32_t now_us, const BmcvInstance* m);

// Non-zero while a snapshot is partway out of the endpoint.
uint8_t midi_stream_active();

// A MIDI Clock (0xF8) / Start (0xFA) byte was latched since the last call -
// see midi_realtime.h for why Start and not Continue. Read-and-clear, the
// same contract as adc_read_trig_state(): call once per tick and feed the
// result to InputSample.midi_clock_trig / midi_reset_trig.
uint8_t midi_read_clock_trig();
uint8_t midi_read_reset_trig();

#endif /* INC_DRIVERS_MIDI_H_ */
