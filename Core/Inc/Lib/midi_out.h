#ifndef INC_LIB_MIDI_OUT_H_
#define INC_LIB_MIDI_OUT_H_

#include "engine_state.h"
#include "hw_state.h"
#include "midi_realtime.h" // the System Real-Time status bytes this emits
#include <stdint.h>

// What the module says about itself on the MIDI bus: eight channel outputs and
// four CV inputs as control changes, plus a clock.
//
// The counterpart to midi_realtime.c and sysex.c, which read the bus. Like
// them this is hardware-free - it produces MIDI messages and never touches an
// endpoint. A host drains the queue into whatever transport it has: the
// firmware wraps each message into a USB-MIDI event packet, the browser hands
// the bytes to a Web MIDI port.
//
// Plain messages rather than USB-MIDI packets because the USB framing already
// exists on the firmware side (MIDI_addToUSBReport) and no other host wants it.

// Everything goes out on MIDI channel 1, in one contiguous CC block: the eight
// channels first, then the four inputs.
#define MIDI_OUT_STATUS_CC 0xB0
#define MIDI_OUT_CC_BASE 16
#define MIDI_OUT_CC_INPUT_BASE (MIDI_OUT_CC_BASE + N_CHANNELS)
#define MIDI_OUT_CC_COUNT (N_CHANNELS + N_INPUTS)

// How often the control changes are reconsidered. The engine ticks at 4kHz,
// which is far more resolution than a CC has anywhere to put.
//
// It is a divider rather than a free run because the worst case is not close to
// harmless: a full-scale sine at 5Hz crosses 128 quantization levels about 256
// times a cycle, which is ~1280 CC/s on one channel and ~10k/s across eight -
// inside what the endpoint can carry, with no margin left and a DAW that does
// not enjoy it. At 500Hz the ceiling is MIDI_OUT_CC_COUNT per slot.
#define MIDI_OUT_PUBLISH_US 2000u

// System Real-Time is not on that divider - see midi_out_publish.
#define MIDI_OUT_CLOCK_PPQN 24u

// Slots of clock that may be emitted from one tick. The phase advances by one
// slot per 20.8ms at 120BPM against a 250us tick, so anything beyond a couple
// is a phase jump rather than elapsed time, and re-sending it as clock would
// hand the far end a tempo spike.
#define MIDI_OUT_CLOCK_MAX_BURST 4

// Power of two: the ring wraps by mask. One publish slot is at most
// MIDI_OUT_CC_COUNT messages plus a clock, so this holds two slots over.
#define MIDI_OUT_QUEUE_LEN 32

typedef struct
{
  uint8_t status;
  uint8_t d1;
  uint8_t d2;
  uint8_t len; // 1 for System Real-Time, 3 for a control change
} MidiMsg;

typedef struct
{
  MidiMsg q[MIDI_OUT_QUEUE_LEN];
  uint8_t head;
  uint8_t tail;

  // Last value actually sent for each CC, so an unchanging output costs
  // nothing. 0xFF is "never sent" - a real value is 0..127, so it cannot
  // collide - and is also what an overflowed message resets its CC to, which
  // is what makes a dropped message re-send on the next slot instead of
  // leaving the far end stuck at a stale value.
  uint8_t last_cc[MIDI_OUT_CC_COUNT];

  uint32_t last_publish_us;
  uint8_t have_published;

  uint8_t clk_slot;    // last 24-PPQN slot emitted
  uint8_t clk_running; // a Start has been sent and no Stop since
} MidiOut;

void midi_out_init(MidiOut* o);

// One tick's worth of output. Call after engine_tick, with the same timestamp:
// it reads the levels that tick produced and the clock events it acted on.
//
// Control changes are reconsidered every MIDI_OUT_PUBLISH_US and emitted only
// when the 7-bit value moved. The clock is reconsidered every call, because the
// publish divider would put 10% of jitter on a 24-PPQN tick where the engine
// tick puts 1.2%.
void midi_out_publish(MidiOut* o, const EngineState* es, const HwState* hw, uint32_t now_us);

// Take up to `max` queued messages. Returns how many were written. A host calls
// this when its transport is ready and sends what it gets; nothing here blocks
// or waits on one.
uint8_t midi_out_drain(MidiOut* o, MidiMsg* dst, uint8_t max);

// Converter units to a 7-bit CC value. The full +/-10V range maps onto 0..127,
// so 0V is 64 and a channel clamped to 0..5V uses a quarter of the range.
uint8_t midi_out_scale7(int16_t dac_level);

#endif /* INC_LIB_MIDI_OUT_H_ */
