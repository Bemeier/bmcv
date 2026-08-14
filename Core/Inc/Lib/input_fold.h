#ifndef INC_LIB_INPUT_FOLD_H_
#define INC_LIB_INPUT_FOLD_H_

#include "hw_setup.h"
#include "hw_state.h"
#include "ux_state.h"
#include <stdint.h>

// The step between raw hardware and HwState.
//
// Everything here used to live inside bmcv.c, which meant no host could reach
// it: the tests drove engine_tick() with a hand-filled HwState and simply
// skipped clock dispatch, slider CV summing and the config autosave. Those are
// now here, in a core source, so the firmware, the tests, the simulator and
// (later) a VCV Rack module all run the same code and differ only in where the
// numbers come from.

// One tick of raw hardware input. Whoever owns the hardware fills this in: the
// driver layer on the STM32, the widget in VCV Rack, the frontend through the
// wasm API. Deliberately a plain struct rather than a set of callbacks - it
// costs no indirection on a path that reads 24 buttons and 8 encoders every
// tick, it is trivially constructible in a test, and it matches the contract
// engine.h already documents: the caller fills state in, the core reads it.
typedef struct
{
  // Slider potentiometer before any CV is mixed in, in ADC units.
  // SLIDER_MIN_VALUE..SLIDER_MAX_VALUE.
  uint16_t slider_raw;

  // CV inputs in raw ADC units, +/- ADC_10V, indexed by *converter* channel
  // (what get_adc() takes), not by jack. hw_setup->input_adc_idx maps one to
  // the other.
  int16_t cv_raw[N_INPUTS];

  // Rising edge seen on that converter channel since the last tick. Latched by
  // the caller, because hardware catches edges in the DMA callback at a rate
  // faster than the engine loop - a 1ms gate must not be missed just because
  // the tick landed either side of it. Hosts sampling CV faster than they tick
  // should use input_trig_step() to reproduce that.
  uint8_t cv_trig[N_INPUTS];

  // A MIDI Clock (0xF8) / Start (0xFA) byte arrived since the last tick - see
  // midi_realtime.h. Latched the same way as cv_trig and for the same reason:
  // caught in the USB interrupt, consumed once per engine tick. input_fold
  // only acts on these when no input is configured as the clock/reset source
  // respectively - a physical patch cable always wins.
  uint8_t midi_clock_trig;
  uint8_t midi_reset_trig;

  uint8_t button_down[N_BUTTONS];

  // Free-running absolute position. The delta is computed here and is
  // wraparound-safe, so a host may let this overflow.
  int16_t encoder_pos[N_ENCODERS];
} InputSample;

// A second panel, wherever it is.
//
// InputSample is what this host's own hardware is doing; this is what something
// else is, reaching the module out of band - written straight into the
// instance's RAM by a debug probe, or filled by a host scripting input it has no
// hardware for. input_fold merges it into the sample, so nothing downstream can
// tell the two apart: the module answers it exactly as it answers a finger, with
// the UX feedback, the LED response and the autosave that come with that.
//
// Every field is a *level* or a *free-running position*, never a delta waiting
// to be consumed. That is what makes it safe to write over a wire with no
// handshake, and it is the whole design:
//
//   - A torn write costs one tick of a wrong value and self-corrects. Nothing
//     accumulates, so nothing can be applied twice or lost, and a write that is
//     missed or repeated is harmless.
//   - The two sides never have to agree on a baseline. The writer's encoder
//     origin is its own; only movement is read. See sim_input_adopt() for what
//     it costs when a baseline does have to be shared.
//   - Nothing here writes back. The module's own bookkeeping is in InputFrames
//     below, so a writer never sees its struct change under it.
typedef struct
{
  // OR'd with the physical panel's. Both are levels, so a button held here and
  // a button held there are the same button held.
  uint8_t button_down[N_BUTTONS];

  // Summed with the physical encoder's, and the existing wraparound-safe delta
  // is taken over the total. So this is an origin the writer picks - only how
  // far it moves means anything - and it may overflow.
  int16_t encoder_pos[N_ENCODERS];

  // The crossfader, in the same ADC units as InputSample.slider_raw.
  // REMOTE_SLIDER_NONE hands it back to the physical one. The only control
  // where the two panels can genuinely disagree, since it is an absolute
  // position rather than a level to OR or a movement to sum - see
  // REMOTE_SLIDER_RELEASE_RAW for who wins.
  int16_t slider_raw;

  // Bumped by the writer on every update, whether or not anything changed. Zero
  // means never written.
  //
  // Two jobs, and the second is why it is required rather than optional: it
  // separates one update from the next, and it is a heartbeat. A writer that
  // stops - a page refreshed with a button held, a probe cable pulled
  // mid-press - would otherwise leave its levels standing for ever, and a
  // module stuck holding SHIFT has no way back but a power cycle. Nothing here
  // is read once REMOTE_TIMEOUT_US has passed without a new value.
  //
  // Last in the struct deliberately: a writer that cannot update the whole
  // thing atomically writes the fields first and this afterwards.
  uint32_t seq;
} RemoteInput;

#define REMOTE_SLIDER_NONE (-1)

// How long a mailbox is believed after its last new seq. Long enough that a
// hitched frame or a briefly throttled tab does not drop a held button, short
// enough that a pulled cable is not felt.
#define REMOTE_TIMEOUT_US 250000u

// How far the physical crossfader must move to take itself back from a remote
// one, in raw ADC units against a SLIDER_MIN_VALUE..SLIDER_MAX_VALUE travel.
// Above ADC noise by a wide margin, so it means a hand on the fader; small
// enough to be a nudge rather than a sweep.
#define REMOTE_SLIDER_RELEASE_RAW 120

// Bookkeeping that outlives a single tick: this frame and the one before it.
//
// Two named frames rather than a ring with an index: only one tick of history
// is ever wanted - previous button levels, previous encoder positions, previous
// timestamp - and `curr` staying put means ux->hw_state is set once and a live
// debugger has a fixed address to watch. The ring's alternated every tick.
typedef struct
{
  HwState curr;
  HwState prev;

  // Written by whoever is driving this module from outside it, and by nobody
  // here. See RemoteInput.
  RemoteInput remote;

  // What this side knows about that mailbox. Outside it on purpose: a writer
  // that saw its own struct change could not tell a stale mailbox from a fresh
  // one, and the liveness rule below depends on being able to.
  uint32_t remote_seq;        // the last seq acted on
  uint32_t remote_seen_us;    // when it arrived
  uint16_t remote_slider_ref; // where the physical fader sat when the remote took it
  int16_t remote_slider_prev; // the last slider value seen in the mailbox
  uint8_t remote_live;        // is anybody still there
  uint8_t remote_slider_held; // is the remote fader the one being obeyed
} InputFrames;

// Points ux->hw_state at a zeroed frame timestamped now_us. It keeps pointing
// there for the life of the instance.
void input_frames_init(InputFrames* in, UxState* ux, uint32_t now_us);

// Fold one sample into a fresh HwState frame: trigger sources, clock and reset
// latches, slider CV summing, CV levels, button levels, encoder deltas. Points
// ux->hw_state at the new frame and returns the `dirty` flag engine_tick()
// takes - non-zero when a button level changed or an encoder moved.
//
// Reads hardware and writes HwState, and does nothing else. Acting on what it
// finds - starting the clock, resetting phases, saving config - is the
// engine's job, so that all of it happens in one place at one point in the
// tick.
uint8_t input_fold(InputFrames* in, UxState* ux, const InputSample* sample, uint32_t now_us);

// Gate/trigger edge detection with the same hysteresis the ADC driver uses
// (TRIG_THRESH / TRIG_THRESH_LOW), so a host sampling CV at its own rate
// produces the same edges the hardware would. `state` is the caller's
// per-channel latch, zero-initialised. Returns 1 on a rising edge.
uint8_t input_trig_step(int16_t cv, uint8_t* state);

#endif /* INC_LIB_INPUT_FOLD_H_ */
