#ifndef INC_LIB_CHANNEL_H_
#define INC_LIB_CHANNEL_H_

#include "config.h"
#include "engine_state.h"
#include "hw_state.h"
#include "stepped.h"
#include <stdint.h>

// The signal path for one channel. Everything here takes exactly the state it
// touches - the config it reads, the engine state it writes, the hardware
// frame it samples - and a plain channel index.
//
// It used to take UxState*, which is the whole module: the interaction layer,
// the LED setup tables and the preset vtable came along with it, and nothing
// structural stopped a DSP function from reading shift_state. Narrow
// parameters are what actually enforce the split the layering describes, and
// they are what an audio thread in a plugin host wants to be handed.

// ---------------------------------------------------------------------------
// The sync loop
//
// A channel's phase is corrected toward where the clock says it should be by
// adding a frequency offset proportional to the phase error. That makes it a
// first-order system: the error decays exponentially and cannot overshoot,
// which is the property this module wants - an LFO arriving late is far less
// noticeable than one that lurches past and comes back.
//
// These three numbers are the whole tuning. tests/test_pll.c measures what they
// buy; docs/pll.md carries the baseline to compare against.

// How long an uncorrected phase error takes to decay to 1/e of itself. The
// proportional gain is 1/tau, so this is the loop's whole speed setting.
//
// It was 1.0s, which reproduced what the hand-tuned version did: no named time
// constant, just a gain that happened to be 1. That took 3.9s to close a
// half-cycle step, which reads on the panel as channels drifting into line
// rather than snapping to it.
//
// 0.35s because the loop is first order and so cannot overshoot for any tau -
// the phase step rings zero times at every value measured from 1.0 down to
// 0.15. What sets the floor is jitter: this loop is the only thing filtering a
// wobbly clock, and the oscillator's own rate wobble under a 5% jittery clock
// goes as 1/tau - 1.3% at 1.0s, 2.9% here, 4.6% by 0.2s. 0.35s is about where
// buying more speed starts costing visible wobble instead of nothing.
// See docs/pll.md, "Step 5".
#define PLL_TAU_S 0.35f

// The most the loop may pull a channel off its nominal rate, as a fraction of
// it. The correction is a *speed* change, so an unbounded correction is an
// unbounded lurch: a large phase error used to be worth a frequency offset
// larger than the frequency itself, which is heard as the channel stalling or
// running backwards. Beyond this the error closes at a constant rate instead of
// an exponential one, so a big error takes longer and sounds like nothing.
//
// This is the knob for what a *transition* costs, because a large error is
// clamp-limited for most of its life: it alone takes the half-cycle step from
// 1.97s at 0.15 to 1.55s at 0.25 with tau held. Raised to 0.25 - a quarter off
// rate is a lean you can see on a slow LFO and not a lurch. 0.35 was measured
// and rejected: it closes 0.4s sooner and pegs the clamp through the settle
// after a fader move, which is the artifact this limit exists to prevent.
#define PLL_MAX_PULL 0.25f

// Smoothing on the correction, so the oscillator's frequency has no corner in
// it. Deliberately far shorter than PLL_TAU_S - it is there to round the edges
// of a step, not to slow the loop down.
//
// In seconds, not in ticks. It was a per-tick fraction, which made the filter's
// time constant a property of whatever rate the host happened to tick at.
#define PLL_SMOOTH_S 0.003f

// How long the ratio has to hold still before a new alignment period is taken,
// and how far it may drift in that time and still count as still.
//
// A ratio moving through a crossfade passes a rational approximation every few
// ticks and is between them the rest of the time, so acquiring on the instant
// meant acquiring and discarding eleven alignment periods in a second - the
// correction switching on and off with them. Waiting for the fader to stop is
// what makes a sweep one acquisition instead of eleven.
#define PLL_RATIO_STABLE_US MS(120)
#define PLL_RATIO_EPS 0.001f

// How close to the top of a beat the alignment rational may be swapped. The old
// grid and the new one meet at the beat boundary and are furthest apart in the
// middle of one, so the swap is held for this window and the step it would
// otherwise make lands where there is none. Wide enough to catch a tick at any
// tempo and tick period the module runs at; narrow enough that the two grids
// have not diverged within it.
#define PLL_RETAKE_WINDOW 0.05f

// Zero the oscillator state and open the mute gate. Called at power-on and
// whenever a channel is reset.
void channel_init(uint8_t ch, EngineState* es);

// Back to phase zero, leaving everything else alone. What a reset input does.
void channel_reset_phase(uint8_t ch, EngineState* es);

// Restore one parameter, or a whole channel, to its default value.
// `scene` < 0 means every scene.
void channel_reset_param(uint8_t ch, EngineConfig* cfg, int8_t scene, int8_t param);
void channel_reset(uint8_t ch, EngineState* es, EngineConfig* cfg, int8_t scene);

// One tick of this channel: blend the scene parameters, advance the phase,
// lock it to the beat, generate, cross-modulate, quantize. Lands in
// es->channels_output_level[] and es->channels_effective[].
// Per-channel derived state: memos of work whose answer depends only on inputs
// that usually are not moving. Rebuilt from those inputs whenever they do, so
// it holds nothing a host needs and lives at the end of the instance, where a
// snapshot stops. See BMCV_SNAPSHOT_BYTES.
typedef struct
{
  SteppedScratch stepped;

  // find_denominator()'s answer, and the ratio it was computed for.
  //
  // It is a search with a modff() in it - a real call on this target - and it
  // ran for every channel on every tick while its answer is a function of the
  // ratio alone. A standing patch hands it the same float for minutes. Measured
  // on the module: 17us of a 142.7us tick, spent re-deriving eight answers that
  // were all last tick's.
  //
  // Keyed on the exact float, so what comes back is what the call would have
  // returned, bit for bit. Zero is "nothing cached" and cannot collide with a
  // real ratio: the multiplier is p+1 for p >= 0 and -1/(p-1) for p < 0, both
  // strictly positive.
  float gcd_ratio;
  int16_t gcd_now;
} ChannelScratch;

void channel_compute(uint8_t ch, EngineState* es, const EngineConfig* cfg, const HwState* hw, ChannelScratch* scratch);

// Update this channel's own output-trigger edge state, so other channels can
// use it as a trigger source.
void channel_detect_trigger(uint8_t ch, EngineState* es);

// Advance the output mute ramp and publish es->channels_gated_level[], which
// is what actually leaves the module.
//
// Mute is an output-stage gain, not a zeroed channels_output_level, so a muted
// channel still cross-modulates and still triggers - the two levels genuinely
// differ. engine_tick calls this once per tick; a host reads
// channels_gated_level[] and needs to know nothing else.
void channel_apply_mute(uint8_t ch, EngineState* es, uint8_t muted, uint32_t dt_us);

// Consume this channel's pending output trigger, if any.
uint8_t channel_take_trig(uint8_t ch, EngineState* es);

#endif /* INC_LIB_CHANNEL_H_ */
