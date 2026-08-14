#ifndef BMCV_SIM_RT_H_
#define BMCV_SIM_RT_H_

#include "hw_setup.h"
#include "input_fold.h"
#include <stdint.h>

// Host-side runtime glue: unit conversion, tick decimation and gate edge
// latching. Deliberately free of any host's types - no Emscripten, no VCV
// Rack - so the wasm build, the CLI and the Rack plugin share one
// implementation of the fiddly parts, and so all of it is unit-testable
// without a host at all.

/* ---- unit conversion ---------------------------------------------------- */

// The module's converters are bipolar +/-10V. ADC counts are a quarter of DAC
// counts at the same voltage (ADC_10V 8192, DAC_10V 32768).

static inline float sim_dac_to_volts(int16_t dac) { return (float) dac * (10.0f / (float) DAC_10V); }

static inline int16_t sim_volts_to_adc(float volts)
{
  float raw = volts * ((float) ADC_10V / 10.0f);
  // 14-bit signed converter: the positive side stops one count short.
  if (raw > (float) (ADC_10V - 1))
    raw = (float) (ADC_10V - 1);
  if (raw < (float) (-ADC_10V))
    raw = (float) (-ADC_10V);
  return (int16_t) (raw >= 0.0f ? raw + 0.5f : raw - 0.5f);
}

static inline float sim_adc_to_volts(int16_t adc) { return (float) adc * (10.0f / (float) ADC_10V); }

/* ---- tick decimation ---------------------------------------------------- */

// The engine is dt-driven, so it is correct at any tick rate; running it once
// per audio sample is simply wasteful. A host samples CV every frame but ticks
// the engine on a divider.
//
// Time is accumulated in Q32 microseconds rather than as a rounded integer
// step. At 48kHz the step is exactly 250us and it makes no difference, but at
// 44.1kHz a divider of 11 gives 249.43us, and rounding that to 249 would run
// the engine's clock 0.17% slow - every LFO permanently flat, and drifting
// further from the host's transport the longer the patch is open. Accumulating
// the exact value and handing the engine the *difference* between successive
// integer timestamps keeps the long-run rate exact while still giving
// input_fold a whole number of microseconds.
typedef struct
{
  uint32_t divider; // host frames per control tick
  uint32_t counter;

  uint64_t us_q32; // accumulated time, microseconds in Q32
  uint64_t dt_q32; // exact microseconds per control tick, Q32

  uint32_t now_us; // integer timestamp to pass to the engine
  uint32_t dt_us;  // now_us minus the previous now_us; varies by +/-1us
} SimTickDiv;

// control_rate_hz is what the engine will actually run at; the divider is
// rounded to the nearest whole number of host frames, and the time step is
// derived from the *rounded* divider so it agrees with what really happens.
//
// Starts the clock at zero, so this is the call for a fresh instance.
void sim_tickdiv_config(SimTickDiv* d, float sample_rate_hz, float control_rate_hz);

// The same, keeping the clock where it is. This is what a host calls when the
// sample rate changes under a running module: the engine reads elapsed time as
// an unsigned difference, so a timestamp that goes backwards by a second is
// indistinguishable from one that jumps forward by 71 minutes, and every
// oscillator phase and the clock's tempo estimate go with it.
void sim_tickdiv_reconfig(SimTickDiv* d, float sample_rate_hz, float control_rate_hz);

// Advance one host frame. Returns 1 when a control tick is due, having
// advanced now_us and set dt_us.
uint8_t sim_tickdiv_step(SimTickDiv* d);

// The rate the engine actually ticks at, which is sample_rate/divider and not
// necessarily the rate that was requested.
static inline float sim_tickdiv_rate_hz(const SimTickDiv* d)
{
  return d->dt_q32 ? (float) (4294967296.0 * 1000000.0 / (double) d->dt_q32) : 0.0f;
}

/* ---- gate edge latching ------------------------------------------------- */

// On hardware, gate edges are caught in the ADC DMA callback, which runs
// faster than the engine loop, so a 1ms pulse is never missed. A host that
// samples CV per audio frame but ticks the engine at 4kHz has to do the same
// or it will drop short triggers.
//
// Indexed by *converter channel*, matching InputSample.cv_raw.
typedef struct
{
  uint8_t state[N_INPUTS];   // hysteresis latch, as in the ADC driver
  uint8_t pending[N_INPUTS]; // edge seen since the last take
} SimTrigLatch;

void sim_trig_reset(SimTrigLatch* t);

// Feed one CV sample in raw ADC units. Call per host frame.
void sim_trig_sample(SimTrigLatch* t, uint8_t channel, int16_t cv);

// Consume the pending edge for one channel. Call when building an InputSample.
uint8_t sim_trig_take(SimTrigLatch* t, uint8_t channel);

// Force a one-shot edge, for a UI button or a script that wants a trigger
// without synthesising a voltage ramp.
void sim_trig_fire(SimTrigLatch* t, uint8_t channel);

/* ---- filling an InputSample --------------------------------------------- */
//
// A host thinks in panel jacks and volts; InputSample is indexed by *converter
// channel* and counted in ADC units. These three do that translation, so no
// host has to remember which way round hw_setup->input_adc_idx goes - getting
// it backwards routes the clock into a modulation input and is invisible until
// something refuses to sync.

// One input jack, in volts. Latches the gate edge too, so call it as often as
// the host has samples rather than once per control tick.
void sim_input_cv(InputSample* s, SimTrigLatch* t, const HwSetup* hw, uint8_t jack, float volts);

// Drain the latched edges into the sample. Once per control tick, immediately
// before handing it to the engine.
void sim_input_take_trigs(InputSample* s, SimTrigLatch* t);

// The crossfader, as a fraction of its travel. 0 is the bottom of the raw
// range, which is the end of the panel scene B anchors to.
void sim_input_slider(InputSample* s, float pos01);

// Re-baseline a host's sample against a hardware frame it did not produce.
//
// For adopting another module's state - see bmcv_sim_import(). An InputSample
// is the host's, not the instance's, so a snapshot brings a HwState saying the
// encoders are at one position while the host's sample still says another. The
// next fold would read the difference as a turn: import a module with its ENC 3
// forty detents along and the first tick applies forty detents of edit to a
// patch nobody touched.
//
// Encoder positions and button levels are copied straight through by
// input_fold, so those come back exactly. The slider does not: input_fold sums
// slider CV into it, so what comes back is the raw position only while no input
// is in INPUT_SLIDER mode, and is off by that CV when one is. Taking it anyway
// beats leaving the crossfader somewhere the snapshot never was - the error is
// one tick's worth of CV and self-corrects, where a stale slider is a scene
// jump that does not.
//
// Gate latches are the caller's: an edge caught before the adoption belongs to
// the module that was replaced.
void sim_input_adopt(InputSample* s, const HwState* hw_state);

#endif /* BMCV_SIM_RT_H_ */
