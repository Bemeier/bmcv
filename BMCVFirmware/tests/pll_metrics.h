// Measuring the phase-lock loop.
//
// "It sounds right" is how the sync loop got tuned, and that is a real answer -
// but it is not one a test can hold onto while the algorithm underneath it
// changes. This turns a run into numbers.
//
// Four things matter, and they pull against each other:
//
//   settling   how long until the channel is where the clock says it should be
//   overshoot  whether it goes past and has to come back
//   ringing    whether it crosses back and forth before it stops
//   slew       how hard the instantaneous frequency is yanked on the way
//
// A loop tuned only for the first is fast and audibly wrong: the correction is
// added to the oscillator's frequency, so a big correction is a channel that
// visibly speeds up and slows down. A loop tuned only for the last is smooth
// and never arrives. Every metric here exists so that a change to the loop
// shows up as a trade rather than as "feels about the same".
//
// Errors are reported in *beats* unless a name says otherwise. The loop works
// in cycles of the channel's own waveform, which is not comparable across
// ratios; a beat is the thing the module is trying to line up with.

#ifndef BMCV_PLL_METRICS_H_
#define BMCV_PLL_METRICS_H_

#include "fixture.h"
#include <stdint.h>

// 40 seconds at ENGINE_TICK_US. A PllTrace is ~7MB, so declare it `static` in
// the test rather than on the stack; the long-run alignment cases set
// `decimate` and sample every Nth tick instead of growing this.
#define PLL_TRACE_MAX 160000

typedef struct
{
  // Keep only every Nth sample. 0 and 1 both mean every one. Set it before the
  // run; pll_trace_reset leaves it alone, since it is a property of what the
  // test is measuring rather than of the data.
  uint32_t decimate;
  uint32_t skipped;

  uint32_t n;
  float t_s[PLL_TRACE_MAX];        // seconds since the trace started
  float err_beats[PLL_TRACE_MAX];  // signed, wrapped
  float corr_hz[PLL_TRACE_MAX];    // what the loop is adding to the frequency
  float freq_hz[PLL_TRACE_MAX];    // the nominal rate it is adding it to
  uint8_t locked[PLL_TRACE_MAX];   // whether there was anything to lock to
  float phase[PLL_TRACE_MAX];      // the channel's output phase, 0..1
  uint64_t beat[PLL_TRACE_MAX];    // clock beat counter, for alignment checks
  int16_t gcd[PLL_TRACE_MAX];      // the alignment period the loop chose, in beats
  float beat_phase[PLL_TRACE_MAX]; // 0..1 within the beat
} PllTrace;

typedef struct
{
  // The window the metrics were computed over.
  float duration_s;

  // Time until |error| drops below the tolerance and stays there. Negative if
  // it never does - which is the honest answer, not a failure to measure.
  float settle_s;

  // Largest excursion, and the largest after the first zero crossing. The
  // second is the overshoot proper: a loop that approaches from one side and
  // stops has none.
  float peak_err_beats;
  float overshoot_beats;

  // Overshoot as a fraction of the initial error. ~0 is critically damped or
  // slower; 0.16 is the textbook 0.5 damping ratio; above ~0.3 rings audibly.
  float overshoot_ratio;

  // Zero crossings after the first. This is the ringing count - a well-behaved
  // loop settles with 0 or 1, and anything climbing from there is oscillation.
  uint32_t crossings;

  // The frequency the loop pulled the oscillator to, at its worst, as a
  // fraction of nominal. This is the artifact metric: 0.5 means the channel
  // briefly ran at 1.5x its rate, which is heard.
  float max_freq_dev;

  // How fast that pull changed, per second, as a fraction of nominal. A step
  // in frequency is a corner in the waveform; this is what bounds it.
  float max_freq_slew;

  // What is left once it has settled: RMS and worst case over the last third.
  float rms_err_tail_beats;
  float max_err_tail_beats;

  // The largest step in output phase that the oscillator's own rate does not
  // account for, in cycles.
  //
  // The phase accumulator may only ever wrap at a whole cycle - that is what
  // makes a wrap invisible, since the waveform is identical either side of it.
  // Anything else is a discontinuity in the output: a jump in the middle of a
  // waveform, heard as a click and seen on a scope as a broken edge. This
  // should be at the noise floor in every scenario, always.
  float max_phase_jump;

  // How many times find_denominator changed its mind about the alignment
  // period. The target phase is derived from `beat_counter %% gcd`, so every
  // change teleports the target by whole beats and the loop chases the jump -
  // this is the count of discontinuities fed to a loop that assumes a
  // continuous target.
  uint32_t gcd_changes;
} PllMetrics;

// ---------------------------------------------------------------------------
// Driving

// A clock generator that feeds pulses into the fixture at a tempo, in the same
// PULSES_PER_BEAT the module expects. Jitter is a fraction of the pulse period,
// applied deterministically (a fixed sequence, not rand()) so a failing run
// reproduces.
typedef struct
{
  float bpm;
  float jitter;      // 0..1 of a pulse period
  uint32_t next_us;  // when the next pulse is due
  uint32_t seed;     // for the jitter sequence
  uint8_t running;   //
  uint8_t pulses_per_beat;
} PllClock;

void pll_clock_init(PllClock* c, float bpm);

// Advance the fixture by dt_us, emitting whatever clock pulses fall in that
// interval, and append a sample for channel `ch` to `tr` (pass NULL not to).
void pll_step(Fixture* f, PllClock* c, uint8_t ch, uint32_t dt_us, PllTrace* tr);

// Run for a duration. Convenience over pll_step at ENGINE_TICK_US.
void pll_run(Fixture* f, PllClock* c, uint8_t ch, float seconds, PllTrace* tr);

// Run until the clock reaches a point within the beat, so a test can place a
// disturbance where it actually costs something. See the note on the
// definition: a ratio change on a beat boundary produces no error at all.
void pll_run_to_beat_phase(Fixture* f, PllClock* c, uint8_t ch, float target);

void pll_trace_reset(PllTrace* tr);

// ---------------------------------------------------------------------------
// Measuring

// tol_beats is what counts as arrived - PLL_TOL_BEATS unless a test has a
// reason.
#define PLL_TOL_BEATS 0.01f // 1% of a beat: 5ms at 120bpm

PllMetrics pll_measure(const PllTrace* tr, float tol_beats);

// Print a one-line summary, so a run of the suite reads as a characterisation
// report rather than a pass/fail. Goes to stdout beside the testkit summary.
void pll_report(const char* name, const PllMetrics* m);

void pll_report_header(void);

#endif /* BMCV_PLL_METRICS_H_ */
