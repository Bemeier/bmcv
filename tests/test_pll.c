// Characterising the phase-lock loop.
//
// These are not tight regression pins. The loop is about to be reworked, and a
// test that fails the moment it improves is worse than no test. Each case
// asserts the property that must hold whatever the algorithm is - it settles,
// it does not ring, it does not yank the frequency, it stays aligned over the
// long run - at bounds loose enough to leave room, and *prints* what it
// actually measured. The printed table is the artifact: run it before and after
// a change and the trade is visible.
//
// See tests/pll_metrics.h for what each number means.

#include "channel.h" // PLL_MAX_PULL
#include "clock_sync.h"
#include "config.h"
#include "hw_setup.h"
#include "pll_metrics.h"
#include "testkit.h"
#include <math.h>

// ~7MB. Static, not on the stack.
static PllTrace trace;
static Fixture fx;

// A channel's FRQ parameter for a given multiple of the beat. The mapping in
// channel_compute is: p >= 0 -> p + 1, p < 0 -> -1 / (p - 1), with p the stored
// value over 255. So x1 is 0, x2 is 255, and 1/2 is -255.
static int16_t frq_for_ratio(float ratio)
{
  if (ratio >= 1.0f)
    return (int16_t) lrintf((ratio - 1.0f) * 255.0f);
  return (int16_t) lrintf((1.0f - 1.0f / ratio) * 255.0f);
}

static void setup(Fixture* f, uint8_t ch, float ratio)
{
  fixture_init(f);
  fixture_set_param(f, ch, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(f, ch, 0, CH_PARAM_FRQ, frq_for_ratio(ratio));
}

// Let the clock establish a tempo before anything is measured. Two pulses give
// beat_freq an interval; a couple of beats give the smoother time to converge,
// so what follows is measuring the phase loop and not the tempo estimator.
static void warm_clock(Fixture* f, PllClock* c, uint8_t ch) { pll_run(f, c, ch, 4.0f, NULL); }

// The distance between two channels' output phases, on the circle: 0.5 is as
// far apart as two channels can be.
static float phase_gap(const Fixture* f, uint8_t a, uint8_t b)
{
  float d = fabsf(f->engine_state.channels_effective[a].phase - f->engine_state.channels_effective[b].phase);
  return d > 0.5f ? 1.0f - d : d;
}

// ---------------------------------------------------------------------------

// The base case: a channel at the beat rate, starting wherever it starts, has
// to arrive and stay.
TEST_CASE(a_channel_locks_to_the_beat_and_stays)
{
  PllClock clk;
  setup(&fx, 0, 1.0f);
  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);

  pll_trace_reset(&trace);
  trace.decimate = 1;
  pll_run(&fx, &clk, 0, 20.0f, &trace);

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("lock, x1 @ 120bpm", &m);

  CHECK(m.settle_s >= 0.0f);    // it arrives at all
  CHECK(m.settle_s < 10.0f);    // within something a musician would accept
  CHECK(m.crossings <= 2);      // it does not ring on the way
  CHECK(m.max_freq_dev < 1.0f); // it never runs at double its rate to get there
  CHECK(m.rms_err_tail_beats < PLL_TOL_BEATS);
}

// The user-facing case this suite exists for. Two scenes at different rates,
// the crossfader moved from one to the other: the target ratio changes under
// the channel and it has to re-acquire.
TEST_CASE(a_scene_transition_relocks)
{
  PllClock clk;
  fixture_init(&fx);
  fixture_set_param(&fx, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 1, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 0, CH_PARAM_FRQ, frq_for_ratio(1.0f));
  fixture_set_param(&fx, 0, 1, CH_PARAM_FRQ, frq_for_ratio(2.0f));
  fx.engine_config.scene_a = 0;
  fx.engine_config.scene_b = 1;
  fx.hw_state.slider_state = SLIDER_MAX_VALUE; // scene A

  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 8.0f, NULL); // settle on A

  // Mid-beat, where the two ratios disagree most. On a beat boundary every
  // ratio agrees and the transition costs nothing, which is a real property of
  // the module and a useless thing to measure.
  pll_run_to_beat_phase(&fx, &clk, 0, 0.5f);

  // Snap to B. A hard move is the worst case; a hand on the fader is gentler.
  fx.hw_state.slider_state = SLIDER_MIN_VALUE;

  pll_trace_reset(&trace);
  pll_run(&fx, &clk, 0, 20.0f, &trace);

  CHECK_NEAR(fx.engine_state.channels_effective[0].freq_ratio, 2.0, 0.01);

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("scene A->B, x1 -> x2 (snap)", &m);

  CHECK(m.peak_err_beats > 0.05f); // the disturbance was real
  CHECK(m.settle_s >= 0.0f);
  CHECK(m.settle_s < 10.0f);
  CHECK(m.crossings <= 3);
  CHECK(m.rms_err_tail_beats < PLL_TOL_BEATS);
}

// The same transition taken over a second, the way a hand moves a fader. The
// loop should have an easier time of it than the snap, not a harder one.
TEST_CASE(a_swept_scene_transition_is_no_worse_than_a_snap)
{
  PllClock clk;
  fixture_init(&fx);
  fixture_set_param(&fx, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 1, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 0, CH_PARAM_FRQ, frq_for_ratio(1.0f));
  fixture_set_param(&fx, 0, 1, CH_PARAM_FRQ, frq_for_ratio(2.0f));
  fx.engine_config.scene_a = 0;
  fx.engine_config.scene_b = 1;
  fx.hw_state.slider_state = SLIDER_MAX_VALUE;

  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 8.0f, NULL);
  pll_run_to_beat_phase(&fx, &clk, 0, 0.5f);

  // The sweep itself, measured on its own. While the fader is moving the target
  // ratio is moving with it and the phase the loop is chasing is not a fixed
  // point at all - so this window says what the *transition* costs, not what
  // settling costs, and the two are worth separating.
  pll_trace_reset(&trace);
  const uint32_t sweep_ticks = (uint32_t) (1e6f / (float) ENGINE_TICK_US);
  for (uint32_t i = 0; i < sweep_ticks; i++)
  {
    float u                  = (float) i / (float) sweep_ticks;
    fx.hw_state.slider_state = (int16_t) (SLIDER_MAX_VALUE + u * (SLIDER_MIN_VALUE - SLIDER_MAX_VALUE));
    pll_step(&fx, &clk, 0, ENGINE_TICK_US, &trace);
  }
  fx.hw_state.slider_state = SLIDER_MIN_VALUE;

  PllMetrics during = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("scene sweep, during the 1s move", &during);

  // ...and then what it takes to settle afterwards.
  pll_trace_reset(&trace);
  pll_run(&fx, &clk, 0, 20.0f, &trace);

  PllMetrics after = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("scene sweep, settling after", &after);

  CHECK(after.settle_s >= 0.0f);
  CHECK(after.settle_s < 12.0f);
  CHECK(after.rms_err_tail_beats < PLL_TOL_BEATS);

  // The invariant. See the phase-continuity case below for why this is the one
  // that matters most.
  CHECK(during.max_phase_jump < 0.01f);
  CHECK(after.max_phase_jump < 0.01f);

  // The artifact budget, and the whole point of the rework: a fader move must
  // not turn into a speed lurch. This was 1.398 before the alignment period was
  // latched - the oscillator briefly running at 2.4x its own rate.
  CHECK(during.max_freq_dev <= PLL_MAX_PULL * 1.05f);
  CHECK(during.gcd_changes <= 2);
  CHECK(during.peak_err_beats < 1.0f);
}

// The case the loop was reworked for.
//
// The target phase is `(beat_counter - origin) % gcd + beat_phase) * ratio`,
// and gcd comes from find_denominator, which is a search for a rational
// approximation. As the ratio moves continuously the answer does not: it jumps
// between 8, 7, 6, none, 5 and back. Each jump used to move the origin, the
// target teleported by whole beats, and the loop corrected for a step nothing
// had caused - 108 flips over this sweep, 3.6 beats of error, and the
// oscillator pulled to 1.7x its own rate.
//
// gcd is latched now and only re-taken at a super-period wrap, where the old
// and new periods agree on where the channel is. Measured after: 2 flips.
TEST_CASE(a_moving_ratio_does_not_flip_the_alignment_period)
{
  PllClock clk;
  fixture_init(&fx);
  fixture_set_param(&fx, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 1, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 0, CH_PARAM_FRQ, frq_for_ratio(1.0f));
  fixture_set_param(&fx, 0, 1, CH_PARAM_FRQ, frq_for_ratio(4.0f));
  fx.engine_config.scene_a = 0;
  fx.engine_config.scene_b = 1;
  fx.hw_state.slider_state = SLIDER_MAX_VALUE;

  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 8.0f, NULL);
  pll_run_to_beat_phase(&fx, &clk, 0, 0.5f);

  pll_trace_reset(&trace);
  const uint32_t ticks = (uint32_t) (3e6f / (float) ENGINE_TICK_US);
  for (uint32_t i = 0; i < ticks; i++)
  {
    float u                  = (float) i / (float) ticks;
    fx.hw_state.slider_state = (int16_t) (SLIDER_MAX_VALUE + u * (SLIDER_MIN_VALUE - SLIDER_MAX_VALUE));
    pll_step(&fx, &clk, 0, ENGINE_TICK_US, &trace);
  }

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("ratio sweep x1 -> x4 over 3s", &m);

  // A ratio moving across a factor of four crosses a great many rational
  // approximations. The alignment period may follow it, but only after it has
  // stopped moving - not once per approximation.
  CHECK(m.gcd_changes <= 4);
  CHECK(m.max_phase_jump < 0.01f);

  // And whatever it does, the oscillator is never pulled past the clamp.
  CHECK(m.max_freq_dev <= PLL_MAX_PULL * 1.05f);

  pll_run(&fx, &clk, 0, 20.0f, NULL);
  CHECK(fabsf(fx.engine_state.channels_effective[0].phase_error) < 0.05f);
}

// The phase accumulator may only ever wrap at a whole cycle.
//
// The waveform is a function of phase modulo one, so a wrap at exactly one
// cycle is invisible: the sample either side of it is the same sample. A wrap
// anywhere else is a step in the middle of the waveform - a click, and a broken
// edge on a scope.
//
// This is not theoretical. The accumulator used to wrap at the super-period,
// gcd * ratio cycles, which is only a whole number of cycles when the ratio is
// the rational multiple find_denominator says it is. That held while gcd was
// recomputed every tick (to within its 0.025 tolerance, so the step was there
// but small). Latching gcd while the ratio kept moving removed the guarantee,
// and a crossfade stepped the output by up to 0.34 of a cycle - which is what
// it sounded like.
//
// Asserted across every moving-ratio case in this file, because a sweep is
// where it appeared and where it would appear again.
TEST_CASE(the_phase_never_steps_mid_waveform)
{
  PllClock clk;
  fixture_init(&fx);
  fixture_set_param(&fx, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 1, CH_PARAM_AMP, 20000);
  fixture_set_param(&fx, 0, 0, CH_PARAM_FRQ, frq_for_ratio(0.75f));
  fixture_set_param(&fx, 0, 1, CH_PARAM_FRQ, frq_for_ratio(3.0f));
  fx.engine_config.scene_a = 0;
  fx.engine_config.scene_b = 1;
  fx.hw_state.slider_state = SLIDER_MAX_VALUE;

  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 8.0f, NULL);

  // Back and forth, at three speeds, landing at a different beat phase each
  // time - the fader move most likely to catch a wrap in an awkward place.
  pll_trace_reset(&trace);
  const float sweeps[3] = {0.3f, 1.0f, 2.5f};
  for (int s = 0; s < 3; s++)
  {
    for (int dir = 0; dir < 2; dir++)
    {
      uint32_t ticks = (uint32_t) (sweeps[s] * 1e6f / (float) ENGINE_TICK_US);
      for (uint32_t i = 0; i < ticks; i++)
      {
        float u                  = (float) i / (float) ticks;
        float pos                = dir ? (1.0f - u) : u;
        fx.hw_state.slider_state = (int16_t) (SLIDER_MAX_VALUE + pos * (SLIDER_MIN_VALUE - SLIDER_MAX_VALUE));
        pll_step(&fx, &clk, 0, ENGINE_TICK_US, &trace);
      }
      pll_run(&fx, &clk, 0, 1.7f, &trace);
    }
  }

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("fader worked back and forth", &m);

  // One tick of ordinary advance at this rate is well under 0.001 cycles, so
  // anything approaching a percent of a cycle is a step and not a rounding.
  CHECK(m.max_phase_jump < 0.01f);
}

// A step in phase with nothing else changing - the cleanest look at the loop's
// own dynamics. The channel is shoved off target and has to come back.
TEST_CASE(a_phase_step_settles_without_ringing)
{
  PllClock clk;
  setup(&fx, 0, 1.0f);
  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 12.0f, NULL);
  pll_run_to_beat_phase(&fx, &clk, 0, 0.5f);

  // Very nearly the largest error there is - not exactly it.
  //
  // Exactly half a cycle sits *on* the wrap point of phase_error, where +0.5
  // and -0.5 are the same place and which way the loop corrects is decided by
  // the last bit of the arithmetic. That is fine behaviour and unmeasurable
  // behaviour: the loop picks a side, the pick registers as a zero crossing,
  // and overshoot_beats - the largest error after the first crossing - then
  // reads the *undisturbed* error as overshoot and reports a ratio of 1.0.
  //
  // Which side it picks turns out to depend on the tick period, so at 250us
  // this read 0.0 and across 280-345us it read 1.0, for a loop doing the same
  // correct thing in both. Stepping just short of the boundary makes the
  // direction unambiguous and the metric mean what it says. Settling, ringing
  // and the tail are asserted below and cover the exactly-half case too.
  fx.engine_state.channels_shared_phase[0] += 0.45f;

  pll_trace_reset(&trace);
  pll_run(&fx, &clk, 0, 25.0f, &trace);

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("phase step, half a cycle", &m);

  CHECK(m.settle_s >= 0.0f);
  CHECK(m.settle_s < 2.5f);        // and quickly enough to read as a snap, not a drift
  CHECK(m.crossings <= 2);         // ringing is the thing to avoid here
  CHECK(m.overshoot_ratio < 0.5f); // and going well past it
  CHECK(m.rms_err_tail_beats < PLL_TOL_BEATS);

  // Even against the largest error there is, the channel is never pulled more
  // than PLL_MAX_PULL off its rate. That is what makes a big correction
  // inaudible rather than merely brief.
  CHECK(m.max_freq_dev <= PLL_MAX_PULL * 1.05f);
}

// A tempo change mid-run. The tempo estimate moves, so the loop is chasing a
// target that has both jumped and changed speed.
TEST_CASE(a_tempo_step_relocks)
{
  PllClock clk;
  setup(&fx, 0, 1.0f);
  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 12.0f, NULL);
  pll_run_to_beat_phase(&fx, &clk, 0, 0.5f);

  clk.bpm = 140.0f;

  pll_trace_reset(&trace);
  pll_run(&fx, &clk, 0, 25.0f, &trace);

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("tempo step, 120 -> 140bpm", &m);

  CHECK(m.settle_s >= 0.0f);
  CHECK(m.settle_s < 15.0f);
  CHECK(m.max_freq_dev < 1.0f);
  CHECK(m.rms_err_tail_beats < PLL_TOL_BEATS);
}

// The reason for the odd bits of the implementation: a ratio that is not a
// whole number of cycles per beat still has to come back to the same place
// every `gcd` beats, and stay doing it for as long as the patch runs.
//
// 3/4 of the beat rate: four beats is three cycles, so gcd is 4 and the channel
// is only back at phase 0 every fourth beat. Nothing shorter aligns, which is
// exactly what the beat_counter %% gcd extrapolation is for.
TEST_CASE(a_polyrhythmic_ratio_holds_its_alignment_over_the_long_run)
{
  PllClock clk;
  setup(&fx, 0, 0.75f);
  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);

  CHECK(fx.engine_state.channels_effective[0].gcd == 4);

  pll_run(&fx, &clk, 0, 10.0f, NULL); // acquire

  pll_trace_reset(&trace);
  trace.decimate = 8;
  pll_run(&fx, &clk, 0, 240.0f, &trace); // eight minutes of beats

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("x0.75, 240s alignment", &m);

  // No drift: the worst error late in the run is no worse than the RMS early.
  CHECK(m.max_err_tail_beats < 4.0f * PLL_TOL_BEATS);
  CHECK(m.rms_err_tail_beats < PLL_TOL_BEATS);

  // And it is still tracking, not merely quiet: something must have been locked
  // the whole way.
  uint32_t locked = 0;
  for (uint32_t i = 0; i < trace.n; i++)
    locked += trace.locked[i];
  CHECK(locked > trace.n - trace.n / 20);
}

// Which cycle of the pattern lands on the downbeat.
//
// A x2/3 channel is two cycles every three beats. Repeating every three beats
// is necessary but not sufficient: the channel also has to come back to phase 0
// on the *same* beat of the three, every time. There are three phases that all
// satisfy "repeats every three beats" - 0, 1/3 and 2/3 of a cycle off the bar -
// and only one of them is the alignment that was asked for.
//
// Nothing asserted this, and reducing the loop's error to a single cycle -
// which is otherwise an appealing simplification, since the waveform repeats -
// silently picks whichever of the three is nearest. Caught by ear on hardware
// before this test existed.
TEST_CASE(the_pattern_lands_on_the_same_beat_every_time)
{
  PllClock clk;
  setup(&fx, 0, 2.0f / 3.0f);
  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);

  CHECK(fx.engine_state.channels_effective[0].gcd == 3);

  pll_run(&fx, &clk, 0, 12.0f, NULL); // acquire

  pll_trace_reset(&trace);
  trace.decimate = 1;
  pll_run(&fx, &clk, 0, 30.0f, &trace);

  // Against the clock's own grid: the channel has no origin of its own, which
  // is what stops two channels at one ratio disagreeing about where the period
  // starts. See two_channels_at_the_same_rate_share_a_phase.
  const uint64_t origin = 0;

  // At the top of every third beat the channel must be at phase 0; at the top
  // of the other two it must not be, or the pattern is not three beats long.
  uint32_t on_pattern = 0, off_pattern = 0;

  for (uint32_t i = 1; i < trace.n; i++)
  {
    if (trace.beat[i] == trace.beat[i - 1])
      continue; // only the tick a beat turns over

    uint64_t k      = (trace.beat[i] - origin) % 3u;
    float ph        = trace.phase[i];
    float from_zero = fminf(ph, 1.0f - ph);

    if (k == 0)
    {
      CHECK(from_zero < 0.02f);
      on_pattern++;
    }
    else
    {
      CHECK(from_zero > 0.1f);
      off_pattern++;
    }
  }

  CHECK(on_pattern > 15);
  CHECK(off_pattern > 30);
}

// The same, at a ratio with no whole-beat period at all. find_denominator gives
// up (gcd <= 0), the loop stops correcting, and the channel free-runs at the
// right rate. That is the intended behaviour and it must not drift wildly or
// start fighting itself.
TEST_CASE(a_ratio_with_no_beat_period_free_runs_cleanly)
{
  PllClock clk;
  setup(&fx, 0, 1.0f / 3.0f * 1.11f); // deliberately not a simple fraction
  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);

  pll_trace_reset(&trace);
  trace.decimate = 4;
  pll_run(&fx, &clk, 0, 60.0f, &trace);

  // Nothing to lock to, so no correction should be applied at all.
  CHECK(fx.engine_state.channels_effective[0].gcd <= 0);
  CHECK_NEAR(fx.engine_state.channels_phase_correction[0], 0.0, 1e-6);

  // ...and it still oscillates at the rate it was asked for.
  float ratio = fx.engine_state.channels_effective[0].freq_ratio;
  CHECK_NEAR(fx.engine_state.channels_effective[0].freq_hz, 2.0 * (double) ratio, 0.02);
}

// A real clock jitters. The loop should absorb it rather than pass it through
// to the oscillator, which is what "smoothness" means when the input is noisy.
TEST_CASE(clock_jitter_is_absorbed_not_passed_through)
{
  PllClock clk;
  setup(&fx, 0, 1.0f);
  pll_clock_init(&clk, 120.0f);
  clk.jitter = 0.05f; // 5% of a pulse period
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 10.0f, NULL);

  pll_trace_reset(&trace);
  pll_run(&fx, &clk, 0, 30.0f, &trace);

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("x1, 5% clock jitter", &m);

  // The phase wobbles, but bounded, and the oscillator is not being swung
  // around by every late pulse.
  CHECK(m.max_err_tail_beats < 0.1f);
  CHECK(m.max_freq_dev < 0.5f);
}

// Clock stops, module free-runs at the tempo it last saw, clock comes back.
// Nothing here should produce a jump.
TEST_CASE(losing_and_regaining_the_clock_relocks)
{
  PllClock clk;
  setup(&fx, 0, 1.0f);
  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 12.0f, NULL);

  clk.running = 0;
  pll_run(&fx, &clk, 0, 6.0f, NULL);
  CHECK(!fx.engine_state.clock.have_beat);

  // Back on, at the same tempo, from wherever the phase happens to be.
  clk.running = 1;
  clk.next_us = fx.hw_state.time;

  pll_trace_reset(&trace);
  pll_run(&fx, &clk, 0, 25.0f, &trace);

  PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
  pll_report("clock lost 6s, then back", &m);

  CHECK(m.settle_s >= 0.0f);
  CHECK(m.settle_s < 15.0f);
  CHECK(m.rms_err_tail_beats < PLL_TOL_BEATS);
}

// Eight channels at eight ratios, all locking at once. They share a clock and
// nothing else; one channel's correction must not show up in another's.
TEST_CASE(channels_lock_independently)
{
  PllClock clk;
  const float ratios[N_CHANNELS] = {1.0f, 2.0f, 4.0f, 0.5f, 0.25f, 0.75f, 3.0f, 1.5f};

  fixture_init(&fx);
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    fixture_set_param(&fx, c, 0, CH_PARAM_AMP, 20000);
    fixture_set_param(&fx, c, 0, CH_PARAM_FRQ, frq_for_ratio(ratios[c]));
  }

  pll_clock_init(&clk, 120.0f);
  warm_clock(&fx, &clk, 0);
  pll_run(&fx, &clk, 0, 30.0f, NULL);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    const ChannelEffective* eff = &fx.engine_state.channels_effective[c];
    float err_beats             = (eff->freq_ratio != 0.0f) ? eff->phase_error / eff->freq_ratio : 0.0f;

    // The stored parameter is 8-bit, so a requested 1.5 lands on 1.502.
    CHECK_NEAR(eff->freq_ratio, (double) ratios[c], 0.01);
    CHECK(fabsf(err_beats) < 4.0f * PLL_TOL_BEATS);
  }
}

// Two channels set to the same rate must sit at the same phase. It is the
// module's most basic promise, and the one a user checks by eye.
//
// Broken by the alignment-period rework. The target is measured from a
// *per-channel* `beat_origin`, and the origin may only move where the channel's
// own old period says the move is seamless: `(beat_counter - origin) % gcd == 0`.
// At gcd 1 that test is vacuously true, so a channel coming back from x1 to
// x1/2 re-bases its origin on whatever beat the fader happened to settle on.
// An origin one beat out at x1/2 is half a cycle - two identical channels, both
// locked, both stable, exactly 180 degrees apart. A quarter cycle at x1/4.
//
// The detour length is swept in whole beats because the failure is a parity:
// half the return beats land on the right grid by luck.
TEST_CASE(two_channels_at_the_same_rate_share_a_phase)
{
  // x1/2 puts a one-beat error at half a cycle, x1/4 at a quarter - the two
  // the module was heard doing.
  const float rates[2] = {0.5f, 0.25f};

  for (int r = 0; r < 2; r++)
    for (int extra_beats = 0; extra_beats < 4; extra_beats++)
    {
      PllClock clk;
      fixture_init(&fx);

      // ch0 never moves: the same rate in both scenes. ch1 is that rate in A and
      // x1 in B, so the fader takes it away and brings it back while ch0 stays.
      for (uint8_t c = 0; c < 2; c++)
      {
        fixture_set_param(&fx, c, 0, CH_PARAM_AMP, 20000);
        fixture_set_param(&fx, c, 1, CH_PARAM_AMP, 20000);
        fixture_set_param(&fx, c, 0, CH_PARAM_FRQ, frq_for_ratio(rates[r]));
        fixture_set_param(&fx, c, 1, CH_PARAM_FRQ, frq_for_ratio(c == 0 ? rates[r] : 1.0f));
      }
      fx.engine_config.scene_a = 0;
      fx.engine_config.scene_b = 1;
      fx.hw_state.slider_state = SLIDER_MAX_VALUE; // scene A

      pll_clock_init(&clk, 120.0f);
      warm_clock(&fx, &clk, 0);
      pll_run(&fx, &clk, 0, 16.0f, NULL);

      float before = phase_gap(&fx, 0, 1);
      CHECK(before < 0.02f); // they start together, or the case measures nothing

      fx.hw_state.slider_state = SLIDER_MIN_VALUE; // ch1 -> x1
      pll_run(&fx, &clk, 0, 6.0f + 0.5f * (float) extra_beats, NULL);

      fx.hw_state.slider_state = SLIDER_MAX_VALUE; // ch1 -> x1/2 again
      pll_run(&fx, &clk, 0, 24.0f, NULL);

      float after = phase_gap(&fx, 0, 1);
      fprintf(stdout, "  x%.2f, detour +%d beats: gap before %.4f, after %.4f cycles\n", (double) rates[r], extra_beats, (double) before,
              (double) after);

      CHECK_NEAR(fx.engine_state.channels_effective[1].freq_ratio, (double) rates[r], 0.01);
      CHECK(after < 0.02f);
    }
}

// A channel must settle at every rate it is given, including one it reaches
// from another rate with the same denominator.
//
// The alignment rational is q beats to p cycles. p was only ever taken inside
// the branch that saw q change, so a move between two ratios sharing a q never
// took it: x3/4 and x1/4 both answer q 4, and the channel kept x3/4's p of 3.
// That left the loop comparing a target confined to one cycle against a
// position roaming over three, so the correction sat on the PLL_MAX_PULL clamp
// and crossed back and forth for as long as the patch was left alone - the
// slow wander heard on the module. p was also never taken on the first
// acquisition at all, so it was 1 for the life of any channel whose ratio never
// moved, and only ever correct by luck.
TEST_CASE(a_rate_change_that_keeps_its_denominator_still_settles)
{
  // Both members of each pair answer the same q, which is the whole point.
  const float pairs[3][2] = {{0.75f, 0.25f}, {0.25f, 0.75f}, {1.0f, 4.0f}};

  for (int i = 0; i < 3; i++)
  {
    PllClock clk;
    fixture_init(&fx);
    fixture_set_param(&fx, 0, 0, CH_PARAM_AMP, 20000);
    fixture_set_param(&fx, 0, 1, CH_PARAM_AMP, 20000);
    fixture_set_param(&fx, 0, 0, CH_PARAM_FRQ, frq_for_ratio(pairs[i][0]));
    fixture_set_param(&fx, 0, 1, CH_PARAM_FRQ, frq_for_ratio(pairs[i][1]));
    fx.engine_config.scene_a = 0;
    fx.engine_config.scene_b = 1;
    fx.hw_state.slider_state = SLIDER_MAX_VALUE;

    pll_clock_init(&clk, 120.0f);
    warm_clock(&fx, &clk, 0);
    pll_run(&fx, &clk, 0, 16.0f, NULL);

    fx.hw_state.slider_state = SLIDER_MIN_VALUE;
    pll_run(&fx, &clk, 0, 20.0f, NULL); // well past any acquisition transient

    pll_trace_reset(&trace);
    trace.decimate = 1;
    pll_run(&fx, &clk, 0, 20.0f, &trace);
    PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);

    fprintf(stdout, "  x%.2f -> x%.2f: q %d p %d, rms_tail %.5f beats, fdev %.3f\n", (double) pairs[i][0], (double) pairs[i][1],
            fx.engine_state.channels_gcd[0], fx.engine_state.channels_period_cycles[0], (double) m.rms_err_tail_beats,
            (double) m.max_freq_dev);

    CHECK(m.rms_err_tail_beats < PLL_TOL_BEATS); // it is settled, not circling
    CHECK(m.max_freq_dev < 0.01f);               // and not leaning on the clamp
    CHECK(m.crossings <= 2);
  }
}

// The loop's smoothing is a fraction applied once per tick, so its time
// constant is a number of ticks rather than a number of seconds. Two hosts at
// different control rates therefore get different loops out of the same code.
// Documented as a measurement rather than asserted: it is a property of the
// current implementation, and one worth removing.
TEST_CASE(the_loop_response_depends_on_the_control_rate)
{
  PllClock clk;
  float settle[2];
  const uint32_t rates[2] = {ENGINE_TICK_US, ENGINE_TICK_US * 4};

  for (int r = 0; r < 2; r++)
  {
    setup(&fx, 0, 1.0f);
    pll_clock_init(&clk, 120.0f);

    for (uint32_t i = 0; i < (uint32_t) (16e6f / (float) rates[r]); i++)
      pll_step(&fx, &clk, 0, rates[r], NULL);

    fx.engine_state.channels_shared_phase[0] += 0.5f;

    pll_trace_reset(&trace);
    for (uint32_t i = 0; i < (uint32_t) (25e6f / (float) rates[r]); i++)
      pll_step(&fx, &clk, 0, rates[r], &trace);

    PllMetrics m = pll_measure(&trace, PLL_TOL_BEATS);
    settle[r]    = m.settle_s;

    char name[64];
    snprintf(name, sizeof(name), "phase step @ %uus tick", rates[r]);
    pll_report(name, &m);
  }

  // Both still lock - the point is only that they do it differently.
  CHECK(settle[0] >= 0.0f);
  CHECK(settle[1] >= 0.0f);
}

int main(void)
{
  pll_report_header();

  RUN_TEST(a_channel_locks_to_the_beat_and_stays);
  RUN_TEST(a_scene_transition_relocks);
  RUN_TEST(a_swept_scene_transition_is_no_worse_than_a_snap);
  RUN_TEST(a_moving_ratio_does_not_flip_the_alignment_period);
  RUN_TEST(the_phase_never_steps_mid_waveform);
  RUN_TEST(a_phase_step_settles_without_ringing);
  RUN_TEST(a_tempo_step_relocks);
  RUN_TEST(a_polyrhythmic_ratio_holds_its_alignment_over_the_long_run);
  RUN_TEST(the_pattern_lands_on_the_same_beat_every_time);
  RUN_TEST(a_ratio_with_no_beat_period_free_runs_cleanly);
  RUN_TEST(clock_jitter_is_absorbed_not_passed_through);
  RUN_TEST(losing_and_regaining_the_clock_relocks);
  RUN_TEST(channels_lock_independently);
  RUN_TEST(two_channels_at_the_same_rate_share_a_phase);
  RUN_TEST(a_rate_change_that_keeps_its_denominator_still_settles);
  RUN_TEST(the_loop_response_depends_on_the_control_rate);

  fprintf(stdout, "\n");
  return TESTKIT_SUMMARY();
}
