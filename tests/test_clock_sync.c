#include "clock_sync.h"
#include "testkit.h"
#include <math.h>

TEST_CASE(beat_freq_smooth_converges_to_the_pulse_rate)
{
  ClockState clk;
  Clock_Init(&clk);

  const uint32_t dt_pulse = 125000; // 4 pulses/250ms beat -> 2.0 Hz beat rate
  uint32_t t              = 0;
  for (int i = 0; i < 30; i++)
  {
    t += dt_pulse;
    Clock_Trigger(&clk, t);
  }

  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);
  CHECK(clk.have_beat);
}

TEST_CASE(have_beat_drops_after_clock_loss)
{
  ClockState clk;
  Clock_Init(&clk);

  Clock_Trigger(&clk, 0);
  Clock_Trigger(&clk, 125000);
  CHECK(clk.have_beat);

  Clock_Poll(&clk, 125000 + 4 * 125000 + 1);

  CHECK(!clk.have_beat);
}

// Two clocks in one process must not see each other. This is what makes N
// module instances possible in a single VCV Rack patch.
TEST_CASE(clocks_are_independent)
{
  ClockState a, b;
  Clock_Init(&a);
  Clock_Init(&b);

  uint32_t t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 125000; // 2.0 Hz
    Clock_Trigger(&a, t);
  }

  t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 250000; // 1.0 Hz
    Clock_Trigger(&b, t);
  }

  CHECK_NEAR(a.beat_freq_smooth, 2.0, 0.02);
  CHECK_NEAR(b.beat_freq_smooth, 1.0, 0.02);
  CHECK(a.have_beat);
  CHECK(b.have_beat);

  // Losing one clock does not drop the other.
  Clock_Poll(&b, t + 4 * 250000 + 1);
  CHECK(!b.have_beat);
  CHECK(a.have_beat);
}

// A reset and the pulses that follow it used to produce 1e6 / 0: the interval
// is only measured outside the post-reset guard, but the divide was not gated
// on having measured one. freq_est is a leaky integrator, so the inf never
// washed out - the module ran at an infinite tempo until it was power-cycled.
TEST_CASE(a_pulse_inside_the_reset_guard_does_not_divide_by_zero)
{
  ClockState clk;
  Clock_Init(&clk);

  Clock_Reset(&clk, 0);
  Clock_Trigger(&clk, 500);
  Clock_Trigger(&clk, 1500);

  CHECK(isfinite(clk.beat_freq));
  CHECK(isfinite(clk.beat_freq_smooth));
  CHECK(isfinite(clk.bpm));

  // ...and a sane clock afterwards is still measured correctly, which is the
  // half a clamp would not have given: an inf that saturates is still an inf.
  uint32_t t = 10000;
  for (int i = 0; i < 30; i++)
  {
    t += 125000;
    Clock_Trigger(&clk, t);
  }
  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);
}

// The same divide, reached from the other side: a reset arriving mid-run must
// not let the pulse after it measure an interval that spans the reset.
TEST_CASE(a_reset_mid_run_does_not_corrupt_the_tempo)
{
  ClockState clk;
  Clock_Init(&clk);

  uint32_t t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 125000;
    Clock_Trigger(&clk, t);
  }
  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);

  Clock_Reset(&clk, t + 60000);
  for (int i = 0; i < 8; i++)
  {
    t += 125000;
    Clock_Trigger(&clk, t);
  }

  CHECK(isfinite(clk.beat_freq_smooth));
  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);
}

// Contact bounce, or an audio-rate signal patched into the clock jack. Without
// the minimum interval a pair of edges 50us apart is a 5kHz beat, and at
// alpha 0.3 the whole rack of LFOs sits at audio rate for several pulses.
TEST_CASE(bounce_does_not_move_the_tempo)
{
  ClockState clk;
  Clock_Init(&clk);

  uint32_t t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 125000;
    Clock_Trigger(&clk, t);
  }
  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);

  uint64_t beats_before = clk.beat_counter;

  // One pulse, arriving as six edges over 250us.
  t += 125000;
  for (int i = 0; i < 6; i++)
  {
    Clock_Trigger(&clk, t + (uint32_t) i * 50u);
  }

  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);
  // and it counted once, not six times
  CHECK(clk.beat_counter - beats_before <= 1);
}

// A tempo outside the plausible range is a glitch, and a glitch must not
// survive in the estimate.
TEST_CASE(an_implausible_interval_is_not_a_measurement)
{
  ClockState clk;
  Clock_Init(&clk);

  uint32_t t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 125000;
    Clock_Trigger(&clk, t);
  }

  // 2ms apart: past the bounce guard, but 7500 BPM.
  t += 2000;
  Clock_Trigger(&clk, t);

  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);
  CHECK(clk.beat_freq * 60.0f <= CLOCK_BPM_MAX);
}

// beat_phase leaves here for channel.c's target-phase term, where it is added
// to a beat count and multiplied by the ratio - so a value outside [0,1) is
// whole beats of phase error applied to all eight channels at once.
//
// The source changes under it whenever an input is assigned to INPUT_CLOCK
// while MIDI Clock is running: input_fold keys off the configuration, not off
// whether a cable is patched, so the switch lands on the very next tick. The
// pulse count is in the old source's units, and assigning PULSES_PER_BEAT
// without dropping it left 16 of the 24 reachable counts reading between 1.0
// and 4.75 until the new source sent a pulse.
TEST_CASE(a_source_change_cannot_put_beat_phase_out_of_range)
{
  for (unsigned pc = 0; pc < CLOCK_PULSES_PER_BEAT_MIDI; pc++)
  {
    ClockState clk;
    Clock_Init(&clk);
    Clock_SetPulsesPerBeat(&clk, CLOCK_PULSES_PER_BEAT_MIDI);

    // Run on MIDI until the counter sits at pc, so every reachable value of it
    // is covered rather than whichever one a fixed pulse count happens to give.
    uint32_t t = 0;
    for (int i = 0; i < 500 && clk.pulse_counter != pc; i++)
    {
      t += 20833; // ~2Hz beat at 24 ppb
      Clock_Trigger(&clk, t);
    }
    if (clk.pulse_counter != pc)
      continue; // not a reachable count

    Clock_SetPulsesPerBeat(&clk, CLOCK_PULSES_PER_BEAT_CV);

    // The engine polls every tick, and the new source has not sent a pulse yet.
    for (int i = 0; i < 8; i++)
    {
      t += 250;
      Clock_Poll(&clk, t);
      CHECK(clk.beat_phase >= 0.0f && clk.beat_phase < 1.0f);
    }
  }
}

// The counter is what carries the stale units, so it has to go; the measured
// interval is what tells Clock_Poll a stopped clock has stopped, so it stays -
// the same split Clock_Reset makes, for the same reason.
TEST_CASE(a_source_change_drops_the_pulse_count_but_not_the_timeout)
{
  ClockState clk;
  Clock_Init(&clk);
  Clock_SetPulsesPerBeat(&clk, CLOCK_PULSES_PER_BEAT_MIDI);

  uint32_t t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 20833;
    Clock_Trigger(&clk, t);
  }
  CHECK(clk.pulse_counter > 0);
  CHECK(clk.last_pulse_delta_us > 0);

  Clock_SetPulsesPerBeat(&clk, CLOCK_PULSES_PER_BEAT_CV);

  CHECK(clk.PULSES_PER_BEAT == CLOCK_PULSES_PER_BEAT_CV);
  CHECK(clk.pulse_counter == 0);
  CHECK(!clk.have_pulse);
  CHECK(clk.last_pulse_delta_us > 0);

  // and a clock that has stopped still reads as stopped
  Clock_Poll(&clk, t + 4 * clk.last_pulse_delta_us + 1);
  CHECK(!clk.have_beat);
}

// Called every tick with the same value, so it must not be a reset in disguise:
// re-setting the value it already holds has to leave the count running.
TEST_CASE(setting_the_same_pulses_per_beat_is_a_no_op)
{
  ClockState clk;
  Clock_Init(&clk);

  uint32_t t = 0;
  for (int i = 0; i < 6; i++)
  {
    t += 125000;
    Clock_Trigger(&clk, t);
  }

  uint32_t counter = clk.pulse_counter;
  bool pulsed      = clk.have_pulse;

  for (int i = 0; i < 10; i++)
    Clock_SetPulsesPerBeat(&clk, CLOCK_PULSES_PER_BEAT_CV);

  CHECK(clk.pulse_counter == counter);
  CHECK(clk.have_pulse == pulsed);
  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);
}

int main(void)
{
  RUN_TEST(a_source_change_cannot_put_beat_phase_out_of_range);
  RUN_TEST(a_source_change_drops_the_pulse_count_but_not_the_timeout);
  RUN_TEST(setting_the_same_pulses_per_beat_is_a_no_op);
  RUN_TEST(beat_freq_smooth_converges_to_the_pulse_rate);
  RUN_TEST(have_beat_drops_after_clock_loss);
  RUN_TEST(clocks_are_independent);
  RUN_TEST(a_pulse_inside_the_reset_guard_does_not_divide_by_zero);
  RUN_TEST(a_reset_mid_run_does_not_corrupt_the_tempo);
  RUN_TEST(bounce_does_not_move_the_tempo);
  RUN_TEST(an_implausible_interval_is_not_a_measurement);
  return TESTKIT_SUMMARY();
}
