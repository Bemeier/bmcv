// Host-side runtime glue: unit conversion, tick decimation, gate latching.
//
// This is the code every host shares - the wasm sim, the CLI, and eventually
// the VCV Rack plugin - so getting it wrong is wrong everywhere at once. It is
// also the part that cannot be tested through a host, which is exactly why it
// lives in its own translation unit.

#include "hw_setup.h"
#include "sim_rt.h"
#include "testkit.h"
#include <math.h>

/* ---- unit conversion ---------------------------------------------------- */

TEST_CASE(dac_counts_map_to_volts)
{
  CHECK_NEAR(sim_dac_to_volts(0), 0.0, 1e-6);
  CHECK_NEAR(sim_dac_to_volts(DAC_5V), 5.0, 1e-6);
  CHECK_NEAR(sim_dac_to_volts(-DAC_5V), -5.0, 1e-6);

  // DAC_10V is the scale factor, not an attainable count: the converter is
  // 16-bit signed, so -10V is exactly reachable and +10V is one count short.
  CHECK_NEAR(sim_dac_to_volts(INT16_MIN), -10.0, 1e-6);
  CHECK_NEAR(sim_dac_to_volts(INT16_MAX), 9.99969, 1e-4);
}

TEST_CASE(volts_map_to_adc_counts_and_clamp)
{
  CHECK(sim_volts_to_adc(0.0f) == 0);
  CHECK(sim_volts_to_adc(10.0f) == ADC_10V - 1); // 14-bit signed tops out one short
  CHECK(sim_volts_to_adc(-10.0f) == -ADC_10V);
  CHECK(sim_volts_to_adc(5.0f) == ADC_5V);
  CHECK(sim_volts_to_adc(-5.0f) == -ADC_5V);

  // Beyond the rails the converter saturates rather than wrapping.
  CHECK(sim_volts_to_adc(50.0f) == ADC_10V - 1);
  CHECK(sim_volts_to_adc(-50.0f) == -ADC_10V);
}

TEST_CASE(volt_round_trips_stay_within_one_count)
{
  for (float v = -10.0f; v <= 10.0f; v += 0.25f)
  {
    float back = sim_adc_to_volts(sim_volts_to_adc(v));
    CHECK_NEAR(back, v, 10.0 / (double) ADC_10V);
  }
}

/* ---- tick decimation ---------------------------------------------------- */

TEST_CASE(divider_is_chosen_for_the_sample_rate)
{
  SimTickDiv d;

  sim_tickdiv_config(&d, 48000.0f, 4000.0f);
  CHECK(d.divider == 12);

  sim_tickdiv_config(&d, 96000.0f, 4000.0f);
  CHECK(d.divider == 24);

  sim_tickdiv_config(&d, 192000.0f, 4000.0f);
  CHECK(d.divider == 48);

  // 44100/4000 = 11.025, so the nearest whole number of frames is 11 and the
  // engine really runs at 44100/11 = 4009Hz, not at 4000.
  sim_tickdiv_config(&d, 44100.0f, 4000.0f);
  CHECK(d.divider == 11);
  CHECK_NEAR(sim_tickdiv_rate_hz(&d), 44100.0 / 11.0, 0.5);

  // A control rate above the sample rate cannot mean less than every frame.
  sim_tickdiv_config(&d, 48000.0f, 100000.0f);
  CHECK(d.divider == 1);
}

TEST_CASE(a_tick_is_due_exactly_every_divider_frames)
{
  SimTickDiv d;
  sim_tickdiv_config(&d, 48000.0f, 4000.0f);

  int ticks = 0;
  for (int i = 0; i < 48000; i++) // one second of audio
  {
    if (sim_tickdiv_step(&d))
      ticks++;
  }

  CHECK(ticks == 4000);
  CHECK(d.now_us == 1000000); // and exactly one second elapsed
}

TEST_CASE(dt_us_sums_to_now_us)
{
  SimTickDiv d;
  sim_tickdiv_config(&d, 44100.0f, 4000.0f);

  uint64_t summed = 0;
  for (int i = 0; i < 44100 * 5; i++)
  {
    if (sim_tickdiv_step(&d))
      summed += d.dt_us;
  }

  // What the engine integrates must equal the timestamps it is handed, or the
  // two notions of time diverge.
  CHECK(summed == (uint64_t) d.now_us);
}

// The reason for the Q32 accumulator. A rounded 249us step at 44.1kHz would
// lose ~1.7s over ten minutes; the engine would run every LFO slow.
TEST_CASE(no_drift_over_ten_minutes_at_44k1)
{
  SimTickDiv d;
  sim_tickdiv_config(&d, 44100.0f, 4000.0f);

  const int64_t frames = 44100LL * 600LL; // 10 minutes
  for (int64_t i = 0; i < frames; i++)
  {
    sim_tickdiv_step(&d);
  }

  // Elapsed engine time must match real time to well under a millisecond.
  double expected_us = (double) frames * 1000000.0 / 44100.0;
  double err_us      = fabs((double) d.now_us - expected_us);

  // The only slack is the last partial tick (< 250us) plus rounding.
  CHECK(err_us < 300.0);
}

TEST_CASE(no_drift_over_ten_minutes_at_48k)
{
  SimTickDiv d;
  sim_tickdiv_config(&d, 48000.0f, 4000.0f);

  const int64_t frames = 48000LL * 600LL;
  for (int64_t i = 0; i < frames; i++)
  {
    sim_tickdiv_step(&d);
  }

  CHECK(d.now_us == 600000000u); // exact at this rate
}

/* ---- gate latching ------------------------------------------------------ */

TEST_CASE(a_gate_shorter_than_a_tick_is_still_seen)
{
  SimTrigLatch t;
  sim_trig_reset(&t);

  // A host samples CV per audio frame but only builds an InputSample per
  // control tick. A one-frame pulse in between must not be lost.
  sim_trig_sample(&t, 0, 0);
  sim_trig_sample(&t, 0, sim_volts_to_adc(5.0f)); // the pulse
  sim_trig_sample(&t, 0, 0);
  sim_trig_sample(&t, 0, 0);

  CHECK(sim_trig_take(&t, 0) == 1);
  CHECK(sim_trig_take(&t, 0) == 0); // consumed
}

TEST_CASE(a_held_gate_triggers_once)
{
  SimTrigLatch t;
  sim_trig_reset(&t);

  for (int i = 0; i < 100; i++)
  {
    sim_trig_sample(&t, 0, sim_volts_to_adc(5.0f));
  }

  CHECK(sim_trig_take(&t, 0) == 1);
  CHECK(sim_trig_take(&t, 0) == 0);

  // Still high: no retrigger.
  for (int i = 0; i < 100; i++)
  {
    sim_trig_sample(&t, 0, sim_volts_to_adc(5.0f));
  }
  CHECK(sim_trig_take(&t, 0) == 0);

  // Falls below the lower threshold, then rises again.
  sim_trig_sample(&t, 0, 0);
  sim_trig_sample(&t, 0, sim_volts_to_adc(5.0f));
  CHECK(sim_trig_take(&t, 0) == 1);
}

TEST_CASE(hysteresis_rejects_a_noisy_gate)
{
  SimTrigLatch t;
  sim_trig_reset(&t);

  sim_trig_sample(&t, 0, TRIG_THRESH);
  CHECK(sim_trig_take(&t, 0) == 1);

  // Wobbling between the two thresholds must not produce more triggers.
  for (int i = 0; i < 50; i++)
  {
    sim_trig_sample(&t, 0, TRIG_THRESH_LOW + 1);
    sim_trig_sample(&t, 0, TRIG_THRESH);
  }
  CHECK(sim_trig_take(&t, 0) == 0);
}

TEST_CASE(multiple_pulses_between_ticks_collapse_to_one)
{
  SimTrigLatch t;
  sim_trig_reset(&t);

  // The InputSample carries a flag, not a count, so several edges in one tick
  // are one trigger. Matches the hardware, where trig_flag is a single bit.
  for (int i = 0; i < 5; i++)
  {
    sim_trig_sample(&t, 0, sim_volts_to_adc(5.0f));
    sim_trig_sample(&t, 0, 0);
  }

  CHECK(sim_trig_take(&t, 0) == 1);
  CHECK(sim_trig_take(&t, 0) == 0);
}

TEST_CASE(channels_latch_independently)
{
  SimTrigLatch t;
  sim_trig_reset(&t);

  sim_trig_sample(&t, 2, sim_volts_to_adc(5.0f));

  CHECK(sim_trig_take(&t, 0) == 0);
  CHECK(sim_trig_take(&t, 1) == 0);
  CHECK(sim_trig_take(&t, 2) == 1);
  CHECK(sim_trig_take(&t, 3) == 0);
}

TEST_CASE(out_of_range_channels_are_ignored)
{
  SimTrigLatch t;
  sim_trig_reset(&t);

  sim_trig_sample(&t, N_INPUTS, sim_volts_to_adc(5.0f));
  sim_trig_fire(&t, 99);
  CHECK(sim_trig_take(&t, N_INPUTS) == 0);

  for (uint8_t c = 0; c < N_INPUTS; c++)
  {
    CHECK(sim_trig_take(&t, c) == 0);
  }
}

TEST_CASE(fire_produces_a_trigger_without_a_voltage)
{
  SimTrigLatch t;
  sim_trig_reset(&t);

  sim_trig_fire(&t, 1);
  CHECK(sim_trig_take(&t, 1) == 1);
  CHECK(sim_trig_take(&t, 1) == 0);
}

int main(void)
{
  RUN_TEST(dac_counts_map_to_volts);
  RUN_TEST(volts_map_to_adc_counts_and_clamp);
  RUN_TEST(volt_round_trips_stay_within_one_count);
  RUN_TEST(divider_is_chosen_for_the_sample_rate);
  RUN_TEST(a_tick_is_due_exactly_every_divider_frames);
  RUN_TEST(dt_us_sums_to_now_us);
  RUN_TEST(no_drift_over_ten_minutes_at_44k1);
  RUN_TEST(no_drift_over_ten_minutes_at_48k);
  RUN_TEST(a_gate_shorter_than_a_tick_is_still_seen);
  RUN_TEST(a_held_gate_triggers_once);
  RUN_TEST(hysteresis_rejects_a_noisy_gate);
  RUN_TEST(multiple_pulses_between_ticks_collapse_to_one);
  RUN_TEST(channels_latch_independently);
  RUN_TEST(out_of_range_channels_are_ignored);
  RUN_TEST(fire_produces_a_trigger_without_a_voltage);
  return TESTKIT_SUMMARY();
}
