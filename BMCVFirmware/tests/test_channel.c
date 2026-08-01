#include "clock_sync.h"
#include "fixture.h"
#include "stepped_random_table.h"
#include "helpers.h"
#include "testkit.h"

TEST_CASE(zero_amplitude_channel_outputs_the_offset)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 1234);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);

  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_output_level[0] == 1234);
}

TEST_CASE(amplitude_bounds_the_output_around_the_offset)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 0);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0); // multiplier 1x
  Clock_Init();

  for (int i = 0; i < 50; i++)
  {
    fixture_tick(&f, 20000);
    int16_t v = f.engine_state.channels_output_level[0];
    CHECK(v >= -10001 && v <= 10001);
  }
}

TEST_CASE(input_add_mode_adds_the_raw_input_value)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 1000);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);
  f.engine_config.channel_state[0].src_input     = 0;
  f.engine_config.channel_state[0].input_amp_mode = INPUT_AMP_ADD;
  f.hw_state.input_state[0]                       = 500;

  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_output_level[0] == 1500);
}

TEST_CASE(input_mult_mode_zero_input_zeroes_the_output)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 1000);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);
  f.engine_config.channel_state[0].src_input     = 0;
  f.engine_config.channel_state[0].input_amp_mode = INPUT_AMP_MULT;
  f.hw_state.input_state[0]                       = 0;

  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_output_level[0] == 0);
}

TEST_CASE(continuous_quantize_matches_quantize_value)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 5000);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);
  f.engine_config.channel_state[0].quantize_mode = QUANTIZE_CONTINUOUS;
  f.engine_config.quantize_mask                  = 0b111111111111;

  fixture_tick(&f, 1000);

  int16_t expected = quantize_value(5000, f.engine_config.quantize_mask);
  CHECK(f.engine_state.channels_output_level[0] == expected);
}

TEST_CASE(phase_advances_by_frequency_times_dt_without_pll_lock)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0); // multiplier 1x
  Clock_Init();                                 // have_beat=false, beat_freq_smooth=1.0Hz, no PLL correction applied

  fixture_tick(&f, 500000); // 0.5s at 1Hz -> phase advances by 0.5

  CHECK_NEAR(f.engine_state.channels_shared_phase[0], 0.5, 1e-4);
}


// ---- pattern-length latching (stepped modes) -------------------------------
//
// Switching pattern length mid-cycle moves the step grid under the playhead
// and jumps the output. So it is held until the cycle wraps - except while the
// encoder is being turned, where instant feedback matters more.

static void set_stepped_channel(Fixture* f, int16_t mod_param)
{
  f->engine_config.channel_state[0].shape_mode = SHAPE_STEPPED_HARD;
  fixture_set_param(f, 0, 0, CH_PARAM_FRQ, 0);   // 1x the beat
  fixture_set_param(f, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(f, 0, 0, CH_PARAM_MOD, mod_param);
}

TEST_CASE(pattern_length_is_latched_until_the_cycle_wraps)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init();
  set_stepped_channel(&f, sr_length_param[0]); // shortest pattern
  fixture_tick(&f, 1000);
  int8_t latched = f.engine_state.channels_length_idx[0];
  CHECK(latched == 0);

  // move well past the edit window so this counts as a scene-style change
  for (int i = 0; i < 40; i++)
    fixture_tick(&f, 20000); // 0.8s, but phase only reaches ~0.8 at 1Hz

  fixture_set_param(&f, 0, 0, CH_PARAM_MOD, sr_length_param[SR_LENGTH_COUNT - 1]);
  fixture_tick(&f, 1000);

  // still mid-cycle, so the length must not have moved yet
  CHECK(f.engine_state.channels_length_idx[0] == 0);
  CHECK(f.engine_state.channels_shared_phase[0] < 1.0f);
}

TEST_CASE(pattern_length_updates_once_the_cycle_wraps)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init();
  set_stepped_channel(&f, sr_length_param[0]);
  fixture_tick(&f, 1000);
  for (int i = 0; i < 40; i++)
    fixture_tick(&f, 20000);

  fixture_set_param(&f, 0, 0, CH_PARAM_MOD, sr_length_param[SR_LENGTH_COUNT - 1]);
  // run past the wrap
  for (int i = 0; i < 80; i++)
    fixture_tick(&f, 20000);

  CHECK(f.engine_state.channels_length_idx[0] == SR_LENGTH_COUNT - 1);
}

TEST_CASE(pattern_length_applies_immediately_while_the_encoder_is_turning)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init();
  set_stepped_channel(&f, sr_length_param[0]);
  fixture_tick(&f, 1000);
  for (int i = 0; i < 40; i++)
    fixture_tick(&f, 20000);
  CHECK(f.engine_state.channels_length_idx[0] == 0);

  // mark the channel as just-edited, the way update_channel_param() does
  f.engine_state.channels_last_delta[0] = f.hw_state.time;
  fixture_set_param(&f, 0, 0, CH_PARAM_MOD, sr_length_param[SR_LENGTH_COUNT - 1]);
  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_length_idx[0] == SR_LENGTH_COUNT - 1);
  CHECK(f.engine_state.channels_shared_phase[0] < 1.0f); // proved it did not wait for a wrap
}

int main(void)
{
RUN_TEST(zero_amplitude_channel_outputs_the_offset);
RUN_TEST(amplitude_bounds_the_output_around_the_offset);
RUN_TEST(input_add_mode_adds_the_raw_input_value);
RUN_TEST(input_mult_mode_zero_input_zeroes_the_output);
RUN_TEST(continuous_quantize_matches_quantize_value);
RUN_TEST(phase_advances_by_frequency_times_dt_without_pll_lock);
  RUN_TEST(pattern_length_is_latched_until_the_cycle_wraps);
  RUN_TEST(pattern_length_updates_once_the_cycle_wraps);
  RUN_TEST(pattern_length_applies_immediately_while_the_encoder_is_turning);
  return TESTKIT_SUMMARY();
}
