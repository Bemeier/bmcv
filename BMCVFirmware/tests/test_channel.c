#include "clock_sync.h"
#include "fixture.h"
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

int main(void)
{
RUN_TEST(zero_amplitude_channel_outputs_the_offset);
RUN_TEST(amplitude_bounds_the_output_around_the_offset);
RUN_TEST(input_add_mode_adds_the_raw_input_value);
RUN_TEST(input_mult_mode_zero_input_zeroes_the_output);
RUN_TEST(continuous_quantize_matches_quantize_value);
RUN_TEST(phase_advances_by_frequency_times_dt_without_pll_lock);
  return TESTKIT_SUMMARY();
}
