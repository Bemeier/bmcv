#include "config_validate.h"
#include "fixture.h"
#include "testkit.h"
#include <string.h>

// A config this build would itself produce must survive untouched - the
// validator is a repair for foreign data, not a normaliser applied to healthy
// presets.
TEST_CASE(a_valid_config_is_passed_through_unchanged)
{
  Fixture f;
  fixture_init(&f);

  f.engine_config.scene_a       = 2;
  f.engine_config.scene_b       = 5;
  f.engine_config.quantize_mask = 0b101011010101;
  f.engine_config.input_mode[0] = INPUT_CLOCK;
  f.engine_config.input_mode[3] = INPUT_SLIDER;
  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    f.engine_config.channel_state[c].shape_mode     = SHAPE_STEPPED_SEMI;
    f.engine_config.channel_state[c].quantize_mode  = QUANTIZE_CONTINUOUS;
    f.engine_config.channel_state[c].input_amp_mode = INPUT_AMP_MULT;
    f.engine_config.channel_state[c].src_input      = 1;
    f.engine_config.channel_state[c].src_trig       = N_INPUTS + 2;
    fixture_set_param(&f, c, 0, CH_PARAM_OFS, (int16_t) (1000 + c));
  }

  EngineConfig before = f.engine_config;
  config_validate(&f.engine_config);

  CHECK(memcmp(&before, &f.engine_config, sizeof(EngineConfig)) == 0);
}

// The downgrade case: a preset written by a build with more shape modes than
// this one. Without clamping this indexes shape_mode_color[] past its end.
TEST_CASE(shape_mode_from_a_newer_build_is_brought_into_range)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.channel_state[0].shape_mode = 99;
  f.engine_config.channel_state[1].shape_mode = -5;

  config_validate(&f.engine_config);

  CHECK(f.engine_config.channel_state[0].shape_mode >= 0);
  CHECK(f.engine_config.channel_state[0].shape_mode < SHAPE_MODE_COUNT);
  CHECK(f.engine_config.channel_state[1].shape_mode >= 0);
  CHECK(f.engine_config.channel_state[1].shape_mode < SHAPE_MODE_COUNT);
}

TEST_CASE(out_of_range_modes_are_brought_into_range)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.channel_state[0].quantize_mode  = (ChannelQuantizeMode) 77;
  f.engine_config.channel_state[0].input_amp_mode = (ChannelInputAmpMode) -3;
  f.engine_config.scene_a                         = 200;
  f.engine_config.scene_b                         = 99;
  f.engine_config.input_mode[2]                   = (InputMode) 41;

  config_validate(&f.engine_config);

  CHECK(f.engine_config.channel_state[0].quantize_mode < QUANTIZE_MODE_COUNT);
  CHECK(f.engine_config.channel_state[0].input_amp_mode < INPUT_AMP_MODE_COUNT);
  CHECK(f.engine_config.scene_a < N_SCENES);
  CHECK(f.engine_config.scene_b < N_SCENES);
  CHECK(f.engine_config.input_mode[2] < INPUT_MODE_COUNT);
}

// Routing must fall back to "unassigned" rather than be clamped onto a real
// source, which would silently route a channel somewhere never chosen.
TEST_CASE(out_of_range_routing_becomes_unassigned_not_clamped)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.channel_state[0].src_input = 42;
  f.engine_config.channel_state[0].src_trig  = 120;
  f.engine_config.channel_state[1].src_input = -7;
  f.engine_config.channel_state[1].src_trig  = -7;
  // a legitimately unassigned channel must stay unassigned
  f.engine_config.channel_state[2].src_input = -1;
  f.engine_config.channel_state[2].src_trig  = -1;
  // and a valid assignment must survive
  f.engine_config.channel_state[3].src_input = N_INPUTS - 1;
  f.engine_config.channel_state[3].src_trig  = N_INPUTS + N_CHANNELS - 1;

  config_validate(&f.engine_config);

  CHECK(f.engine_config.channel_state[0].src_input == -1);
  CHECK(f.engine_config.channel_state[0].src_trig == -1);
  CHECK(f.engine_config.channel_state[1].src_input == -1);
  CHECK(f.engine_config.channel_state[1].src_trig == -1);
  CHECK(f.engine_config.channel_state[2].src_input == -1);
  CHECK(f.engine_config.channel_state[2].src_trig == -1);
  CHECK(f.engine_config.channel_state[3].src_input == N_INPUTS - 1);
  CHECK(f.engine_config.channel_state[3].src_trig == N_INPUTS + N_CHANNELS - 1);
}

TEST_CASE(stray_quantiser_mask_bits_are_dropped)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.quantize_mask = 0xFFFF;

  config_validate(&f.engine_config);

  CHECK(f.engine_config.quantize_mask == 0x0FFF);
}

// Whatever it is handed, the result must be safe to index with.
TEST_CASE(every_field_is_index_safe_after_validation)
{
  Fixture f;
  fixture_init(&f);
  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    f.engine_config.channel_state[c].shape_mode     = (int8_t) (c * 37 - 60);
    f.engine_config.channel_state[c].quantize_mode  = (ChannelQuantizeMode) (c * 13 - 20);
    f.engine_config.channel_state[c].input_amp_mode = (ChannelInputAmpMode) (c * 9 - 15);
    f.engine_config.channel_state[c].src_input      = (int8_t) (c * 31 - 50);
    f.engine_config.channel_state[c].src_trig       = (int8_t) (c * 29 - 45);
  }

  config_validate(&f.engine_config);

  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    const ChannelConfig* ch = &f.engine_config.channel_state[c];
    CHECK(ch->shape_mode >= 0 && ch->shape_mode < SHAPE_MODE_COUNT);
    CHECK(ch->quantize_mode >= 0 && ch->quantize_mode < QUANTIZE_MODE_COUNT);
    CHECK(ch->input_amp_mode >= 0 && ch->input_amp_mode < INPUT_AMP_MODE_COUNT);
    CHECK(ch->src_input == -1 || (ch->src_input >= 0 && ch->src_input < N_INPUTS));
    CHECK(ch->src_trig == -1 || (ch->src_trig >= 0 && ch->src_trig < N_INPUTS + N_CHANNELS));
  }
}

int main(void)
{
  RUN_TEST(a_valid_config_is_passed_through_unchanged);
  RUN_TEST(shape_mode_from_a_newer_build_is_brought_into_range);
  RUN_TEST(out_of_range_modes_are_brought_into_range);
  RUN_TEST(out_of_range_routing_becomes_unassigned_not_clamped);
  RUN_TEST(stray_quantiser_mask_bits_are_dropped);
  RUN_TEST(every_field_is_index_safe_after_validation);
  return TESTKIT_SUMMARY();
}
