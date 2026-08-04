// Reading a preset written by an older build.
//
// The records here are built the way the old firmware built them - the old
// struct, laid out by the same compiler this test runs under, which is the
// relationship that holds on the module too (a chip is only ever read back by
// the target that wrote it).

#include "config_migrate.h"
#include "stepped_random_table.h" // SR_LENGTH_COUNT
#include "testkit.h"
#include <string.h>

// v3's shape numbering, which is what v4 renumbered.
enum
{
  V3_SHAPE_LFO,
  V3_SHAPE_STEPPED_SMOOTH,
  V3_SHAPE_STEPPED_SEMI,
  V3_SHAPE_STEPPED_HARD,
  V3_SHAPE_PWM,
};

// v2's ChannelConfig: no sr_length_idx, no clamp_mode.
typedef struct __attribute__((packed))
{
  int8_t src_input;
  int8_t src_trig;
  int8_t shape_mode;
  ChannelInputAmpMode input_amp_mode;
  ChannelQuantizeMode quantize_mode;
  int16_t params[N_SCENES][CH_PARAM_COUNT];
} ChannelConfigV2;

typedef struct __attribute__((packed))
{
  uint8_t clock_div;
  uint8_t scene_a;
  uint8_t scene_b;
  uint8_t current_preset;
  uint16_t quantize_mask;
  InputMode input_mode[N_INPUTS];
  ChannelConfigV2 channel_state[N_CHANNELS];
} EngineConfigV2;

TEST_CASE(a_current_record_is_read_unchanged)
{
  EngineConfig in;
  memset(&in, 0, sizeof(in));
  in.scene_a                         = 2;
  in.scene_b                         = 5;
  in.quantize_mask                   = 0x0555;
  in.channel_state[3].params[1][CH_PARAM_AMP] = 12345;
  in.channel_state[3].shape_mode              = SHAPE_PWM;

  EngineConfig out;
  CHECK(config_migrate(CONFIG_STATE_VERSION, sizeof(in), &in, &out) == 1);

  CHECK(out.scene_a == 2);
  CHECK(out.scene_b == 5);
  CHECK(out.quantize_mask == 0x0555);
  CHECK(out.channel_state[3].params[1][CH_PARAM_AMP] == 12345);
  CHECK(out.channel_state[3].shape_mode == SHAPE_PWM);
}

// The whole point of the v4 bump: three stepped modes became one, which
// renumbered PWM under it.
TEST_CASE(v3_shape_modes_are_renumbered_not_clamped)
{
  EngineConfig in;
  memset(&in, 0, sizeof(in));
  in.channel_state[0].shape_mode = V3_SHAPE_LFO;
  in.channel_state[1].shape_mode = V3_SHAPE_STEPPED_SMOOTH;
  in.channel_state[2].shape_mode = V3_SHAPE_STEPPED_SEMI;
  in.channel_state[3].shape_mode = V3_SHAPE_STEPPED_HARD;
  in.channel_state[4].shape_mode = V3_SHAPE_PWM;

  EngineConfig out;
  CHECK(config_migrate(3, sizeof(in), &in, &out) == 1);

  CHECK(out.channel_state[0].shape_mode == SHAPE_LFO);
  CHECK(out.channel_state[1].shape_mode == SHAPE_STEPPED);
  CHECK(out.channel_state[2].shape_mode == SHAPE_STEPPED);
  CHECK(out.channel_state[3].shape_mode == SHAPE_STEPPED);

  // The one that would have been silently wrong: clamping 4 to the new
  // SHAPE_MODE_COUNT-1 also lands on PWM, but clamping 2 and 3 lands there too.
  CHECK(out.channel_state[4].shape_mode == SHAPE_PWM);
}

TEST_CASE(v3_keeps_everything_the_layout_already_carried)
{
  EngineConfig in;
  memset(&in, 0, sizeof(in));
  in.scene_a                                  = 1;
  in.scene_b                                  = 6;
  in.channel_state[2].params[4][CH_PARAM_FRQ] = -128;
  in.channel_state[2].sr_length_idx           = 3;
  in.channel_state[2].clamp_mode              = CLAMP_UNI_5;
  in.channel_state[2].src_input               = 1;

  EngineConfig out;
  CHECK(config_migrate(3, sizeof(in), &in, &out) == 1);

  CHECK(out.scene_a == 1);
  CHECK(out.scene_b == 6);
  CHECK(out.channel_state[2].params[4][CH_PARAM_FRQ] == -128);
  CHECK(out.channel_state[2].sr_length_idx == 3);
  CHECK(out.channel_state[2].clamp_mode == CLAMP_UNI_5);
  CHECK(out.channel_state[2].src_input == 1);
}

TEST_CASE(v2_gains_the_two_appended_fields_at_their_defaults)
{
  EngineConfigV2 in;
  memset(&in, 0, sizeof(in));
  in.scene_a = 3;

  EngineConfig out;
  CHECK(config_migrate(2, sizeof(in), &in, &out) == 1);

  CHECK(out.scene_a == 3);
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    CHECK(out.channel_state[c].sr_length_idx == 0);
    CHECK(out.channel_state[c].clamp_mode == CLAMP_BI_10);
  }
}

// AMP was half the swing it is now, so an unconverted v2 patch would come back
// twice as loud as it was dialled in.
TEST_CASE(v2_amplitudes_are_halved_to_keep_the_level)
{
  EngineConfigV2 in;
  memset(&in, 0, sizeof(in));
  in.channel_state[0].params[0][CH_PARAM_AMP] = 20000;
  in.channel_state[0].params[0][CH_PARAM_OFS] = 4000;
  in.channel_state[1].params[2][CH_PARAM_AMP] = -8000;

  EngineConfig out;
  CHECK(config_migrate(2, sizeof(in), &in, &out) == 1);

  CHECK(out.channel_state[0].params[0][CH_PARAM_AMP] == 10000);
  CHECK(out.channel_state[1].params[2][CH_PARAM_AMP] == -4000);

  // Only AMP. An offset is a voltage and always was.
  CHECK(out.channel_state[0].params[0][CH_PARAM_OFS] == 4000);
}

TEST_CASE(v2_carries_its_routing_across)
{
  EngineConfigV2 in;
  memset(&in, 0, sizeof(in));
  in.quantize_mask                    = 0x0AAA;
  in.input_mode[2]                    = INPUT_SLIDER;
  in.channel_state[5].src_input       = 3;
  in.channel_state[5].src_trig        = 2;
  in.channel_state[5].input_amp_mode  = INPUT_AMP_MULT;
  in.channel_state[5].quantize_mode   = QUANTIZE_CONTINUOUS;
  in.channel_state[5].shape_mode      = V3_SHAPE_STEPPED_HARD;

  EngineConfig out;
  CHECK(config_migrate(2, sizeof(in), &in, &out) == 1);

  CHECK(out.quantize_mask == 0x0AAA);
  CHECK(out.input_mode[2] == INPUT_SLIDER);
  CHECK(out.channel_state[5].src_input == 3);
  CHECK(out.channel_state[5].src_trig == 2);
  CHECK(out.channel_state[5].input_amp_mode == INPUT_AMP_MULT);
  CHECK(out.channel_state[5].quantize_mode == QUANTIZE_CONTINUOUS);
  CHECK(out.channel_state[5].shape_mode == SHAPE_STEPPED);
}

TEST_CASE(an_unknown_version_is_refused)
{
  EngineConfig in;
  memset(&in, 0, sizeof(in));
  EngineConfig out;

  CHECK(config_migrate(0, sizeof(in), &in, &out) == 0);
  CHECK(config_migrate(1, sizeof(in), &in, &out) == 0);
  CHECK(config_migrate(CONFIG_STATE_VERSION + 1, sizeof(in), &in, &out) == 0);
  CHECK(config_migrate(9999, sizeof(in), &in, &out) == 0);
}

// A version this build knows, at a length it does not, is a corrupt record and
// not something to convert on a guess.
TEST_CASE(a_length_that_does_not_match_the_version_is_refused)
{
  EngineConfig in;
  memset(&in, 0, sizeof(in));
  EngineConfig out;

  CHECK(config_migrate(CONFIG_STATE_VERSION, sizeof(in) - 1, &in, &out) == 0);
  CHECK(config_migrate(3, sizeof(in) + 1, &in, &out) == 0);
  CHECK(config_migrate(2, sizeof(EngineConfig), &in, &out) == 0);
}

// Whatever a record holds, what comes out is safe to index with. This is the
// same guarantee config_validate gives a current record; a converted one must
// not be an exception to it.
TEST_CASE(a_converted_record_is_validated)
{
  EngineConfig in;
  memset(&in, 0xFF, sizeof(in)); // every field out of range at once
  in.scene_a = 99;
  in.scene_b = 99;

  EngineConfig out;
  CHECK(config_migrate(3, sizeof(in), &in, &out) == 1);

  CHECK(out.scene_a < N_SCENES);
  CHECK(out.scene_b < N_SCENES);
  CHECK((out.quantize_mask & ~0x0FFFu) == 0);
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    CHECK(out.channel_state[c].shape_mode >= 0 && out.channel_state[c].shape_mode < SHAPE_MODE_COUNT);
    CHECK(out.channel_state[c].clamp_mode >= 0 && out.channel_state[c].clamp_mode < CLAMP_MODE_COUNT);
    CHECK(out.channel_state[c].sr_length_idx >= 0 && out.channel_state[c].sr_length_idx < SR_LENGTH_COUNT);
    CHECK(out.channel_state[c].src_input >= -1 && out.channel_state[c].src_input < N_INPUTS);
  }
}

TEST_CASE(a_null_argument_is_refused)
{
  EngineConfig cfg;
  memset(&cfg, 0, sizeof(cfg));

  CHECK(config_migrate(CONFIG_STATE_VERSION, sizeof(cfg), NULL, &cfg) == 0);
  CHECK(config_migrate(CONFIG_STATE_VERSION, sizeof(cfg), &cfg, NULL) == 0);
}

int main(void)
{
  RUN_TEST(a_current_record_is_read_unchanged);
  RUN_TEST(v3_shape_modes_are_renumbered_not_clamped);
  RUN_TEST(v3_keeps_everything_the_layout_already_carried);
  RUN_TEST(v2_gains_the_two_appended_fields_at_their_defaults);
  RUN_TEST(v2_amplitudes_are_halved_to_keep_the_level);
  RUN_TEST(v2_carries_its_routing_across);
  RUN_TEST(an_unknown_version_is_refused);
  RUN_TEST(a_length_that_does_not_match_the_version_is_refused);
  RUN_TEST(a_converted_record_is_validated);
  RUN_TEST(a_null_argument_is_refused);
  return TESTKIT_SUMMARY();
}
