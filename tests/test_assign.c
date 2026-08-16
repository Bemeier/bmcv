#include "assign.h"
#include "fixture.h"
#include "testkit.h"
#include "ui_select.h"

TEST_CASE(two_step_assign_copies_channel_to_channel)
{
  Fixture f;
  fixture_init(&f);

  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 111);
  fixture_set_param(&f, 0, 0, CH_PARAM_SHP, 222);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 333);

  f.ui_state.shift_state = SHIFT_STATE_CPY;
  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0); // press channel 0: pick as source
  ui_sel_press(&f.ux, TGT_CHANNEL, 1, 0); // press channel 1: copy 0 -> 1

  CHECK(f.engine_config.channel_state[1].params[0][CH_PARAM_FRQ] == 111);
  CHECK(f.engine_config.channel_state[1].params[0][CH_PARAM_SHP] == 222);
  CHECK(f.engine_config.channel_state[1].params[0][CH_PARAM_AMP] == 333);
}

TEST_CASE(clear_channel_active_scene_only_leaves_other_scenes_intact)
{
  Fixture f;
  fixture_init(&f);

  fixture_set_param(&f, 2, 0, CH_PARAM_OFS, 500);
  fixture_set_param(&f, 2, 1, CH_PARAM_OFS, 999);
  f.engine_state.active_scene = 0;

  clear_channel(2, 0, &f.ux);

  CHECK(f.engine_config.channel_state[2].params[0][CH_PARAM_OFS] == 0);
  CHECK(f.engine_config.channel_state[2].params[1][CH_PARAM_OFS] == 999);
}

TEST_CASE(clear_channel_all_scenes_resets_everything)
{
  Fixture f;
  fixture_init(&f);

  fixture_set_param(&f, 2, 0, CH_PARAM_OFS, 500);
  fixture_set_param(&f, 2, 1, CH_PARAM_OFS, 999);

  clear_channel(2, 1, &f.ux);

  CHECK(f.engine_config.channel_state[2].params[0][CH_PARAM_OFS] == 0);
  CHECK(f.engine_config.channel_state[2].params[1][CH_PARAM_OFS] == 0);
  CHECK(f.engine_config.channel_state[2].params[0][CH_PARAM_FRQ] == -255);
}

// The settings are as much part of the channel as its numbers are: a channel
// that reads as cleared while an input is still modulating it is a half-state.
TEST_CASE(clearing_a_whole_channel_also_drops_its_routing_and_modes)
{
  Fixture f;
  fixture_init(&f);

  ChannelConfig* ch  = &f.engine_config.channel_state[2];
  ch->src_input      = 1;
  ch->src_trig       = 3;
  ch->input_amp_mode = INPUT_AMP_MULT;
  ch->quantize_mode  = QUANTIZE_CONTINUOUS;
  ch->shape_mode     = SHAPE_STEPPED;
  ch->st_length_idx  = 5;
  ch->clamp_mode     = CLAMP_UNI_5;

  clear_channel(2, 1, &f.ux);

  CHECK(ch->src_input == -1 && ch->src_trig == -1);
  CHECK(ch->input_amp_mode == INPUT_AMP_DISABLED);
  CHECK(ch->quantize_mode == QUANTIZE_DISABLED);
  CHECK(ch->shape_mode == SHAPE_LFO);
  CHECK(ch->st_length_idx == 0);

  // Except the output range, which describes the rig rather than the patch.
  CHECK(ch->clamp_mode == CLAMP_UNI_5);
}

// A tap is scoped to one scene, so it must not take the settings with it.
TEST_CASE(clearing_one_scene_leaves_the_channel_settings_alone)
{
  Fixture f;
  fixture_init(&f);

  ChannelConfig* ch = &f.engine_config.channel_state[2];
  ch->src_input     = 1;
  ch->shape_mode    = SHAPE_STEPPED;

  clear_channel(2, 0, &f.ux);

  CHECK(ch->src_input == 1);
  CHECK(ch->shape_mode == SHAPE_STEPPED);
}

TEST_CASE(two_step_assign_copies_scene_to_scene_for_every_channel)
{
  Fixture f;
  fixture_init(&f);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    fixture_set_param(&f, c, 0, CH_PARAM_OFS, 100 + c);
  }

  f.ui_state.shift_state = SHIFT_STATE_CPY;
  ui_sel_press(&f.ux, TGT_SCENE, 0, 0); // pick scene 0 as source
  ui_sel_press(&f.ux, TGT_SCENE, 1, 0); // copy scene 0 -> scene 1

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    CHECK(f.engine_config.channel_state[c].params[1][CH_PARAM_OFS] == 100 + c);
  }
}

int main(void)
{
  RUN_TEST(two_step_assign_copies_channel_to_channel);
  RUN_TEST(clear_channel_active_scene_only_leaves_other_scenes_intact);
  RUN_TEST(clear_channel_all_scenes_resets_everything);
  RUN_TEST(clearing_a_whole_channel_also_drops_its_routing_and_modes);
  RUN_TEST(clearing_one_scene_leaves_the_channel_settings_alone);
  RUN_TEST(two_step_assign_copies_scene_to_scene_for_every_channel);
  return TESTKIT_SUMMARY();
}
