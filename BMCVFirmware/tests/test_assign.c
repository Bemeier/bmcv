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
  RUN_TEST(two_step_assign_copies_scene_to_scene_for_every_channel);
  return TESTKIT_SUMMARY();
}
