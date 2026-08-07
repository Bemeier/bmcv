// The selection FSM. The property worth protecting here is that
// ui_sel_is_candidate and ui_sel_press agree: anything the renderer offers as
// a target must actually do something when pressed, and anything it does not
// offer must be inert. Those were separate hand-written conditions per mode
// before, and had already drifted.
#include "assign.h"
#include "config.h"
#include "fixture.h"
#include "testkit.h"
#include "ui_mode.h"
#include "ui_select.h"

static void enter(Fixture* f, uint8_t mode)
{
  f->ui_state.shift_state = mode;
  ui_sel_reset(&f->ui_state);
}

TEST_CASE(copy_holds_a_source_then_commits_on_the_second_press)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_CPY);

  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 4242);

  CHECK(!ui_sel_pending(&f.ui_state));
  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  CHECK(ui_sel_pending(&f.ui_state));
  CHECK(ui_sel_is_src(&f.ui_state, TGT_CHANNEL, 0));

  ui_sel_press(&f.ux, TGT_CHANNEL, 3, 0);
  CHECK(f.engine_config.channel_state[3].params[0][CH_PARAM_AMP] == 4242);
  // Committing clears the selection, so the next press starts a new copy.
  CHECK(!ui_sel_pending(&f.ui_state));
}

TEST_CASE(copy_from_a_channel_offers_both_channels_and_scenes_as_targets)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_CPY);

  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);

  CHECK(ui_sel_is_candidate(&f.ux, TGT_CHANNEL, 5));
  CHECK(ui_sel_is_candidate(&f.ux, TGT_SCENE, 2));
  // Its own button is not a target: CPY does not deselect.
  CHECK(!ui_sel_is_candidate(&f.ux, TGT_CHANNEL, 0));
}

// A whole scene cannot be poured into one channel, so that button must not
// invite the press.
TEST_CASE(copy_from_a_scene_offers_only_scenes)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_CPY);

  ui_sel_press(&f.ux, TGT_SCENE, 0, 0);

  CHECK(ui_sel_is_candidate(&f.ux, TGT_SCENE, 4));
  CHECK(!ui_sel_is_candidate(&f.ux, TGT_CHANNEL, 4));
}

// The invariant: offered == actionable, for every mode and every pairing.
TEST_CASE(anything_offered_as_a_candidate_actually_commits)
{
  const uint8_t modes[]    = {SHIFT_STATE_CPY, SHIFT_STATE_CLR, SHIFT_STATE_MON, SHIFT_STATE_QNT};
  const TargetKind kinds[] = {TGT_CHANNEL, TGT_SCENE, TGT_INPUT};

  for (unsigned m = 0; m < sizeof modes / sizeof modes[0]; m++)
  {
    for (unsigned sk = 0; sk < 3; sk++)
    {
      for (unsigned dk = 0; dk < 3; dk++)
      {
        Fixture f;
        fixture_init(&f);
        enter(&f, modes[m]);

        // Pick a source of kind sk, if this mode accepts one at all.
        if (!ui_sel_is_candidate(&f.ux, kinds[sk], 0))
          continue;
        ui_sel_press(&f.ux, kinds[sk], 0, 0);

        if (!ui_sel_pending(&f.ui_state))
          continue; // immediate mode (CLR): nothing is held

        int offered = ui_sel_is_candidate(&f.ux, kinds[dk], 1);
        ui_sel_press(&f.ux, kinds[dk], 1, 0);
        int consumed = !ui_sel_pending(&f.ui_state);

        // Offering a target must mean pressing it resolves the selection,
        // and a target that was not offered must leave it untouched.
        CHECK(offered == consumed);
      }
    }
  }
}

TEST_CASE(clear_acts_on_the_first_press_without_holding_a_source)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_CLR);

  fixture_set_param(&f, 2, 0, CH_PARAM_OFS, 500);
  fixture_set_param(&f, 2, 1, CH_PARAM_OFS, 999);
  f.engine_state.active_scene = 0;

  CHECK(ui_sel_is_candidate(&f.ux, TGT_CHANNEL, 2));
  ui_sel_press(&f.ux, TGT_CHANNEL, 2, /*is_long=*/0);

  CHECK(f.engine_config.channel_state[2].params[0][CH_PARAM_OFS] == 0);
  CHECK(f.engine_config.channel_state[2].params[1][CH_PARAM_OFS] == 999);
  CHECK(!ui_sel_pending(&f.ui_state)); // never held anything
}

TEST_CASE(a_long_clear_press_on_a_channel_wipes_every_scene)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_CLR);

  fixture_set_param(&f, 2, 0, CH_PARAM_OFS, 500);
  fixture_set_param(&f, 2, 1, CH_PARAM_OFS, 999);

  ui_sel_press(&f.ux, TGT_CHANNEL, 2, /*is_long=*/1);

  CHECK(f.engine_config.channel_state[2].params[0][CH_PARAM_OFS] == 0);
  CHECK(f.engine_config.channel_state[2].params[1][CH_PARAM_OFS] == 0);
}

TEST_CASE(monitor_routes_an_input_and_unroutes_on_pressing_the_source_again)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_MON);

  ui_sel_press(&f.ux, TGT_CHANNEL, 1, 0);
  ui_sel_press(&f.ux, TGT_INPUT, 2, 0);
  CHECK(f.engine_config.channel_state[1].src_input == 2);
  CHECK(!ui_sel_pending(&f.ui_state));

  // Re-pressing the held source is the documented way to undo the routing.
  ui_sel_press(&f.ux, TGT_CHANNEL, 1, 0);
  CHECK(ui_sel_is_candidate(&f.ux, TGT_CHANNEL, 1)); // offered, because deselect is allowed
  ui_sel_press(&f.ux, TGT_CHANNEL, 1, 0);
  CHECK(f.engine_config.channel_state[1].src_input == -1);
  CHECK(!ui_sel_pending(&f.ui_state));
}

TEST_CASE(quantizer_picking_a_channel_puts_it_in_trigger_mode)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_QNT);
  f.engine_config.channel_state[4].quantize_mode = QUANTIZE_DISABLED;

  // Choosing what triggers a channel implies it should be triggered at all;
  // otherwise the assignment would silently do nothing.
  ui_sel_press(&f.ux, TGT_CHANNEL, 4, 0);
  CHECK(f.engine_config.channel_state[4].quantize_mode == QUANTIZE_TRIG_SRC);

  ui_sel_press(&f.ux, TGT_INPUT, 1, 0);
  CHECK(f.engine_config.channel_state[4].src_trig == 1);
}

TEST_CASE(quantizer_pointing_a_channel_at_itself_turns_triggering_off)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_QNT);

  ui_sel_press(&f.ux, TGT_CHANNEL, 4, 0);
  ui_sel_press(&f.ux, TGT_CHANNEL, 4, 0);

  CHECK(f.engine_config.channel_state[4].src_trig == -1);
  CHECK(f.engine_config.channel_state[4].quantize_mode == QUANTIZE_DISABLED);
}

TEST_CASE(quantizer_can_take_another_channel_as_its_trigger_source)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_QNT);

  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  ui_sel_press(&f.ux, TGT_CHANNEL, 5, 0);

  CHECK(f.engine_config.channel_state[0].src_trig == N_INPUTS + 5);
}

TEST_CASE(modes_with_no_action_ignore_presses_and_offer_nothing)
{
  Fixture f;
  fixture_init(&f);
  enter(&f, SHIFT_STATE_NONE);

  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 77);
  CHECK(!ui_sel_is_candidate(&f.ux, TGT_CHANNEL, 0));
  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  CHECK(!ui_sel_pending(&f.ui_state));
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_AMP] == 77);
}

// The keyboard overlay is what makes a tap on another ctrl button a note
// rather than an exit, so it is the same flag both behaviours read.
TEST_CASE(quantizer_is_the_only_mode_that_survives_a_tap_on_another_ctrl_button)
{
  for (uint8_t m = 0; m < SHIFT_STATE_COUNT; m++)
  {
    const UiModeDesc* d = ui_mode(m);
    CHECK((d->keyboard_overlay != 0) == (m == SHIFT_STATE_QNT));
  }
}

int main(void)
{
  RUN_TEST(copy_holds_a_source_then_commits_on_the_second_press);
  RUN_TEST(copy_from_a_channel_offers_both_channels_and_scenes_as_targets);
  RUN_TEST(copy_from_a_scene_offers_only_scenes);
  RUN_TEST(anything_offered_as_a_candidate_actually_commits);
  RUN_TEST(clear_acts_on_the_first_press_without_holding_a_source);
  RUN_TEST(a_long_clear_press_on_a_channel_wipes_every_scene);
  RUN_TEST(monitor_routes_an_input_and_unroutes_on_pressing_the_source_again);
  RUN_TEST(quantizer_picking_a_channel_puts_it_in_trigger_mode);
  RUN_TEST(quantizer_pointing_a_channel_at_itself_turns_triggering_off);
  RUN_TEST(quantizer_can_take_another_channel_as_its_trigger_source);
  RUN_TEST(modes_with_no_action_ignore_presses_and_offer_nothing);
  RUN_TEST(quantizer_is_the_only_mode_that_survives_a_tap_on_another_ctrl_button);
  return TESTKIT_SUMMARY();
}
