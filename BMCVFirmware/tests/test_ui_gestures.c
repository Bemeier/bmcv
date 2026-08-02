// End-to-end coverage of the gesture handling that stage 1 rewrote in terms
// of button events. These go through engine_tick, so they exercise the UX
// rate limit and the event accumulation together with the handlers - the
// combination that was previously untested.
#include "channel.h"
#include "config.h"
#include "ctrl_button.h"
#include "fixture.h"
#include "testkit.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_mode.h"

static int8_t ctrl_btn(Fixture* f, uint8_t id) { return f->ux_setup->ctrl_buttons[id].button; }
static int8_t scene_btn(Fixture* f, uint8_t id) { return f->ux_setup->scenes[id].button; }

TEST_CASE(holding_a_ctrl_button_latches_its_shift_mode_and_it_survives_release)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_NONE;

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_CPY), UI_T_HOLD + MS(50));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_CPY);

  // Releasing after the hold must not drop straight back out.
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_CPY));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_CPY);
}

TEST_CASE(a_tap_that_is_too_short_to_latch_does_not_enter_a_shift_mode)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_NONE;

  fixture_press(&f, ctrl_btn(&f, SHIFT_STATE_CPY), MS(40));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
}

TEST_CASE(tapping_a_ctrl_button_exits_an_active_shift_mode)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_SYS), UI_T_HOLD + MS(50));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_SYS));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_SYS);

  fixture_press(&f, ctrl_btn(&f, SHIFT_STATE_SYS), MS(40));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
}

// QNT overlays the ctrl and scene buttons with a semitone keyboard, so only
// its own button may exit it.
TEST_CASE(quantizer_mode_is_not_exited_by_other_ctrl_buttons)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_QNT), UI_T_HOLD + MS(50));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_QNT));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_QNT);

  fixture_press(&f, ctrl_btn(&f, SHIFT_STATE_SYS), MS(40));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_QNT);

  fixture_press(&f, ctrl_btn(&f, SHIFT_STATE_QNT), MS(40));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
}

TEST_CASE(tapping_a_param_button_in_normal_mode_selects_that_param)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state    = SHIFT_STATE_NONE;
  f.ui_state.selected_param = CH_PARAM_FRQ;

  fixture_press(&f, ctrl_btn(&f, CH_PARAM_AMP), MS(40));
  CHECK(f.ui_state.selected_param == CH_PARAM_AMP);
}

TEST_CASE(holding_a_scene_button_activates_it_momentarily_and_release_restores)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_NONE;

  fixture_hold(&f, scene_btn(&f, 3), MS(50));
  CHECK(f.ui_state.momentary_scene == 3);
  CHECK(f.engine_state.active_scene == 3);

  fixture_release(&f, scene_btn(&f, 3));
  CHECK(f.ui_state.momentary_scene == -1);
}

// The threshold that used to be written `> 10` (microseconds) in the render
// path while the action path used MS(10). Both now come from UI_T_DEBOUNCE.
TEST_CASE(a_scene_button_bounce_does_not_trigger_momentary_activation)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_NONE;

  fixture_press(&f, scene_btn(&f, 2), MS(3));
  CHECK(f.ui_state.momentary_scene == -1);
}

TEST_CASE(turning_an_encoder_edits_the_selected_param_of_the_active_scene)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state      = SHIFT_STATE_NONE;
  f.ui_state.selected_param   = CH_PARAM_OFS;
  f.engine_state.active_scene = 0;

  int16_t before = f.engine_config.channel_state[0].params[0][CH_PARAM_OFS];
  fixture_encoder(&f, f.ux_setup->channels[0].encoder, 2);
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_OFS] != before);
}

// In SYS the encoder steps the shape mode rather than a scene parameter.
TEST_CASE(encoder_in_sys_mode_steps_the_shape_mode)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_SYS), UI_T_HOLD + MS(50));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_SYS));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_SYS);

  f.engine_config.channel_state[1].shape_mode = SHAPE_LFO;
  fixture_encoder(&f, f.ux_setup->channels[1].encoder, 1);
  CHECK(f.engine_config.channel_state[1].shape_mode == SHAPE_STEPPED);
}

// Ctrl-page settings clamp at both ends instead of wrapping. They are short
// lists of unrelated states, so rolling off one end into the other is the
// largest change on the page and never a move anyone means to make - and with
// every list's default at index 0, spinning fully left is a blind reset.
TEST_CASE(ctrl_page_settings_clamp_at_both_ends_instead_of_wrapping)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_SYS), UI_T_HOLD + MS(50));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_SYS));

  int8_t enc = f.ux_setup->channels[1].encoder;

  for (int i = 0; i < SHAPE_MODE_COUNT + 3; i++)
    fixture_encoder(&f, enc, 1);
  CHECK(f.engine_config.channel_state[1].shape_mode == SHAPE_MODE_COUNT - 1);

  for (int i = 0; i < SHAPE_MODE_COUNT + 3; i++)
    fixture_encoder(&f, enc, -1);
  CHECK(f.engine_config.channel_state[1].shape_mode == 0); // the default, at the far left

  // One detent is one step, however big the encoder delta claims to be.
  fixture_encoder(&f, enc, 4);
  CHECK(f.engine_config.channel_state[1].shape_mode == 1);
}

TEST_CASE(the_output_clamp_setting_clamps_rather_than_wrapping_too)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_SAV), UI_T_HOLD + MS(50));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_SAV));

  int8_t enc = f.ux_setup->channels[0].encoder;

  for (int i = 0; i < CLAMP_MODE_COUNT + 3; i++)
    fixture_encoder(&f, enc, 1);
  CHECK(f.engine_config.channel_state[0].clamp_mode == CLAMP_MODE_COUNT - 1);

  for (int i = 0; i < CLAMP_MODE_COUNT + 3; i++)
    fixture_encoder(&f, enc, -1);
  CHECK(f.engine_config.channel_state[0].clamp_mode == CLAMP_BI_10);
}

// A press with no encoder movement clears the selected param; the same press
// while turning the encoder must not, because there the button is acting as a
// fine-adjust modifier.
TEST_CASE(pressing_the_encoder_clears_the_param_only_when_it_was_not_turned)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state      = SHIFT_STATE_NONE;
  f.ui_state.selected_param   = CH_PARAM_OFS;
  f.engine_state.active_scene = 0;

  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 4321);
  fixture_press(&f, f.ux_setup->channels[0].button, MS(40));
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_OFS] == 0);

  // Now the modifier case: turn the encoder during the hold.
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 4321);
  f.hw_state.button_state[f.ux_setup->channels[0].button] = 1;
  fixture_tick(&f, MS(1));
  fixture_encoder(&f, f.ux_setup->channels[0].encoder, 1);
  fixture_hold(&f, f.ux_setup->channels[0].button, MS(600));
  fixture_release(&f, f.ux_setup->channels[0].button);
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_OFS] != 0);
}

// A tap is scoped to the scene on screen; holding widens it to every scene.
// Both report back, and the wider one says so by lasting longer.
TEST_CASE(a_tap_clears_the_param_in_one_scene_and_a_hold_clears_it_in_all)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state      = SHIFT_STATE_NONE;
  f.ui_state.selected_param   = CH_PARAM_OFS;
  f.engine_state.active_scene = 0;

  for (uint8_t s = 0; s < N_SCENES; s++)
    fixture_set_param(&f, 0, s, CH_PARAM_OFS, 4321);

  fixture_press(&f, f.ux_setup->channels[0].button, MS(40));
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_OFS] == 0);
  CHECK(f.engine_config.channel_state[0].params[1][CH_PARAM_OFS] == 4321);

  FeedbackKind k;
  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 0, &k));
  CHECK(k == FB_CLEAR);

  fixture_press(&f, f.ux_setup->channels[0].button, MS(600));
  for (uint8_t s = 0; s < N_SCENES; s++)
    CHECK(f.engine_config.channel_state[0].params[s][CH_PARAM_OFS] == 0);

  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 0, &k));
  CHECK(k == FB_CLEAR_ALL);
}

// Rotation is absolute rather than a toggle, so a row of channels can be muted
// by feel without checking each one's state first.
TEST_CASE(the_encoder_mutes_one_way_and_unmutes_the_other_in_mute_mode)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_MUT), UI_T_HOLD + MS(50));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_MUT));

  fixture_encoder(&f, f.ux_setup->channels[3].encoder, -1);
  CHECK(f.ui_state.muted[3]);
  fixture_encoder(&f, f.ux_setup->channels[3].encoder, -1); // still muted
  CHECK(f.ui_state.muted[3]);

  fixture_encoder(&f, f.ux_setup->channels[3].encoder, 1);
  CHECK(!f.ui_state.muted[3]);
  fixture_encoder(&f, f.ux_setup->channels[3].encoder, 1);
  CHECK(!f.ui_state.muted[3]);
}

int main(void)
{
  RUN_TEST(holding_a_ctrl_button_latches_its_shift_mode_and_it_survives_release);
  RUN_TEST(a_tap_that_is_too_short_to_latch_does_not_enter_a_shift_mode);
  RUN_TEST(tapping_a_ctrl_button_exits_an_active_shift_mode);
  RUN_TEST(quantizer_mode_is_not_exited_by_other_ctrl_buttons);
  RUN_TEST(tapping_a_param_button_in_normal_mode_selects_that_param);
  RUN_TEST(holding_a_scene_button_activates_it_momentarily_and_release_restores);
  RUN_TEST(a_scene_button_bounce_does_not_trigger_momentary_activation);
  RUN_TEST(turning_an_encoder_edits_the_selected_param_of_the_active_scene);
  RUN_TEST(encoder_in_sys_mode_steps_the_shape_mode);
  RUN_TEST(ctrl_page_settings_clamp_at_both_ends_instead_of_wrapping);
  RUN_TEST(the_output_clamp_setting_clamps_rather_than_wrapping_too);
  RUN_TEST(pressing_the_encoder_clears_the_param_only_when_it_was_not_turned);
  RUN_TEST(a_tap_clears_the_param_in_one_scene_and_a_hold_clears_it_in_all);
  RUN_TEST(the_encoder_mutes_one_way_and_unmutes_the_other_in_mute_mode);
  return TESTKIT_SUMMARY();
}
