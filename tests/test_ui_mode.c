// Shift-mode entry/exit and the behaviours that are now table-driven rather
// than switch arms: mute, and the rule that an exit tap is consumed by the
// exit instead of also changing the selected parameter.
#include "channel.h"
#include "config.h"
#include "fixture.h"
#include "testkit.h"
#include "ui_mode.h"
#include "ui_select.h"

static int8_t ctrl_btn(Fixture* f, uint8_t id) { return f->ux_setup->ctrl_buttons[id].button; }

static void latch(Fixture* f, uint8_t mode)
{
  fixture_hold(f, ctrl_btn(f, mode), UI_T_HOLD + MS(50));
  fixture_release(f, ctrl_btn(f, mode));
}

/* ---- exit semantics ---------------------------------------------------- */

TEST_CASE(a_modes_own_button_always_exits_it)
{
  for (uint8_t mode = 0; mode < SHIFT_STATE_NONE; mode++)
  {
    Fixture f;
    fixture_init(&f);
    latch(&f, mode);
    CHECK(f.ui_state.shift_state == mode);

    fixture_press(&f, ctrl_btn(&f, mode), MS(40));
    CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  }
}

TEST_CASE(another_ctrl_button_exits_every_mode_except_the_quantizer)
{
  for (uint8_t mode = 0; mode < SHIFT_STATE_NONE; mode++)
  {
    uint8_t other = (mode == SHIFT_STATE_SYS) ? SHIFT_STATE_MON : SHIFT_STATE_SYS;

    Fixture f;
    fixture_init(&f);
    latch(&f, mode);
    fixture_press(&f, ctrl_btn(&f, other), MS(40));

    if (mode == SHIFT_STATE_QNT)
      CHECK(f.ui_state.shift_state == SHIFT_STATE_QNT); // keyboard overlay owns the tap
    else
      CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  }
}

// The tap that leaves a mode is spent on leaving it. Previously it also set
// selected_param in the same tick, so exiting via an arbitrary ctrl button
// silently changed what the encoders edit.
TEST_CASE(the_tap_that_exits_a_mode_does_not_also_change_the_selected_param)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.selected_param = CH_PARAM_FRQ;

  latch(&f, SHIFT_STATE_CPY);
  fixture_press(&f, ctrl_btn(&f, CH_PARAM_SHP), MS(40));

  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  CHECK(f.engine_config.selected_param == CH_PARAM_FRQ);

  // A second tap, now that no mode is active, does select it.
  fixture_press(&f, ctrl_btn(&f, CH_PARAM_SHP), MS(40));
  CHECK(f.engine_config.selected_param == CH_PARAM_SHP);
}

TEST_CASE(leaving_a_mode_drops_any_half_finished_selection)
{
  Fixture f;
  fixture_init(&f);

  latch(&f, SHIFT_STATE_CPY);
  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  CHECK(ui_sel_pending(&f.ui_state));

  fixture_press(&f, ctrl_btn(&f, SHIFT_STATE_CPY), MS(40));
  CHECK(!ui_sel_pending(&f.ui_state));
}

/* ---- mute -------------------------------------------------------------- */

TEST_CASE(mute_toggles_on_release_rather_than_on_press)
{
  Fixture f;
  fixture_init(&f);
  latch(&f, SHIFT_STATE_MUT);

  int8_t btn = f.ux_setup->channels[2].button;

  // Held down, but not yet released: still unmuted.
  fixture_hold(&f, btn, MS(200));
  CHECK(!f.ui_state.muted[2]);

  fixture_release(&f, btn);
  CHECK(f.ui_state.muted[2]);

  // And it is a toggle.
  fixture_press(&f, btn, MS(50));
  CHECK(!f.ui_state.muted[2]);
}

TEST_CASE(mute_only_affects_the_channel_whose_button_was_pressed)
{
  Fixture f;
  fixture_init(&f);
  latch(&f, SHIFT_STATE_MUT);

  fixture_press(&f, f.ux_setup->channels[5].button, MS(50));

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    CHECK(f.ui_state.muted[c] == (c == 5));
  }
}

TEST_CASE(mute_survives_leaving_the_mute_mode)
{
  Fixture f;
  fixture_init(&f);
  latch(&f, SHIFT_STATE_MUT);
  fixture_press(&f, f.ux_setup->channels[1].button, MS(50));
  CHECK(f.ui_state.muted[1]);

  fixture_press(&f, ctrl_btn(&f, SHIFT_STATE_MUT), MS(40));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  CHECK(f.ui_state.muted[1]);
}

// Ramped, not stepped: a hard jump to zero clicks.
TEST_CASE(muting_ramps_the_output_down_instead_of_jumping)
{
  Fixture f;
  fixture_init(&f);
  f.engine_state.channels_output_level[0] = DAC_5V;

  // Unmuted, the gain sits fully open.
  f.hw_state.dt = MS(1);
  channel_apply_mute(0, &f.engine_state, f.ui_state.muted[0], f.hw_state.dt);
  CHECK_NEAR(f.engine_state.channels_mute_gain[0], 1.0f, 0.001f);

  f.ui_state.muted[0] = 1;

  // One DAC tick in, it has moved but is nowhere near silent.
  f.hw_state.dt = MS(1);
  channel_apply_mute(0, &f.engine_state, f.ui_state.muted[0], f.hw_state.dt);
  CHECK(f.engine_state.channels_mute_gain[0] < 1.0f);
  CHECK(f.engine_state.channels_mute_gain[0] > 0.5f);

  // Well past the ramp length it is fully closed and stays there.
  for (int i = 0; i < 10; i++)
  {
    f.hw_state.dt = MS(1);
    channel_apply_mute(0, &f.engine_state, f.ui_state.muted[0], f.hw_state.dt);
  }
  CHECK_NEAR(f.engine_state.channels_mute_gain[0], 0.0f, 0.001f);
}

// Mute is the output stage only, so a muted channel still drives anything
// routed from it.
TEST_CASE(a_muted_channel_still_works_as_a_trigger_source)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.muted[0] = 1;

  f.engine_state.channels_output_level[0] = 0;
  channel_detect_trigger(0, &f.engine_state);

  f.engine_state.channels_output_level[0] = DAC_5V; // rising edge
  channel_detect_trigger(0, &f.engine_state);

  CHECK(channel_take_trig(0, &f.engine_state) == 1);
}

/* ---- the table itself --------------------------------------------------- */

TEST_CASE(every_shift_state_has_a_descriptor)
{
  for (uint8_t m = 0; m < SHIFT_STATE_COUNT; m++)
  {
    const UiModeDesc* d = ui_mode(m);
    CHECK(d != NULL && d->name != NULL);
  }
  // Out of range falls back rather than reading past the table.
  CHECK(ui_mode(SHIFT_STATE_COUNT + 5) == ui_mode(SHIFT_STATE_NONE));
}

// A mode that hands presses to the selection model must declare what kind
// those presses address, or the press would resolve to TGT_NONE.
TEST_CASE(modes_that_select_declare_the_kind_their_buttons_address)
{
  for (uint8_t m = 0; m < SHIFT_STATE_COUNT; m++)
  {
    const UiModeDesc* d = ui_mode(m);
    if (d->channel_btn_action == CHB_SELECT)
      CHECK(d->channel_btn_kind != TGT_NONE);
    if (d->scene_btn_action == SCN_SELECT)
      CHECK(d->scene_btn_kind != TGT_NONE);
    if (d->action != ACT_NONE)
      CHECK(d->src_kinds != 0);
  }
}

int main(void)
{
  RUN_TEST(a_modes_own_button_always_exits_it);
  RUN_TEST(another_ctrl_button_exits_every_mode_except_the_quantizer);
  RUN_TEST(the_tap_that_exits_a_mode_does_not_also_change_the_selected_param);
  RUN_TEST(leaving_a_mode_drops_any_half_finished_selection);
  RUN_TEST(mute_toggles_on_release_rather_than_on_press);
  RUN_TEST(mute_only_affects_the_channel_whose_button_was_pressed);
  RUN_TEST(mute_survives_leaving_the_mute_mode);
  RUN_TEST(muting_ramps_the_output_down_instead_of_jumping);
  RUN_TEST(a_muted_channel_still_works_as_a_trigger_source);
  RUN_TEST(every_shift_state_has_a_descriptor);
  RUN_TEST(modes_that_select_declare_the_kind_their_buttons_address);
  return TESTKIT_SUMMARY();
}
