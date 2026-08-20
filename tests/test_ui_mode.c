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

// Any ctrl button used to exit, which put every page one stray press from
// closing - and made the parameter row unreachable as a row, since the tap that
// selected SHP also shut the page you wanted it on.
TEST_CASE(another_page_button_does_not_leave_a_page)
{
  for (uint8_t mode = 0; mode < SHIFT_STATE_NONE; mode++)
  {
    uint8_t other = (mode == SHIFT_STATE_SYS) ? SHIFT_STATE_MIX : SHIFT_STATE_SYS;

    Fixture f;
    fixture_init(&f);
    latch(&f, mode);
    fixture_press(&f, ctrl_btn(&f, other), MS(40));

    CHECK(f.ui_state.shift_state == mode);
  }
}

// The three unlit caps are the exception, because they are the buttons that are
// not labelled with anything else: they are the way out that does not have to
// be the button you came in on. It includes QNT, whose keyboard overlay covers
// the other five ctrl buttons but not these three.
TEST_CASE(an_unlit_cap_leaves_any_page)
{
  for (uint8_t cap = CH_PARAM_COUNT; cap < SHIFT_STATE_NONE; cap++)
  {
    for (uint8_t mode = 0; mode < SHIFT_STATE_NONE; mode++)
    {
      Fixture f;
      fixture_init(&f);
      latch(&f, mode);
      fixture_press(&f, ctrl_btn(&f, cap), MS(40));

      CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
    }
  }
}

// The tap that leaves a page leaves it *and* selects the parameter on the
// button, in the same tick. The two are not in competition: the six page
// buttons are the six parameter buttons, and a button that is labelled SHP
// should give you SHP whether or not a page happened to be open. This was the
// other way round for a while - the exit swallowed the tap, so getting to SHP
// from its own page meant pressing SHP, watching nothing happen, and pressing
// again.
TEST_CASE(the_tap_that_exits_a_mode_selects_that_buttons_param)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.selected_param = CH_PARAM_FRQ;

  // The SYS page and the SHP parameter are the same button.
  latch(&f, SHIFT_STATE_SYS);
  fixture_press(&f, ctrl_btn(&f, SHIFT_STATE_SYS), MS(40));

  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  CHECK(f.engine_config.selected_param == CH_PARAM_SHP);
}

/* ---- held pages -------------------------------------------------------- */

// The performative half of the gesture: hold STA, tap the scene you want on
// that end of the crossfader, let go, and you are back where you were with the
// assignment made. Nothing to tap shut afterwards, which is the whole point.
TEST_CASE(a_page_used_while_its_button_is_held_leaves_on_release)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_STA), UI_T_HOLD + MS(50));
  CHECK(f.ui_state.shift_state == SHIFT_STATE_STA);

  fixture_press(&f, f.ux_setup->scenes[3].button, MS(40));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_STA));

  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  CHECK(f.engine_config.scene_a == 3); // and the assignment stuck
}

// An encoder turn counts as using the page too - the gesture is "hold the
// button, change the thing, let go", whichever control the thing is on.
TEST_CASE(an_encoder_turned_while_the_page_is_held_leaves_it_on_release)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_SYS), UI_T_HOLD + MS(50));
  fixture_encoder(&f, f.ux_setup->channels[2].encoder, 1);
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_SYS));

  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  CHECK(f.engine_config.channel_state[2].shape_mode == 1);
}

// The other half: a hold that did nothing latches, which is the two-handed way
// in and has to stay reachable without touching anything else first.
TEST_CASE(a_page_held_without_being_used_latches)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_STA), UI_T_LONG + MS(500));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_STA));

  CHECK(f.ui_state.shift_state == SHIFT_STATE_STA);
}

// Only the press that opened the page arms the release. Once latched, the page
// stays open however much is done on it - otherwise every assignment made with
// a free hand would close the page under it.
TEST_CASE(a_latched_page_stays_open_when_it_is_used)
{
  Fixture f;
  fixture_init(&f);
  latch(&f, SHIFT_STATE_STA);

  fixture_press(&f, f.ux_setup->scenes[2].button, MS(40));

  CHECK(f.engine_config.scene_a == 2);
  CHECK(f.ui_state.shift_state == SHIFT_STATE_STA);
}

// Picking the source of a copy is half a gesture, so it does not arm the
// release: letting go of CPY after choosing what to copy has to leave the page
// open, or there is nowhere to put it.
TEST_CASE(picking_a_source_while_the_page_is_held_keeps_it_open)
{
  Fixture f;
  fixture_init(&f);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_CPY), UI_T_HOLD + MS(50));
  fixture_press(&f, f.ux_setup->channels[0].button, MS(40));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_CPY));

  CHECK(f.ui_state.shift_state == SHIFT_STATE_CPY);
  CHECK(ui_sel_pending(&f.ui_state));
}

// ... and the commit does arm it, so a copy performed entirely under one held
// button ends with the page closed and the copy made.
TEST_CASE(committing_while_the_page_is_held_leaves_on_release)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 1234);

  fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_CPY), UI_T_HOLD + MS(50));
  fixture_press(&f, f.ux_setup->channels[0].button, MS(40));
  fixture_press(&f, f.ux_setup->channels[1].button, MS(40));
  fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_CPY));

  CHECK(f.ui_state.shift_state == SHIFT_STATE_NONE);
  CHECK(f.engine_config.channel_state[1].params[0][CH_PARAM_FRQ] == 1234);
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
  RUN_TEST(another_page_button_does_not_leave_a_page);
  RUN_TEST(an_unlit_cap_leaves_any_page);
  RUN_TEST(the_tap_that_exits_a_mode_selects_that_buttons_param);
  RUN_TEST(a_page_used_while_its_button_is_held_leaves_on_release);
  RUN_TEST(an_encoder_turned_while_the_page_is_held_leaves_it_on_release);
  RUN_TEST(a_page_held_without_being_used_latches);
  RUN_TEST(a_latched_page_stays_open_when_it_is_used);
  RUN_TEST(picking_a_source_while_the_page_is_held_keeps_it_open);
  RUN_TEST(committing_while_the_page_is_held_leaves_on_release);
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
