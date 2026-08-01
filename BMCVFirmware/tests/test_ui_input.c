// Unit tests for the button event layer. Deliberately driven directly rather
// than through the fixture: these assert exact threshold behaviour, and going
// via engine_tick would couple them to the UX rate limit.
#include "testkit.h"
#include "ui_input.h"
#include <string.h>

// One engine tick with the given button levels held.
static void tick(UiInput* in, HwState* hw, uint32_t dt_us)
{
  hw->dt = dt_us;
  hw->time += dt_us;
  ui_input_update(in, hw);
}

static void press(HwState* hw, int8_t btn) { hw->button_state[btn] = 1; }
static void release(HwState* hw, int8_t btn) { hw->button_state[btn] = 0; }

TEST_CASE(press_and_quick_release_yields_down_then_tap)
{
  UiInput in = {0};
  HwState hw = {0};

  press(&hw, 3);
  tick(&in, &hw, MS(1));
  CHECK(btn_ev(&in, 3, BTN_EV_DOWN));
  CHECK(btn_down(&in, 3));
  CHECK(!btn_ev(&in, 3, BTN_EV_TAP));

  ui_input_drain(&in);
  tick(&in, &hw, MS(30));
  release(&hw, 3);
  tick(&in, &hw, MS(1));

  CHECK(btn_ev(&in, 3, BTN_EV_UP));
  CHECK(btn_ev(&in, 3, BTN_EV_TAP));
  CHECK(!btn_down(&in, 3));
  CHECK(!btn_ev(&in, 3, BTN_EV_HOLD));
}

TEST_CASE(a_bounce_shorter_than_the_debounce_window_produces_no_release_event)
{
  UiInput in = {0};
  HwState hw = {0};

  press(&hw, 5);
  tick(&in, &hw, MS(1));
  ui_input_drain(&in);

  tick(&in, &hw, MS(4)); // total held: 4ms, under UI_T_DEBOUNCE
  release(&hw, 5);
  tick(&in, &hw, MS(1));

  CHECK(!btn_ev(&in, 5, BTN_EV_UP));
  CHECK(!btn_ev(&in, 5, BTN_EV_TAP));
}

TEST_CASE(hold_long_and_vlong_each_fire_once_at_their_threshold)
{
  UiInput in = {0};
  HwState hw = {0};

  press(&hw, 7);
  tick(&in, &hw, MS(1));
  ui_input_drain(&in);

  // Just short of UI_T_HOLD.
  tick(&in, &hw, MS(90));
  CHECK(!btn_ev(&in, 7, BTN_EV_HOLD));

  tick(&in, &hw, MS(20)); // now past 100ms
  CHECK(btn_ev(&in, 7, BTN_EV_HOLD));
  CHECK(!btn_ev(&in, 7, BTN_EV_LONG));

  // Does not fire a second time.
  ui_input_drain(&in);
  tick(&in, &hw, MS(10));
  CHECK(!btn_ev(&in, 7, BTN_EV_HOLD));

  tick(&in, &hw, MS(400)); // past 500ms
  CHECK(btn_ev(&in, 7, BTN_EV_LONG));
  CHECK(!btn_ev(&in, 7, BTN_EV_VLONG));

  ui_input_drain(&in);
  tick(&in, &hw, MS(500)); // past 1000ms
  CHECK(btn_ev(&in, 7, BTN_EV_VLONG));
  CHECK(!btn_ev(&in, 7, BTN_EV_LONG));

  // A release after a long hold is an UP but not a TAP.
  release(&hw, 7);
  tick(&in, &hw, MS(1));
  CHECK(btn_ev(&in, 7, BTN_EV_UP));
  CHECK(!btn_ev(&in, 7, BTN_EV_TAP));
}

// The regression this whole layer exists for: the UX pass runs on input
// change or every 8ms, so a threshold crossing lands on a tick where nothing
// is listening. Events must survive until the next drain.
TEST_CASE(events_accumulate_across_ticks_where_the_ux_layer_does_not_run)
{
  UiInput in = {0};
  HwState hw = {0};

  press(&hw, 2);
  tick(&in, &hw, MS(1));
  // no drain: pretend the UX layer has not run since

  for (int i = 0; i < 600; i++)
  {
    tick(&in, &hw, MS(1));
  }
  release(&hw, 2);
  tick(&in, &hw, MS(1));

  // One drain later, every gesture the press produced is still visible.
  CHECK(btn_ev(&in, 2, BTN_EV_DOWN));
  CHECK(btn_ev(&in, 2, BTN_EV_HOLD));
  CHECK(btn_ev(&in, 2, BTN_EV_LONG));
  CHECK(btn_ev(&in, 2, BTN_EV_UP));
  CHECK(!btn_ev(&in, 2, BTN_EV_VLONG)); // only held ~600ms
  CHECK(!btn_ev(&in, 2, BTN_EV_TAP));

  ui_input_drain(&in);
  CHECK(in.ev[2] == BTN_EV_NONE);
}

TEST_CASE(held_duration_survives_release_and_resets_on_the_next_press)
{
  UiInput in = {0};
  HwState hw = {0};

  press(&hw, 1);
  tick(&in, &hw, MS(1));
  tick(&in, &hw, MS(700));
  release(&hw, 1);
  tick(&in, &hw, MS(1));

  // Handlers reacting to the release still need to know how long it was.
  CHECK(btn_held(&in, 1) >= MS(700));
  CHECK(btn_released_after(&in, 1, UI_T_LONG));
  CHECK(!btn_released_after(&in, 1, UI_T_VLONG));

  ui_input_drain(&in);
  press(&hw, 1);
  tick(&in, &hw, MS(1));
  CHECK(btn_held(&in, 1) == 0);
}

TEST_CASE(btn_holding_is_a_live_level_and_goes_false_on_release)
{
  UiInput in = {0};
  HwState hw = {0};

  press(&hw, 4);
  tick(&in, &hw, MS(1));
  tick(&in, &hw, MS(1200));
  CHECK(btn_holding(&in, 4, UI_T_VLONG));

  release(&hw, 4);
  tick(&in, &hw, MS(1));
  // held_us still reads 1200ms, but the button is up - the indicator must
  // not stay lit.
  CHECK(!btn_holding(&in, 4, UI_T_VLONG));
}

TEST_CASE(encoder_deltas_accumulate_until_drained)
{
  UiInput in = {0};
  HwState hw = {0};

  hw.encoder_delta[2] = 3;
  tick(&in, &hw, MS(1));
  hw.encoder_delta[2] = 2;
  tick(&in, &hw, MS(1));
  hw.encoder_delta[2] = -1;
  tick(&in, &hw, MS(1));

  CHECK(enc_delta(&in, 2) == 4);
  CHECK(in.dt == MS(3));

  ui_input_drain(&in);
  CHECK(enc_delta(&in, 2) == 0);
  CHECK(in.dt == 0);
}

TEST_CASE(out_of_range_ids_are_ignored_rather_than_reading_past_the_arrays)
{
  UiInput in = {0};
  CHECK(btn_ev(&in, -1, BTN_EV_DOWN) == 0);
  CHECK(btn_ev(&in, N_BUTTONS, BTN_EV_DOWN) == 0);
  CHECK(btn_down(&in, -1) == 0);
  CHECK(btn_held(&in, N_BUTTONS) == 0);
  CHECK(enc_delta(&in, -1) == 0);
  CHECK(enc_delta(&in, N_ENCODERS) == 0);
}

int main(void)
{
  RUN_TEST(press_and_quick_release_yields_down_then_tap);
  RUN_TEST(a_bounce_shorter_than_the_debounce_window_produces_no_release_event);
  RUN_TEST(hold_long_and_vlong_each_fire_once_at_their_threshold);
  RUN_TEST(events_accumulate_across_ticks_where_the_ux_layer_does_not_run);
  RUN_TEST(held_duration_survives_release_and_resets_on_the_next_press);
  RUN_TEST(btn_holding_is_a_live_level_and_goes_false_on_release);
  RUN_TEST(encoder_deltas_accumulate_until_drained);
  RUN_TEST(out_of_range_ids_are_ignored_rather_than_reading_past_the_arrays);
  return TESTKIT_SUMMARY();
}
