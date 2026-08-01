// These assertions were impossible before the LED framebuffer landed: the UX
// layer wrote straight into the WS2812 driver, so there was nothing to read
// back. Now it renders into EngineState.leds[] and tests can inspect it.
#include "color_presets.h"
#include "ctrl_button.h"
#include "fixture.h"
#include "led_fb.h"
#include "testkit.h"
#include "ux_state.h"

static LedRgb led_of_ctrl_button(Fixture* f, uint8_t btn_id) { return f->engine_state.leds[f->ux_setup->ctrl_buttons[btn_id].led]; }

TEST_CASE(hsv_maps_onto_expected_rgb)
{
  Fixture f;
  fixture_init(&f);

  led_set_hsv(&f.ux, 0, 0, SAT_OFF, 100); // zero saturation -> neutral white
  CHECK(f.engine_state.leds[0].r == 100);
  CHECK(f.engine_state.leds[0].g == 100);
  CHECK(f.engine_state.leds[0].b == 100);

  led_set_hsv(&f.ux, 1, 0, SAT_OFF, 0); // value 0 -> off
  CHECK(f.engine_state.leds[1].r == 0 && f.engine_state.leds[1].g == 0 && f.engine_state.leds[1].b == 0);
}

TEST_CASE(bipolar_cv_shows_green_when_positive_and_red_when_negative)
{
  Fixture f;
  fixture_init(&f);

  led_set_dac(&f.ux, 0, DAC_5V / 2);
  CHECK(f.engine_state.leds[0].g > 0);
  CHECK(f.engine_state.leds[0].r == 0);

  led_set_dac(&f.ux, 1, -DAC_5V / 2);
  CHECK(f.engine_state.leds[1].r > 0);
  CHECK(f.engine_state.leds[1].g == 0);
}

TEST_CASE(out_of_range_led_index_is_ignored_rather_than_corrupting_memory)
{
  Fixture f;
  fixture_init(&f);

  // ctrl buttons 6..8 have led index -1 in HwSetup; this must be a no-op.
  led_set_hsv(&f.ux, -1, HUE_RED, SAT_MAX, VAL_MAX);
  led_set_hsv(&f.ux, LED_COUNT, HUE_RED, SAT_MAX, VAL_MAX);

  for (int16_t i = 0; i < LED_COUNT; i++)
  {
    CHECK(f.engine_state.leds[i].r == 0 && f.engine_state.leds[i].g == 0 && f.engine_state.leds[i].b == 0);
  }
}

TEST_CASE(selected_param_button_lights_up_and_others_do_not)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state    = SHIFT_STATE_NONE;
  f.ui_state.selected_param = CH_PARAM_AMP;

  for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
  {
    write_ctrl_button_led(&f.ux_setup->ctrl_buttons[b], &f.ux);
  }

  LedRgb selected = led_of_ctrl_button(&f, CH_PARAM_AMP);
  CHECK(selected.r > 0 || selected.g > 0 || selected.b > 0);

  LedRgb other = led_of_ctrl_button(&f, CH_PARAM_FRQ);
  CHECK(other.r == 0 && other.g == 0 && other.b == 0);
}

TEST_CASE(active_shift_mode_button_blinks_with_the_slow_blink_signal)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_SYS;

  f.ui_state.blink_slow = 1;
  write_ctrl_button_led(&f.ux_setup->ctrl_buttons[SHIFT_STATE_SYS], &f.ux);
  LedRgb lit = led_of_ctrl_button(&f, SHIFT_STATE_SYS);
  CHECK(lit.r > 0 || lit.g > 0 || lit.b > 0);

  f.ui_state.blink_slow = 0;
  write_ctrl_button_led(&f.ux_setup->ctrl_buttons[SHIFT_STATE_SYS], &f.ux);
  LedRgb dark = led_of_ctrl_button(&f, SHIFT_STATE_SYS);
  CHECK(dark.r == 0 && dark.g == 0 && dark.b == 0);
}

int main(void)
{
  RUN_TEST(hsv_maps_onto_expected_rgb);
  RUN_TEST(bipolar_cv_shows_green_when_positive_and_red_when_negative);
  RUN_TEST(out_of_range_led_index_is_ignored_rather_than_corrupting_memory);
  RUN_TEST(selected_param_button_lights_up_and_others_do_not);
  RUN_TEST(active_shift_mode_button_blinks_with_the_slow_blink_signal);
  return TESTKIT_SUMMARY();
}
