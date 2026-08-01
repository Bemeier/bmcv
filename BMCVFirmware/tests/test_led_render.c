// These assertions were impossible before the LED framebuffer landed: the UX
// layer wrote straight into the WS2812 driver, so there was nothing to read
// back. Now it renders into EngineState.leds[] and tests can inspect it.
#include "color_presets.h"
#include "ctrl_button.h"
#include "fixture.h"
#include "led_fb.h"
#include "testkit.h"
#include "ui_feedback.h"
#include "ui_render.h"
#include "ui_select.h"
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

  ui_render(&f.ux);

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
  ui_render(&f.ux);
  LedRgb lit = led_of_ctrl_button(&f, SHIFT_STATE_SYS);
  CHECK(lit.r > 0 || lit.g > 0 || lit.b > 0);

  f.ui_state.blink_slow = 0;
  ui_render(&f.ux);
  LedRgb dark = led_of_ctrl_button(&f, SHIFT_STATE_SYS);
  CHECK(dark.r == 0 && dark.g == 0 && dark.b == 0);
}

static LedRgb led_of_channel(Fixture* f, uint8_t ch) { return f->engine_state.leds[f->ux_setup->channels[ch].led]; }
static LedRgb led_of_scene(Fixture* f, uint8_t s) { return f->engine_state.leds[f->ux_setup->scenes[s].led]; }
static int lit(LedRgb c) { return c.r > 0 || c.g > 0 || c.b > 0; }
static int purple(LedRgb c) { return c.b > 0 && c.r > 0 && c.g == 0; }

// The layer stack is the whole point of the renderer: base < context < edit <
// confirmation, and each layer must be able to win over the one below.
TEST_CASE(a_confirmation_flash_overrides_the_candidate_highlight_beneath_it)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CPY;
  f.ui_state.blink_fast  = 1;

  // Channel 3 is a copy candidate, so it is highlighted...
  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  ui_render(&f.ux);
  LedRgb candidate = led_of_channel(&f, 3);
  CHECK(lit(candidate));

  // ...and a confirmation on the same LED must replace it, not blend with it.
  ui_feedback_emit(&f.ui_state, FB_WRITE, TGT_CHANNEL, 3);
  ui_render(&f.ux);
  LedRgb confirmed = led_of_channel(&f, 3);
  CHECK(confirmed.g > candidate.g);
}

// The bug class the layered renderer removes: an arm that wrote nothing left
// the framebuffer holding whatever the previous frame drew.
TEST_CASE(every_channel_and_scene_led_is_written_in_every_mode)
{
  for (uint8_t mode = 0; mode < SHIFT_STATE_COUNT; mode++)
  {
    Fixture f;
    fixture_init(&f);

    // Paint the framebuffer with a colour no renderer would produce.
    for (int16_t i = 0; i < LED_COUNT; i++)
      led_set_rgb(&f.ux, i, 1, 1, 1);

    f.ui_state.shift_state = mode;
    ui_render(&f.ux);

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
      LedRgb l = led_of_channel(&f, c);
      CHECK(!(l.r == 1 && l.g == 1 && l.b == 1));
    }
    for (uint8_t s = 0; s < N_SCENES; s++)
    {
      LedRgb l = led_of_scene(&f, s);
      CHECK(!(l.r == 1 && l.g == 1 && l.b == 1));
    }
  }
}

TEST_CASE(a_muted_channel_reads_purple_in_every_mode_not_just_in_mute)
{
  const uint8_t modes[] = {SHIFT_STATE_NONE, SHIFT_STATE_SYS, SHIFT_STATE_MUT, SHIFT_STATE_CPY};

  for (unsigned m = 0; m < sizeof modes / sizeof modes[0]; m++)
  {
    Fixture f;
    fixture_init(&f);
    f.ui_state.shift_state                  = modes[m];
    f.ui_state.muted[2]                     = 1;
    f.engine_state.channels_output_level[2] = DAC_5V; // would otherwise be green

    // blink_fast low, so any candidate pulse is in its gap and the base layer
    // is what shows through.
    f.ui_state.blink_fast = 0;
    ui_render(&f.ux);
    CHECK(purple(led_of_channel(&f, 2)));
  }
}

// The agreed rule: output level is the base, the value being edited only
// surfaces on touch and then decays back.
TEST_CASE(the_edit_value_shows_on_touch_and_decays_back_to_the_output_level)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                      = SHIFT_STATE_SYS;
  f.engine_state.channels_output_level[1]     = -DAC_5V;            // red as a base
  f.engine_config.channel_state[1].shape_mode = SHAPE_STEPPED_HARD; // cyan as a value

  f.ui_state.channels_edit_hold[1] = UI_EDIT_DISPLAY;
  ui_render(&f.ux);
  LedRgb touched = led_of_channel(&f, 1);
  CHECK(touched.b > 0); // the shape colour, not the level

  f.ui_state.channels_edit_hold[1] = 0;
  ui_render(&f.ux);
  LedRgb settled = led_of_channel(&f, 1);
  CHECK(settled.r > 0 && settled.b == 0); // back to the negative output level
}

int main(void)
{
  RUN_TEST(hsv_maps_onto_expected_rgb);
  RUN_TEST(bipolar_cv_shows_green_when_positive_and_red_when_negative);
  RUN_TEST(out_of_range_led_index_is_ignored_rather_than_corrupting_memory);
  RUN_TEST(selected_param_button_lights_up_and_others_do_not);
  RUN_TEST(active_shift_mode_button_blinks_with_the_slow_blink_signal);
  RUN_TEST(a_confirmation_flash_overrides_the_candidate_highlight_beneath_it);
  RUN_TEST(every_channel_and_scene_led_is_written_in_every_mode);
  RUN_TEST(a_muted_channel_reads_purple_in_every_mode_not_just_in_mute);
  RUN_TEST(the_edit_value_shows_on_touch_and_decays_back_to_the_output_level);
  return TESTKIT_SUMMARY();
}
