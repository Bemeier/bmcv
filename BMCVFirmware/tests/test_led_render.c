// These assertions were impossible before the LED framebuffer landed: the UX
// layer wrote straight into the WS2812 driver, so there was nothing to read
// back. Now it renders into EngineState.leds[] and tests can inspect it.
#include "color_presets.h"
#include "ctrl_button.h"
#include "fixture.h"
#include "led_fb.h"
#include "stepped_random_table.h" // SR_LENGTH_COUNT
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
// Which primary dominates, for colours bright enough that the others are not
// quite zero.
static int bluest(LedRgb c) { return c.b > c.r && c.b > c.g; }
static int greenest(LedRgb c) { return c.g > c.r && c.g > c.b; }

// The layer stack is the whole point of the renderer: base < context < edit <
// confirmation, and each layer must be able to win over the one below.
TEST_CASE(a_confirmation_flash_overrides_the_candidate_highlight_beneath_it)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CPY;
  f.ui_state.blink_mark  = 0;

  // Channel 3 is somewhere the held source can go, so it is white...
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

TEST_CASE(a_muted_channel_reads_purple_wherever_mute_is_what_the_led_shows)
{
  // Not every mode any more: a shift mode's channel LEDs show that mode's own
  // setting, so mute surfaces where it is the subject (MUT) and where nothing
  // else has claimed the LED (no mode).
  const uint8_t modes[] = {SHIFT_STATE_NONE, SHIFT_STATE_MUT};

  for (unsigned m = 0; m < sizeof modes / sizeof modes[0]; m++)
  {
    Fixture f;
    fixture_init(&f);
    f.ui_state.shift_state                  = modes[m];
    f.ui_state.muted[2]                     = 1;
    f.engine_state.channels_output_level[2] = DAC_5V; // would otherwise be green

    ui_render(&f.ux);
    CHECK(purple(led_of_channel(&f, 2)));
  }
}

// The rule the shift pages now follow: one fact per LED. A mode's channel LED
// is that mode's setting and nothing else - the output level is no longer
// showing underneath it.
TEST_CASE(a_shift_mode_shows_its_own_setting_and_never_the_output_level)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                      = SHIFT_STATE_SYS;
  f.engine_state.channels_output_level[1]     = -DAC_5V;            // red, were it shown
  f.engine_config.channel_state[1].shape_mode = SHAPE_STEPPED_HARD; // yellow as a value

  ui_render(&f.ux);
  LedRgb l = led_of_channel(&f, 1);
  CHECK(l.g > 0 && l.b == 0); // the shape colour...

  // ...and it does not decay back to anything: there is nothing transient here.
  f.ui_state.channels_edit_hold[1] = 0;
  ui_render(&f.ux);
  CHECK(led_of_channel(&f, 1).g > 0);
}

// A mode with no per-channel setting leaves the ring dark rather than falling
// back to the output level.
TEST_CASE(a_mode_with_no_channel_setting_leaves_the_channel_leds_dark)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                  = SHIFT_STATE_STB;
  f.engine_state.channels_output_level[4] = DAC_5V;

  ui_render(&f.ux);
  CHECK(!lit(led_of_channel(&f, 4)));
}

// The mute page is the one place both states are lit, since "passing" is as
// much a state there as "gated".
TEST_CASE(the_mute_page_shows_green_for_passing_and_purple_for_gated)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_MUT;
  f.ui_state.muted[2]    = 1;

  ui_render(&f.ux);
  CHECK(purple(led_of_channel(&f, 2)));

  LedRgb passing = led_of_channel(&f, 3);
  CHECK(passing.g > 0 && passing.b == 0);
}

// Pattern length is a stepped-mode setting, so on the channels where turning
// the encoder would do nothing the ring says so by staying dark.
TEST_CASE(pattern_length_only_lights_the_channels_it_applies_to)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                      = SHIFT_STATE_STA;
  f.engine_config.channel_state[0].shape_mode = SHAPE_STEPPED_SEMI;
  f.engine_config.channel_state[1].shape_mode = SHAPE_LFO;

  ui_render(&f.ux);
  CHECK(lit(led_of_channel(&f, 0)));
  CHECK(!lit(led_of_channel(&f, 1)));

  // And the hue moves with the setting, so two lengths do not look alike.
  LedRgb shortest                                = led_of_channel(&f, 0);
  f.engine_config.channel_state[0].sr_length_idx = SR_LENGTH_COUNT / 2;
  ui_render(&f.ux);
  LedRgb middle = led_of_channel(&f, 0);
  CHECK(shortest.r != middle.r || shortest.g != middle.g || shortest.b != middle.b);
}

// The clamp is polarity and range on one LED, so it uses hue for the first and
// brightness for the second.
TEST_CASE(the_output_clamp_reads_as_polarity_by_hue_and_range_by_brightness)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_SAV;

  f.engine_config.channel_state[0].clamp_mode = CLAMP_BI_10;
  f.engine_config.channel_state[1].clamp_mode = CLAMP_BI_5;
  f.engine_config.channel_state[2].clamp_mode = CLAMP_UNI_10;
  f.engine_config.channel_state[3].clamp_mode = CLAMP_UNI_5;
  ui_render(&f.ux);

  LedRgb bi_10 = led_of_channel(&f, 0), bi_5 = led_of_channel(&f, 1);
  CHECK(bluest(bi_10) && bluest(bi_5)); // bipolar reads purple
  CHECK(bi_10.b > bi_5.b);              // ...bright for the full range, dim for half

  LedRgb uni_10 = led_of_channel(&f, 2), uni_5 = led_of_channel(&f, 3);
  CHECK(greenest(uni_10) && greenest(uni_5)); // unipolar reads green
  CHECK(uni_10.g > uni_5.g);
}

// White is the assignment vocabulary, and it is a mark rather than a blink: in
// the gaps the element still shows what it is set to.
TEST_CASE(a_pickable_element_keeps_its_state_colour_and_marks_itself_white)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CPY; // blue tint, everything pickable

  f.ui_state.blink_mark = 0;
  ui_render(&f.ux);
  LedRgb state = led_of_channel(&f, 3);
  CHECK(state.b > state.r); // the mode's own colour

  f.ui_state.blink_mark = 1;
  ui_render(&f.ux);
  LedRgb mark = led_of_channel(&f, 3);
  CHECK(mark.r > 0 && mark.r == mark.g && mark.g == mark.b); // white
}

// The inverse, once a source is held: valid destinations are steady white and
// everything else goes dark, so nothing that cannot be pressed looks like it
// can be.
TEST_CASE(while_a_source_is_held_only_valid_destinations_light)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_MON; // a channel takes an input, nothing else
  f.ui_state.blink_mark  = 0;

  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  ui_render(&f.ux);

  LedRgb target = led_of_scene(&f, 2); // scene buttons are the inputs here
  CHECK(target.r > 0 && target.r == target.g && target.g == target.b);

  CHECK(!lit(led_of_channel(&f, 1))); // not a destination: dark
  CHECK(purple(led_of_channel(&f, 0)));

  // Scene buttons past the four inputs address nothing, so they stay dark too.
  CHECK(!lit(led_of_scene(&f, N_INPUTS)));

  // And the mark inverts: the destinations drop out briefly rather than lighting.
  f.ui_state.blink_mark = 1;
  ui_render(&f.ux);
  CHECK(!lit(led_of_scene(&f, 2)));
}

// With no mode active the ring is a level meter, and the parameter being edited
// only borrows it on touch.
TEST_CASE(the_selected_parameter_shows_on_touch_and_decays_back_to_the_output_level)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                                                             = SHIFT_STATE_NONE;
  f.ui_state.selected_param                                                          = CH_PARAM_AMP;
  f.engine_state.channels_output_level[1]                                            = -DAC_5V; // red as a base
  f.engine_config.channel_state[1].params[f.engine_state.active_scene][CH_PARAM_AMP] = ADC_5V / 2;

  f.ui_state.channels_edit_hold[1] = UI_EDIT_DISPLAY;
  ui_render(&f.ux);
  LedRgb touched = led_of_channel(&f, 1);
  CHECK(touched.g > 0 && touched.r == 0); // the positive parameter, not the level

  f.ui_state.channels_edit_hold[1] = 0;
  ui_render(&f.ux);
  LedRgb settled = led_of_channel(&f, 1);
  CHECK(settled.r > 0 && settled.g == 0); // back to the negative output level
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
  RUN_TEST(a_muted_channel_reads_purple_wherever_mute_is_what_the_led_shows);
  RUN_TEST(a_shift_mode_shows_its_own_setting_and_never_the_output_level);
  RUN_TEST(a_mode_with_no_channel_setting_leaves_the_channel_leds_dark);
  RUN_TEST(the_mute_page_shows_green_for_passing_and_purple_for_gated);
  RUN_TEST(pattern_length_only_lights_the_channels_it_applies_to);
  RUN_TEST(the_output_clamp_reads_as_polarity_by_hue_and_range_by_brightness);
  RUN_TEST(a_pickable_element_keeps_its_state_colour_and_marks_itself_white);
  RUN_TEST(while_a_source_is_held_only_valid_destinations_light);
  RUN_TEST(the_selected_parameter_shows_on_touch_and_decays_back_to_the_output_level);
  return TESTKIT_SUMMARY();
}
