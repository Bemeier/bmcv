// These assertions were impossible before the LED framebuffer landed: the UX
// layer wrote straight into the WS2812 driver, so there was nothing to read
// back. Now it renders into EngineState.leds[] and tests can inspect it.
#include "color_presets.h"
#include "ctrl_button.h"
#include "fixture.h"
#include "led_curve.h"
#include "led_fb.h"
#include "panel_layout.h"
#include "stepped.h"
#include "stepped_table.h" // ST_LENGTH_COUNT
#include "testkit.h"
#include "ui_channel.h"
#include "ui_feedback.h"
#include "ui_render.h"
#include "ui_select.h"
#include "ui_sparkle.h"
#include "ux_state.h"

static LedRgb led_of_ctrl_button(Fixture* f, uint8_t btn_id) { return f->engine_state.leds[f->ux_setup->ctrl_buttons[btn_id].led]; }

static LedRgb led_of_channel(Fixture* f, uint8_t ch) { return f->engine_state.leds[f->ux_setup->channels[ch].led]; }
static LedRgb led_of_scene(Fixture* f, uint8_t s) { return f->engine_state.leds[f->ux_setup->scenes[s].led]; }
static int lit(LedRgb c) { return c.r > 0 || c.g > 0 || c.b > 0; }

// The VAL_* a colour was drawn at, recovered from the light it puts out.
//
// Not the peak duty. led_set_hsv scales every hue until its luminance is the
// value it was asked for times LED_PALETTE_REF, so the duty a value lands on
// now depends on the hue - which is the whole point of balancing it - and the
// value is what survives that.
static uint16_t value_of(LedRgb c)
{
  float lum = (float) c.r * LED_W_RED + (float) c.g * LED_W_GREEN + (float) c.b * LED_W_BLUE;
  return (uint16_t) (lum / LED_PALETTE_REF / (float) LED_UNIT + 0.5f);
}

// Which primary dominates. Dominance rather than "the others are zero": a dim,
// saturated colour asks for a fraction of a duty step from its minor primaries
// - a VAL_LOW purple wants 0.78 of one from green - and the framebuffer now
// keeps that where the old whole-step maths rounded it away. Asserting a zero
// there was asserting the rounding, not the colour.
static int bluest(LedRgb c) { return c.b > c.r && c.b > c.g; }
static int greenest(LedRgb c) { return c.g > c.r && c.g > c.b; }
static int purple(LedRgb c) { return bluest(c) && c.r > c.g; }

// Well clear of it, for telling a colour's own hue from its rounding tail.
static int dominates(uint16_t major, uint16_t minor) { return major > minor * 4; }

// Which primary sits above which - a colour's identity, with how bright and how
// washed it is divided out. What has to survive the marker being laid over an
// element, since the whole point of washing rather than whitening is that the
// element goes on saying what it is.
static int same_hue_order(LedRgb a, LedRgb b)
{
  return (a.r > a.g) == (b.r > b.g) && (a.r > a.b) == (b.r > b.b) && (a.g > a.b) == (b.g > b.b);
}

// Saturation, as the gap between the brightest and dimmest primary against the
// brightest. Compared as a cross-multiply so it stays in integers.
static int less_saturated_than(LedRgb a, LedRgb b)
{
  uint16_t a_hi = a.r > a.g ? a.r : a.g, a_lo = a.r < a.g ? a.r : a.g;
  uint16_t b_hi = b.r > b.g ? b.r : b.g, b_lo = b.r < b.g ? b.r : b.g;
  if (a.b > a_hi)
    a_hi = a.b;
  if (a.b < a_lo)
    a_lo = a.b;
  if (b.b > b_hi)
    b_hi = b.b;
  if (b.b < b_lo)
    b_lo = b.b;
  return (uint32_t) a_lo * b_hi > (uint32_t) b_lo * a_hi;
}

TEST_CASE(hsv_maps_onto_expected_rgb)
{
  Fixture f;
  fixture_init(&f);

  led_set_hsv(&f.ux, 0, 0, SAT_OFF, 100); // zero saturation -> neutral white
  CHECK(f.engine_state.leds[0].r == f.engine_state.leds[0].g);
  CHECK(f.engine_state.leds[0].g == f.engine_state.leds[0].b);

  // Neutral, and at the brightness it asked for rather than at that duty: white
  // lights all three dies, so it reaches a given brightness on a third of the
  // duty a single-die hue needs.
  CHECK(value_of(f.engine_state.leds[0]) == 100);
  CHECK(f.engine_state.leds[0].r < 100 * LED_UNIT);

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

// With no mode running the whole parameter row is legible - which colour goes
// with which parameter - and the selected one stands well clear of the rest.
TEST_CASE(the_selected_param_button_stands_out_from_the_dim_others)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state         = SHIFT_STATE_NONE;
  f.engine_config.selected_param = CH_PARAM_AMP;

  ui_render(&f.ux);

  LedRgb selected = led_of_ctrl_button(&f, CH_PARAM_AMP);
  LedRgb other    = led_of_ctrl_button(&f, CH_PARAM_FRQ);
  CHECK(lit(selected) && lit(other));
  CHECK(selected.r + selected.g + selected.b > 4 * (other.r + other.g + other.b));
}

// Inside a mode only that mode's own button is lit, so the row cannot be
// mistaken for a parameter selection.
TEST_CASE(parameter_buttons_go_dark_while_a_shift_mode_is_running)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state         = SHIFT_STATE_MIX;
  f.engine_config.selected_param = CH_PARAM_AMP;

  ui_render(&f.ux);
  CHECK(!lit(led_of_ctrl_button(&f, CH_PARAM_AMP)));
  CHECK(!lit(led_of_ctrl_button(&f, CH_PARAM_FRQ)));
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

// The layer stack is the whole point of the renderer: base < context < edit <
// confirmation, and each layer must be able to win over the one below.
TEST_CASE(a_confirmation_flash_overrides_the_candidate_highlight_beneath_it)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CPY;

  // Channel 3 is somewhere the held source can go, so it is white...
  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  ui_render(&f.ux);
  LedRgb candidate = led_of_channel(&f, 3);
  CHECK(lit(candidate));

  // ...and a confirmation on the same LED must replace it, not blend with it.
  ui_feedback_emit(&f.ui_state, FB_WRITE, TGT_CHANNEL, 3);
  ui_render(&f.ux);
  // Purple, not a blend of purple and the white underneath it. Checked as "the
  // colour is not white" rather than "brighter than the candidate was": the
  // candidate is a sparkle now, so how bright it was at that instant is a
  // property of the field and not something to hang an assertion on.
  LedRgb confirmed = led_of_channel(&f, 3);
  CHECK(bluest(confirmed) && dominates(confirmed.b, confirmed.g));
  CHECK(lit(candidate));
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
      CHECK(!(l.r == LED_UNIT && l.g == LED_UNIT && l.b == LED_UNIT));
    }
    for (uint8_t s = 0; s < N_SCENES; s++)
    {
      LedRgb l = led_of_scene(&f, s);
      CHECK(!(l.r == LED_UNIT && l.g == LED_UNIT && l.b == LED_UNIT));
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
  f.engine_state.channels_output_level[1]     = -DAC_5V;   // red, were it shown
  f.engine_config.channel_state[1].shape_mode = SHAPE_PWM; // yellow as a value

  ui_render(&f.ux);
  LedRgb l = led_of_channel(&f, 1);
  CHECK(l.g > 0 && dominates(l.g, l.b)); // the shape colour, not a red level...

  // ...and it does not decay back to anything: there is nothing transient here.
  f.ui_state.param_display_hold = 0;
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
  CHECK(passing.g > 0 && greenest(passing));
}

// Pattern length is a stepped-mode setting, so on the channels where turning
// the encoder would do nothing the ring says so by staying dark.
TEST_CASE(pattern_length_only_lights_the_channels_it_applies_to)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                      = SHIFT_STATE_STA;
  f.engine_config.channel_state[0].shape_mode = SHAPE_STEPPED;
  f.engine_config.channel_state[1].shape_mode = SHAPE_LFO;

  ui_render(&f.ux);
  CHECK(lit(led_of_channel(&f, 0)));
  CHECK(!lit(led_of_channel(&f, 1)));

  // The hue is the length's prime limit, not its position on the knob: a 3-step
  // pattern and a 4-step one subdivide differently and are different colours...
  f.engine_config.channel_state[0].st_length_idx = 0; // 3 steps, a triplet
  ui_render(&f.ux);
  LedRgb three                                   = led_of_channel(&f, 0);
  f.engine_config.channel_state[0].st_length_idx = 1; // 4 steps, straight
  ui_render(&f.ux);
  LedRgb four = led_of_channel(&f, 0);
  CHECK(!same_hue_order(three, four));

  // ...while 3 and 12 subdivide the same way and wear the same colour, which is
  // the whole point of colouring the limit. What tells those two apart is the
  // rate, so the ring has to be moving at four times the speed on the longer
  // one - checked as two phases that land differently once multiplied.
  f.engine_state.channels_effective[0].freq_hz = 1.0f;
  f.engine_state.channels_effective[0].phase   = 0.1f;

  f.engine_config.channel_state[0].st_length_idx = 0; // 3 steps -> step phase 0.3
  ui_render(&f.ux);
  LedRgb slow                                    = led_of_channel(&f, 0);
  f.engine_config.channel_state[0].st_length_idx = 5; // 12 steps -> step phase 0.2
  ui_render(&f.ux);
  LedRgb fast = led_of_channel(&f, 0);

  CHECK(same_hue_order(slow, fast));       // both triplets
  CHECK(value_of(slow) != value_of(fast)); // at different points in their step
}

// One scale for both pages: the hue a division wears depends on its prime limit
// and on nothing else, so a triplet is the same yellow whether it is the rate a
// channel runs at or the way its pattern subdivides.
TEST_CASE(the_division_hue_codes_the_prime_limit_and_is_shared_by_both_pages)
{
  // Octaves are free, so everything that halves down to 1 is one colour.
  CHECK(ui_division_hue(1) == HUE_FREQ_STRAIGHT);
  CHECK(ui_division_hue(4) == HUE_FREQ_STRAIGHT);
  CHECK(ui_division_hue(64) == HUE_FREQ_STRAIGHT);

  CHECK(ui_division_hue(3) == HUE_FREQ_TRIPLET);
  CHECK(ui_division_hue(12) == HUE_FREQ_TRIPLET); // 3 with two octaves on it
  CHECK(ui_division_hue(48) == HUE_FREQ_TRIPLET);

  CHECK(ui_division_hue(5) == HUE_FREQ_QUINTUPLET);

  // Three classes, distinct, and none of them red - red is errors.
  CHECK(HUE_FREQ_STRAIGHT != HUE_FREQ_TRIPLET);
  CHECK(HUE_FREQ_TRIPLET != HUE_FREQ_QUINTUPLET);
  CHECK(HUE_FREQ_QUINTUPLET != HUE_RED);

  // Spread rather than packed. Two gaps to keep: wide enough that the pulse's
  // own hue swing cannot walk one class into the next, and wide enough to tell
  // apart on a panel - which the old 13 between yellow and orange was not.
  CHECK(HUE_FREQ_TRIPLET - HUE_FREQ_QUINTUPLET > 4 * RING_PULSE_HUE_SWING);
  CHECK(HUE_FREQ_QUINTUPLET - HUE_RED > 4 * RING_PULSE_HUE_SWING);
}

// The scale has three classes and the lengths are chosen to stay inside them.
// A length with a 7 in it would fall through to straight and read as a plain
// division, which is the one way this can go quietly wrong.
TEST_CASE(every_pattern_length_lands_on_a_division_class)
{
  for (int i = 0; i < ST_LENGTH_COUNT; i++)
  {
    int len   = st_length_for_index(i);
    uint8_t h = ui_division_hue((uint32_t) len);
    CHECK(h == HUE_FREQ_STRAIGHT || h == HUE_FREQ_TRIPLET || h == HUE_FREQ_QUINTUPLET);

    // Straight has to mean straight: nothing odd left but a 1.
    uint32_t odd = (uint32_t) len;
    while ((odd & 1u) == 0u)
      odd >>= 1;
    CHECK(h != HUE_FREQ_STRAIGHT || odd == 1u);
  }
}

// A long pattern on anything but a very slow channel steps faster than the
// panel is redrawn. Sampling it anyway would alias it into a slow pulse and
// draw the busiest channel on the row as one of the calmest, so those free-run
// at the ceiling together instead.
TEST_CASE(a_pattern_stepping_past_the_ceiling_does_not_follow_its_own_phase)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_STA;

  for (uint8_t c = 0; c < 2; c++)
  {
    f.engine_config.channel_state[c].shape_mode    = SHAPE_STEPPED;
    f.engine_config.channel_state[c].st_length_idx = ST_LENGTH_COUNT - 1; // 64 steps
    f.engine_state.channels_effective[c].freq_hz   = 4.0f;                // 256 steps a second
  }
  // A half cycle apart in their own waveforms, and it must not show.
  f.engine_state.channels_effective[0].phase = 0.0f;
  f.engine_state.channels_effective[1].phase = 0.5f;

  ui_render(&f.ux);
  CHECK(value_of(led_of_channel(&f, 0)) == value_of(led_of_channel(&f, 1)));
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

// A press that acts on release used to show nothing at all until it had
// already happened. It dips out once as it crosses each of its thresholds -
// off, then back on - which says "that registered" without a pulse that keeps
// going and implies something is still in progress.
TEST_CASE(holding_a_clear_dips_once_as_it_crosses_each_threshold)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CLR;

  int8_t btn = f.ux_setup->channels[2].button;

  ui_render(&f.ux);
  LedRgb idle = led_of_channel(&f, 2);
  CHECK(lit(idle));

  // fixture_hold adds to the press, so these are increments, not totals.
  // Just past the point where a release would clear: dark, briefly.
  fixture_hold(&f, btn, MS(20));
  ui_render(&f.ux);
  CHECK(!lit(led_of_channel(&f, 2)));

  // Then back to what the page was showing - the dip was the message. Its own
  // colour, not its own bytes: this element is a candidate too, so the marker
  // is washing it and how far through that wash it is at any instant is a
  // property of the field rather than of the press being tested here.
  fixture_hold(&f, btn, MS(150));
  ui_render(&f.ux);
  LedRgb after = led_of_channel(&f, 2);
  CHECK(lit(after) && same_hue_order(after, idle));

  // Crossing into "every scene" dips again...
  fixture_hold(&f, btn, MS(350));
  ui_render(&f.ux);
  CHECK(!lit(led_of_channel(&f, 2)));

  // ...and comes back brighter, in the same colour.
  fixture_hold(&f, btn, MS(150));
  ui_render(&f.ux);
  LedRgb wide = led_of_channel(&f, 2);
  CHECK(wide.r > idle.r && wide.b > idle.b);
  CHECK(wide.r > wide.b && wide.b > wide.g); // pink, as the page is

  // Nothing has been cleared yet: that happens on the release.
  CHECK(f.engine_config.channel_state[2].params[0][CH_PARAM_FRQ] == -255);
}

// A scene has only one thing a press can mean, so it must not pretend to have
// a second stage.
TEST_CASE(a_one_stage_press_does_not_get_brighter_when_held)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CLR;

  int8_t btn = f.ux_setup->scenes[3].button;

  fixture_hold(&f, btn, MS(150));
  ui_render(&f.ux);
  LedRgb held = led_of_scene(&f, 3);
  CHECK(lit(held));

  fixture_hold(&f, btn, MS(500)); // well past UI_T_LONG and its dip
  ui_render(&f.ux);
  LedRgb still = led_of_scene(&f, 3);
  CHECK(still.r == held.r && still.g == held.g && still.b == held.b);
}

// Clearing is pink and selecting is purple: the page whose whole job is
// destructive should not wear the colour that means "off" on half the others.
TEST_CASE(clearing_and_selecting_do_not_share_a_colour)
{
  UiColor clear = ui_feedback_color(FB_CLEAR);
  UiColor write = ui_feedback_color(FB_WRITE);

  CHECK(clear.h == HUE_PINK);
  CHECK(write.h == HUE_PURPLE && write.h == UI_COL_SOURCE.h);
  CHECK(clear.h != ui_feedback_color(FB_ERROR).h);
}

// The marker washes an element rather than whitening it: it brightens and
// desaturates what is already there, and stops short of white so the element
// goes on saying what it is set to the whole time it is being advertised.
TEST_CASE(a_pickable_element_washes_toward_white_without_reaching_it)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CPY; // blue tint, everything pickable

  // Found rather than assumed. Which instant this element is brightest at is a
  // property of the field, and pinning one here would make the test a hostage
  // to every constant in ui_sparkle.h.
  LedRgb dim, bright;
  int have = 0;
  for (uint32_t t = 0; t < MARK_SPARKLE_PERIOD; t += 2000)
  {
    f.hw_state.time = t;
    ui_render(&f.ux);
    LedRgb l = led_of_channel(&f, 3);
    if (!have)
    {
      dim = bright = l;
      have         = 1;
    }
    if (l.b > bright.b)
      bright = l;
    if (l.b < dim.b)
      dim = l;
  }

  // Its own colour at both ends - the mode's blue tint, never white.
  CHECK(bluest(dim) && bluest(bright));
  CHECK(same_hue_order(bright, dim));

  // Brighter at the top, and washed out on the way: saturation is what the
  // marker animates, and the element keeps some of its own at the peak.
  CHECK(bright.b > dim.b);
  CHECK(less_saturated_than(bright, dim));
  CHECK(!(bright.r == bright.g && bright.g == bright.b)); // stops short of white
}

// The inverse, once a source is held: valid destinations are steady white and
// everything else goes dark, so nothing that cannot be pressed looks like it
// can be.
TEST_CASE(while_a_source_is_held_only_valid_destinations_light)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_MIX; // a channel takes an input, nothing else

  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  ui_render(&f.ux);

  LedRgb target = led_of_scene(&f, 2); // scene buttons are the inputs here
  CHECK(target.r > 0 && target.r == target.g && target.g == target.b);

  CHECK(!lit(led_of_channel(&f, 1))); // not a destination: dark
  CHECK(purple(led_of_channel(&f, 0)));

  // Scene buttons past the four inputs address nothing, so they stay dark too.
  CHECK(!lit(led_of_scene(&f, N_INPUTS)));

  // A destination sparkles like a candidate does, but it never goes out. There
  // is no colour underneath it to fall back to - everything that is not a
  // destination is dark - so a light the field happened to leave out would read
  // as "not a destination", which is the one thing this state must not say.
  for (uint32_t t = 0; t < MARK_SPARKLE_PERIOD * 3u; t += 5000)
  {
    f.hw_state.time = t;
    ui_render(&f.ux);
    LedRgb dest = led_of_scene(&f, 2);
    // White, and never dimmer than the VAL_LOW it used to sit at steadily.
    // Checked as a value: white reaches a brightness on a third of the duty a
    // single-die hue needs, so a duty here would be comparing the wrong thing.
    CHECK(dest.r == dest.g && dest.g == dest.b);
    CHECK(value_of(dest) >= VAL_LOW);
    CHECK(!lit(led_of_channel(&f, 1)));
  }
}

// Arm the FRQ display on every ring, which is the only state in which any of
// the frequency colour is drawn.
static void show_freq(Fixture* f)
{
  f->ui_state.shift_state         = SHIFT_STATE_NONE;
  f->engine_config.selected_param = CH_PARAM_FRQ;
  f->ui_state.param_display_hold  = UI_EDIT_DISPLAY;
}

// Put a channel at a known point of a known cycle, bypassing the oscillator.
static void set_pulse(Fixture* f, uint8_t ch, float hz, float phase)
{
  f->engine_state.channels_effective[ch].freq_hz = hz;
  f->engine_state.channels_effective[ch].phase   = phase;
}

// The frequency ratio is a code, not a level, and it must not be driven by the
// shared blink timers. It used to be multiplied by the fast blink to mark it as
// a code: survivable when touching one channel lit one LED, and a row of eight
// flashing green in unison once picking the parameter lit all of them. It now
// pulses at each channel's own rate instead, which is what the timers must not
// disturb.
TEST_CASE(the_frequency_colour_ignores_the_blink_timers)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);
  set_pulse(&f, 0, 1.0f, 0.0f);

  f.ui_state.blink_slow = 1;
  ui_render(&f.ux);
  LedRgb on = led_of_channel(&f, 0);

  f.ui_state.blink_slow = 0;
  ui_render(&f.ux);
  LedRgb off = led_of_channel(&f, 0);

  CHECK(lit(on));
  CHECK(on.r == off.r && on.g == off.g && on.b == off.b);
}

// The ring pulses at the channel's own output rate, which is the only thing on
// it that says whether a ratio is fast or slow.
TEST_CASE(the_frequency_ring_pulses_with_the_channel_phase)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);

  set_pulse(&f, 0, 1.0f, 0.0f); // peak of the raised cosine
  ui_render(&f.ux);
  uint16_t peak = value_of(led_of_channel(&f, 0));

  set_pulse(&f, 0, 1.0f, 0.5f); // trough
  ui_render(&f.ux);
  uint16_t trough = value_of(led_of_channel(&f, 0));

  CHECK(peak > trough);
  CHECK(peak == RING_PULSE_V_MAX);
  CHECK(trough == RING_PULSE_V_MIN);
}

// A dim pulse, not a blink: it never goes dark and never exceeds the brightness
// every other base layer uses, so the row stays readable throughout.
TEST_CASE(the_frequency_pulse_stays_inside_its_brightness_band)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);

  for (int i = 0; i <= 32; i++)
  {
    set_pulse(&f, 0, 1.0f, (float) i / 32.0f);
    ui_render(&f.ux);
    uint16_t v = value_of(led_of_channel(&f, 0));
    CHECK(v >= RING_PULSE_V_MIN && v <= RING_PULSE_V_MAX);
  }
}

// Past the ceiling the panel cannot resolve the phase, and sampling it anyway
// aliases the fastest channel into a slow pulse. Those free-run together
// instead, so a fast channel never reads as a slower one: its brightness stops
// depending on its own phase entirely.
TEST_CASE(a_channel_above_the_pulse_ceiling_does_not_follow_its_own_phase)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);

  float fast = (float) RING_PULSE_MAX_HZ * 10.0f;

  set_pulse(&f, 0, fast, 0.0f);
  set_pulse(&f, 1, fast, 0.5f);
  ui_render(&f.ux);

  // Same instant, same rate: two channels a half cycle apart in their own
  // waveforms must still light identically, because neither phase is being read.
  CHECK(value_of(led_of_channel(&f, 0)) == value_of(led_of_channel(&f, 1)));
}

// Hue is the ratio's prime limit and nothing else: 1/8 and 16 are both straight
// divisions and wear the same green, while 1/3 and 3/2 are both triplets.
TEST_CASE(the_frequency_hue_codes_the_prime_limit_of_the_ratio)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);

  // Every channel at the pulse peak, so only hue differs between them.
  for (uint8_t c = 0; c < N_CHANNELS; c++)
    set_pulse(&f, c, 1.0f, 0.0f);

  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, -1785); // 1/8  straight
  fixture_set_param(&f, 1, 0, CH_PARAM_FRQ, 3825);  // 16   straight
  fixture_set_param(&f, 2, 0, CH_PARAM_FRQ, -510);  // 1/3  triplet
  fixture_set_param(&f, 3, 0, CH_PARAM_FRQ, 128);   // 3/2  triplet
  fixture_set_param(&f, 4, 0, CH_PARAM_FRQ, 1020);  // 5    quintuplet
  ui_render(&f.ux);

  LedRgb eighth = led_of_channel(&f, 0);
  LedRgb six16  = led_of_channel(&f, 1);
  LedRgb third  = led_of_channel(&f, 2);
  LedRgb dotted = led_of_channel(&f, 3);
  LedRgb five   = led_of_channel(&f, 4);

  // Octaves apart, same class, so byte-identical.
  CHECK(eighth.r == six16.r && eighth.g == six16.g && eighth.b == six16.b);
  CHECK(third.r == dotted.r && third.g == dotted.g && third.b == dotted.b);

  // Green -> yellow -> orange: green has no red in it, and the two warm classes
  // are separated by how much green is left.
  CHECK(greenest(eighth));
  CHECK(third.r > 0 && third.g > 0);
  CHECK(five.r > 0 && five.g > 0);
  CHECK(third.g > five.g); // yellow keeps more green than orange
}

// The peak runs warmer than the trough, which is what gives the pulse contrast
// beyond brightness alone.
TEST_CASE(the_frequency_pulse_warms_the_hue_as_it_brightens)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0); // 1x, straight -> green

  set_pulse(&f, 0, 1.0f, 0.5f); // trough
  ui_render(&f.ux);
  LedRgb trough = led_of_channel(&f, 0);

  set_pulse(&f, 0, 1.0f, 0.0f); // peak
  ui_render(&f.ux);
  LedRgb peak = led_of_channel(&f, 0);

  // Warmer means proportionally more red against the green it is mixed into.
  CHECK(peak.r * trough.g > trough.r * peak.g);
}

// The hue swing must not be able to carry one class into another's colour.
// Quintuplet and triplet are the closest pair on the wheel, so a quintuplet at
// its brightest and a triplet at its dimmest are the worst case there is: if
// those two are still told apart, every other pairing is.
TEST_CASE(the_pulse_never_swings_one_frequency_class_into_another)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);

  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 1020); // 5    quintuplet
  fixture_set_param(&f, 1, 0, CH_PARAM_FRQ, 510);  // 3    triplet
  set_pulse(&f, 0, 1.0f, 0.0f);                    // quintuplet at its peak
  set_pulse(&f, 1, 1.0f, 0.5f);                    // triplet at its trough
  ui_render(&f.ux);

  LedRgb quint = led_of_channel(&f, 0);
  LedRgb trip  = led_of_channel(&f, 1);

  // Compared as a ratio, because the two are at opposite ends of the pulse and
  // so are nowhere near the same brightness. Orange keeps less green against
  // its red than yellow does, at any brightness.
  CHECK(quint.g * trip.r < trip.g * quint.r);
}

// A value between two ratios washes out, so a fine adjust is visible as one -
// but never all the way to white, which belongs to assignment alone.
TEST_CASE(a_value_off_the_frequency_grid_desaturates)
{
  Fixture f;
  fixture_init(&f);
  show_freq(&f);
  for (uint8_t c = 0; c < N_CHANNELS; c++)
    set_pulse(&f, c, 1.0f, 0.0f);

  // All three still round to 1x, so the hue is identical and saturation is the
  // only thing that can differ between them.
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0);  // exactly 1x
  fixture_set_param(&f, 1, 0, CH_PARAM_FRQ, 1);  // inside the deadzone
  fixture_set_param(&f, 2, 0, CH_PARAM_FRQ, 24); // well off the grid
  ui_render(&f.ux);

  LedRgb snapped = led_of_channel(&f, 0);
  LedRgb nearly  = led_of_channel(&f, 1);
  LedRgb between = led_of_channel(&f, 2);

  // Saturation shows up as how much of the other primaries are mixed in: a pure
  // hue leaves them at the floor, a washed one lifts them.
  CHECK(between.r > snapped.r);
  CHECK(between.b > snapped.b);

  // Rounding must not cost a snapped ratio its pure colour.
  CHECK(nearly.r == snapped.r && nearly.g == snapped.g && nearly.b == snapped.b);

  // Still a tinted pastel, not white: the floor stays well under the peak.
  // Both sides in framebuffer units, since this is about the shape of the
  // colour rather than about how bright it was drawn.
  uint16_t peak = between.r > between.g ? between.r : between.g;
  CHECK(between.b * 2 < peak);
}

// With no mode active the ring is a level meter, and the parameter being edited
// only borrows it on touch.
TEST_CASE(the_selected_parameter_shows_on_touch_and_decays_back_to_the_output_level)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                                                             = SHIFT_STATE_NONE;
  f.engine_config.selected_param                                                     = CH_PARAM_AMP;
  f.engine_state.channels_output_level[1]                                            = -DAC_5V; // red as a base
  f.engine_config.channel_state[1].params[f.engine_state.active_scene][CH_PARAM_AMP] = ADC_5V / 2;

  f.ui_state.param_display_hold = UI_EDIT_DISPLAY;
  ui_render(&f.ux);
  LedRgb touched = led_of_channel(&f, 1);
  CHECK(touched.g > 0 && touched.r == 0); // the positive parameter, not the level

  f.ui_state.param_display_hold = 0;
  ui_render(&f.ux);
  LedRgb settled = led_of_channel(&f, 1);
  CHECK(settled.r > 0 && settled.g == 0); // back to the negative output level
}

/* ---- the bipolar ramp -------------------------------------------------- */

// Total light, in the same weighted units the ramp is built in. What the eye
// integrates, which is the only thing worth asserting about a colour whose two
// primaries are deliberately not equally efficient.
static float luminance(LedRgb c) { return (float) c.r * LED_W_RED + (float) c.g * LED_W_GREEN + (float) c.b * LED_W_BLUE; }

// The defect that made the old ring untrustworthy: the green die is roughly
// twice the red one, so at equal duty a positive level outshone the negative
// level of the same size and neither end could be read against the other.
TEST_CASE(equal_magnitudes_read_equally_bright_on_both_polarities)
{
  Fixture f;
  fixture_init(&f);

  const int32_t levels[] = {DAC_5V / 8, DAC_5V / 2, DAC_5V, 2 * DAC_5V};
  for (unsigned i = 0; i < sizeof levels / sizeof levels[0]; i++)
  {
    led_set_dac(&f.ux, 0, levels[i]);
    led_set_dac(&f.ux, 1, -levels[i]);

    float pos = luminance(f.engine_state.leds[0]);
    float neg = luminance(f.engine_state.leds[1]);
    // Within a rounding step of each other, not exactly equal: the two land on
    // different dies and each rounds to its own fraction of a duty step.
    CHECK(pos - neg < 4.0f && neg - pos < 4.0f);
  }
}

// Perceived lightness, not duty, is what rises evenly with voltage. Asserted as
// monotonicity plus a share of the range, since the curve's shape is a tuning
// constant and pinning the numbers would only make led_curve.h unedittable.
TEST_CASE(the_ramp_rises_monotonically_and_spends_most_of_itself_below_5v)
{
  Fixture f;
  fixture_init(&f);

  float last = -1.0f;
  for (int i = 0; i <= 20; i++)
  {
    led_set_dac(&f.ux, 0, (int32_t) i * 2 * DAC_5V / 20);
    float y = luminance(f.engine_state.leds[0]);
    CHECK(y > last);
    last = y;
  }

  // The knee: 5V is most of the way up the *perceived* scale even though it is
  // half the voltage, which is what buys resolution where patches live.
  led_set_dac(&f.ux, 0, DAC_5V);
  float at_5v = luminance(f.engine_state.leds[0]);
  led_set_dac(&f.ux, 0, 2 * DAC_5V);
  float at_10v = luminance(f.engine_state.leds[0]);

  CHECK(powf(at_5v / at_10v, 1.0f / LED_GAMMA) > 0.6f);
}

// Blue is gone: no level, at any magnitude or either polarity, leaves the
// red-green family.
TEST_CASE(no_output_level_lights_the_blue_die)
{
  Fixture f;
  fixture_init(&f);

  for (int i = -24; i <= 24; i++)
  {
    led_set_dac(&f.ux, 0, (int32_t) i * DAC_5V / 8);
    CHECK(f.engine_state.leds[0].b == 0);
  }
}

// The same ramp draws the input jacks and the parameter row, not only the
// channel rings, so off has to mean off: an unpatched jack that glows is a jack
// that looks patched. LED_CV_FLOOR is what would trade that away, and at its
// default it does not.
TEST_CASE(zero_volts_is_dark_and_polarity_emerges_either_side_of_it)
{
  Fixture f;
  fixture_init(&f);

  led_set_dac(&f.ux, 0, 0);
  LedRgb zero = f.engine_state.leds[0];

  if (LED_CV_FLOOR == 0.0f)
  {
    CHECK(!lit(zero));
  }
  else
  {
    // Raised, it is an ember - and a neutral one, the two dies carrying the
    // same light whatever duty that costs each, so the crossing stays visible.
    CHECK(zero.r > 0 && zero.g > 0);
    float r = (float) zero.r * LED_W_RED, g = (float) zero.g * LED_W_GREEN;
    CHECK(r - g < 4.0f && g - r < 4.0f);
  }

  // Either way it is the bottom of the scale and not a clamp across it: a level
  // the ramp has something to say about stands well clear of resting.
  led_set_dac(&f.ux, 0, DAC_5V / 4);
  CHECK(luminance(f.engine_state.leds[0]) > luminance(zero) * 2.5f + 8.0f);
  CHECK(greenest(f.engine_state.leds[0]));

  led_set_dac(&f.ux, 0, -DAC_5V / 4);
  CHECK(f.engine_state.leds[0].r > f.engine_state.leds[0].g);
}

// Past half scale the colour warms - the only cue left that a signal is outside
// the usual range once blue is gone - without either polarity reaching for the
// other's colour.
TEST_CASE(the_top_of_the_range_warms_without_crossing_polarities)
{
  Fixture f;
  fixture_init(&f);

  led_set_dac(&f.ux, 0, DAC_5V);      // 5V: the knee, still pure
  led_set_dac(&f.ux, 1, 2 * DAC_5V);  // 10V
  led_set_dac(&f.ux, 2, -2 * DAC_5V); // -10V

  CHECK(f.engine_state.leds[0].r == 0);
  CHECK(f.engine_state.leds[1].r > 0); // warmed toward yellow-green
  CHECK(f.engine_state.leds[2].g > 0); // warmed toward orange

  CHECK(greenest(f.engine_state.leds[1]));
  CHECK(f.engine_state.leds[2].r > f.engine_state.leds[2].g);
}

// The old mapping masked its blue term with 0xFF, so a param value past twice
// half scale rolled over and drew a full ring as a nearly dark one.
TEST_CASE(a_level_past_full_scale_saturates_rather_than_wrapping)
{
  Fixture f;
  fixture_init(&f);

  led_set_dac(&f.ux, 0, 2 * DAC_5V);
  float full = luminance(f.engine_state.leds[0]);

  const int32_t over[] = {3 * DAC_5V, 100 * DAC_5V, INT32_MAX, INT32_MIN};
  for (unsigned i = 0; i < sizeof over / sizeof over[0]; i++)
  {
    led_set_dac(&f.ux, 1, over[i]);
    float y = luminance(f.engine_state.leds[1]);
    CHECK(y >= full - 4.0f && y <= full + 4.0f);
  }
}

/* ---- the dither -------------------------------------------------------- */

// What the fraction is for: a level under one duty step still averages to what
// it was asked for, instead of rounding to nothing or to a step it never meant.
TEST_CASE(the_dither_averages_to_the_framebuffer_over_a_run_of_frames)
{
  const uint16_t wanted[] = {0, 1, 64, 128, 200, LED_UNIT / 2, LED_UNIT, 3 * LED_UNIT / 2, 7 * LED_UNIT};

  for (unsigned w = 0; w < sizeof wanted / sizeof wanted[0]; w++)
  {
    LedRgb fb[1]     = {{wanted[w], wanted[w], wanted[w]}};
    LedDither acc[1] = {{0, 0, 0}};
    const int frames = 256; // one full turn of the accumulator
    uint32_t total   = 0;
    uint8_t out[3];

    for (int i = 0; i < frames; i++)
    {
      led_fb_quantize(fb, acc, out, 1);
      total += out[0];
    }

    // Exact over a whole period, since the accumulator is a plain carry.
    CHECK(total * LED_UNIT == (uint32_t) wanted[w] * frames);
  }
}

// The safety argument for dithering a panel at 300 frames a second: the swing
// is never more than one duty step, so at the levels this matters at - under
// four steps out of 255 - the flicker is a fraction of a percent of full scale.
TEST_CASE(the_dither_never_swings_more_than_one_duty_step)
{
  LedRgb fb[1]     = {{5 * LED_UNIT / 2, 1, 200}};
  LedDither acc[1] = {{0, 0, 0}};
  uint8_t out[3];

  uint8_t lo[3] = {255, 255, 255}, hi[3] = {0, 0, 0};
  for (int i = 0; i < 512; i++)
  {
    led_fb_quantize(fb, acc, out, 1);
    for (int c = 0; c < 3; c++)
    {
      if (out[c] < lo[c])
        lo[c] = out[c];
      if (out[c] > hi[c])
        hi[c] = out[c];
    }
  }

  for (int c = 0; c < 3; c++)
    CHECK(hi[c] - lo[c] <= 1);
}

// A whole-step colour is not something to dither: every frame must come out
// identical, or the palette would shimmer for no gain.
TEST_CASE(a_whole_step_colour_quantizes_to_the_same_bytes_every_frame)
{
  LedRgb fb[2]     = {{VAL_MED * LED_UNIT, 0, VAL_LOW * LED_UNIT}, {0, VAL_HIG * LED_UNIT, 0}};
  LedDither acc[2] = {{0, 0, 0}, {0, 0, 0}};
  uint8_t out[6];

  for (int i = 0; i < 64; i++)
  {
    led_fb_quantize(fb, acc, out, 2);
    CHECK(out[0] == VAL_MED && out[1] == 0 && out[2] == VAL_LOW);
    CHECK(out[3] == 0 && out[4] == VAL_HIG && out[5] == 0);
  }
}

// The framebuffer can hold more than the driver takes once the constants are
// edited, and that has to clip rather than wrap.
TEST_CASE(the_dither_saturates_instead_of_wrapping_past_full_duty)
{
  LedRgb fb[1]     = {{255 * LED_UNIT, 255 * LED_UNIT + 200, 254 * LED_UNIT}};
  LedDither acc[1] = {{0, 0, 0}};
  uint8_t out[3];

  for (int i = 0; i < 32; i++)
  {
    led_fb_quantize(fb, acc, out, 1);
    CHECK(out[0] == 255 && out[1] == 255 && out[2] == 254);
  }
}

/* ---- the assignment marker's field ------------------------------------- */

// Whatever MARK_SPARKLE_DUTY leaves over is a flash: every candidate full on
// together, once per period. It is the only moment that says "all of these",
// which a field with no floor under it never promises on its own - so what is
// pinned here is that it happens, that every light is in it, and that it stays
// brief. At a duty of 1.0 there is no flash and the bounded gap is the whole
// guarantee; both are checked, so turning it back on cannot silently do
// nothing.
TEST_CASE(the_marker_flashes_every_light_together_for_whatever_its_duty_leaves)
{
  int all_on  = 0;
  const int n = 2000;

  for (int i = 0; i < n; i++)
  {
    uint32_t t = (uint32_t) ((uint64_t) i * MARK_SPARKLE_PERIOD / n);

    float dimmest = 1.0f;
    for (int16_t led = 0; led < LED_COUNT; led++)
    {
      float level = ui_sparkle_level(t, led, SPARKLE_MARK);
      CHECK(level >= 0.0f && level <= 1.0f);
      if (level < dimmest)
        dimmest = level;
    }

    // Every light at once, not merely a bright moment in the field.
    if (dimmest > 0.95f)
      all_on++;
  }

  float flash = (float) all_on / (float) n;
  if (MARK_SPARKLE_DUTY >= 1.0f)
  {
    // Sparkle the whole way through: there is no moment the panel is lit
    // together, and the guarantee that a candidate reads as one is the bounded
    // gap below rather than a flash.
    CHECK(flash == 0.0f);
  }
  else
  {
    CHECK(flash > 0.0f);
    CHECK(flash < 1.0f - MARK_SPARKLE_DUTY + 0.02f);
  }
}

// The point of the whole thing: at any instant the panel is not one brightness.
TEST_CASE(the_marker_does_not_light_every_led_alike)
{
  // Mid-window, where the envelope is not the thing making them differ.
  uint32_t t = MARK_SPARKLE_PERIOD / 4;

  float lo = 2.0f, hi = -1.0f;
  for (int16_t led = 0; led < LED_COUNT; led++)
  {
    float level = ui_sparkle_level(t, led, SPARKLE_MARK);
    if (level < lo)
      lo = level;
    if (level > hi)
      hi = level;
  }

  // A real spread, not two LEDs a rounding step apart.
  CHECK(hi - lo > 0.15f);
}

// Neighbours differ, but the field is a field: it is sampled from a continuous
// surface, so no light can jump. A marker that stepped would read as the gate
// this replaced.
TEST_CASE(the_marker_moves_continuously_across_the_whole_period)
{
  const uint32_t step = 1000; // 1ms, well under the panel's own frame

  // A tenth of the range per step. Loose, deliberately: what this rules out is
  // a *gate*, which moves the whole range between one sample and the next, and
  // the bound has to survive MARK_SPARKLE_HZ being turned up without becoming a
  // second place to record how fast the field travels.
  //
  // Across the whole period, not just the sparkling part: the flash arriving
  // and leaving is the easiest place for a step to hide, and a marker that
  // snapped to full on would undo the point of animating it at all.
  for (int16_t led = 0; led < LED_COUNT; led += 5)
  {
    for (int kind = 0; kind < 2; kind++)
    {
      SparkleKind k = kind ? SPARKLE_TARGET : SPARKLE_MARK;
      float prev    = ui_sparkle_level(0, led, k);
      for (uint32_t t = step; t < MARK_SPARKLE_PERIOD; t += step)
      {
        float level = ui_sparkle_level(t, led, k);
        CHECK(level - prev < 0.1f && prev - level < 0.1f);
        prev = level;
      }
    }
  }
}

// How long a candidate can sit unlit, which is what decides whether it still
// reads as pickable.
//
// The threshold is not "above zero" but "above the base layer it is laid over":
// a marker dimmer than the colour underneath it does nothing to the panel.
//
// Which bound applies depends on the flash. With it, every candidate is full on
// once a period and the gap cannot exceed one; without it the field is the only
// thing lighting anything, and at a sparse gate a light can sit out for many
// seconds. Both are asserted, so the sparsity cannot quietly be turned up until
// nothing ever lights, and turning the flash back on cannot quietly stop
// bounding what it is there to bound.
TEST_CASE(the_flash_is_what_bounds_how_long_a_candidate_sits_unlit)
{
  // A perceived level scaled to MARK_SPARKLE_V, against a base layer at
  // VAL_LOW. Below this the marker is not on the panel at all.
  float visible = powf((float) VAL_LOW / (float) MARK_SPARKLE_V, 1.0f / LED_GAMMA);

  const uint32_t step = 5000;
  const uint32_t span = MARK_SPARKLE_PERIOD * 40u;

  for (int16_t led = 0; led < LED_COUNT; led++)
  {
    uint32_t last_seen = 0, worst = 0;
    for (uint32_t t = 0; t < span; t += step)
    {
      if (ui_sparkle_level(t, led, SPARKLE_MARK) > visible)
      {
        if (t - last_seen > worst)
          worst = t - last_seen;
        last_seen = t;
      }
    }
    if (MARK_SPARKLE_DUTY < 1.0f)
      CHECK(worst < MARK_SPARKLE_PERIOD * 2u); // the flash, once a period
    else
      CHECK(worst < span / 2u); // only that nothing is permanently dark
  }
}

// Over a long enough run every light gets its turn at the top, so no position
// on the panel is permanently the dull one.
TEST_CASE(every_led_reaches_the_bright_end_of_the_field_eventually)
{
  for (int16_t led = 0; led < LED_COUNT; led++)
  {
    float best = 0.0f;
    for (int i = 0; i < 400; i++)
    {
      // Sampled at the peak of successive windows, so the envelope is out of it.
      uint32_t t  = (uint32_t) i * MARK_SPARKLE_PERIOD + MARK_SPARKLE_PERIOD / 4;
      float level = ui_sparkle_level(t, led, SPARKLE_MARK);
      if (level > best)
        best = level;
    }
    CHECK(best > 0.9f);
  }
}

// The field is spread over the box the lights occupy, and those bounds are
// written down rather than derived. `just panel` regenerating the layout after
// a board change has to fail here rather than quietly leaving a row of lights
// outside the field, sampling whatever the noise does past its edge.
TEST_CASE(the_field_bounds_are_the_leds_own_extent)
{
  float x0 = panel_led_pos[0].x, x1 = x0, y0 = panel_led_pos[0].y, y1 = y0;
  for (int16_t led = 1; led < LED_COUNT; led++)
  {
    PanelPoint p = panel_led_pos[led];
    x0           = p.x < x0 ? p.x : x0;
    x1           = p.x > x1 ? p.x : x1;
    y0           = p.y < y0 ? p.y : y0;
    y1           = p.y > y1 ? p.y : y1;
  }

  CHECK(x0 == MARK_FIELD_X0 && x1 == MARK_FIELD_X1);
  CHECK(y0 == MARK_FIELD_Y0 && y1 == MARK_FIELD_Y1);
}

// The two states share a field and differ in urgency, and the difference has to
// survive the constants being tuned - a target that rests, or that drops a
// light, is a target that reads as "not a destination" mid-gesture.
TEST_CASE(a_held_source_target_never_rests_where_a_candidate_mark_does)
{
  int mark_dark = 0;

  for (uint32_t t = 0; t < MARK_SPARKLE_PERIOD * 3u; t += 5000)
  {
    for (int16_t led = 0; led < LED_COUNT; led++)
    {
      CHECK(ui_sparkle_level(t, led, SPARKLE_TARGET) >= TARGET_SPARKLE_FLOOR - 0.001f);
      if (ui_sparkle_level(t, led, SPARKLE_MARK) <= 0.0f)
        mark_dark++;
    }
  }

  // And the mark does rest - otherwise the two are the same animation and the
  // panel has stopped saying which state it is in.
  CHECK(mark_dark > 0);
}

// Correlation of one light against another, offset by the time the field should
// take to travel between them.
static double sparkle_corr(int16_t a, int16_t b, double lag_us)
{
  double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
  int n = 0;
  for (double t = 100000; t < (double) MARK_SPARKLE_PERIOD * 40.0; t += 2000)
  {
    double x = ui_sparkle_level((uint32_t) t, a, SPARKLE_MARK);
    double y = ui_sparkle_level((uint32_t) (t + lag_us), b, SPARKLE_MARK);
    sa += x, sb += y, saa += x * x, sbb += y * y, sab += x * y, n++;
  }
  double ca = saa - sa * sa / n, cb = sbb - sb * sb / n;
  return (ca > 0 && cb > 0) ? (sab - sa * sb / n) / sqrt(ca * cb) : 0.0;
}

// The field travels left to right, and holds together well enough doing it to
// be seen as travel.
//
// Both halves are asserted, and the second is the one that earns its keep. A
// sign test alone passes on a field with no direction in it at all - which is
// exactly what the panel had while the churn was fast enough to replace a
// feature before it finished crossing: the direction was right, the effect
// measured 0.02, and it looked like noise. So this pins a margin, and a churn
// or drift that stops a feature outliving its crossing fails here rather than
// in somebody's eyes a week later.
TEST_CASE(the_field_travels_left_to_right_and_holds_together_doing_it)
{
  struct
  {
    int16_t l, r;
  } pair[] = {{5, 2}, {7, 1}, {8, 13}, {20, 14}};

  for (unsigned i = 0; i < sizeof pair / sizeof pair[0]; i++)
  {
    double cells = (panel_led_pos[pair[i].r].x - panel_led_pos[pair[i].l].x) / (MARK_FIELD_X1 - MARK_FIELD_X0) * (double) MARK_FIELD_COLS;
    double lag   = cells / MARK_SPARKLE_DRIFT * 1e6;

    double travel  = sparkle_corr(pair[i].l, pair[i].r, lag);
    double reverse = sparkle_corr(pair[i].l, pair[i].r, -lag);

    CHECK(travel > reverse);       // the right way round...
    CHECK(travel - reverse > 0.3); // ...and by enough to read as travel
  }
}

// An index off the panel has no position to sample, and must not read one.
TEST_CASE(the_marker_ignores_an_led_index_off_the_panel)
{
  uint32_t t = MARK_SPARKLE_PERIOD / 4;
  CHECK(ui_sparkle_level(t, -1, SPARKLE_MARK) == 0.0f);
  CHECK(ui_sparkle_level(t, LED_COUNT, SPARKLE_MARK) == 0.0f);
}

int main(void)
{
  RUN_TEST(hsv_maps_onto_expected_rgb);
  RUN_TEST(bipolar_cv_shows_green_when_positive_and_red_when_negative);
  RUN_TEST(out_of_range_led_index_is_ignored_rather_than_corrupting_memory);
  RUN_TEST(the_selected_param_button_stands_out_from_the_dim_others);
  RUN_TEST(parameter_buttons_go_dark_while_a_shift_mode_is_running);
  RUN_TEST(active_shift_mode_button_blinks_with_the_slow_blink_signal);
  RUN_TEST(a_confirmation_flash_overrides_the_candidate_highlight_beneath_it);
  RUN_TEST(every_channel_and_scene_led_is_written_in_every_mode);
  RUN_TEST(a_muted_channel_reads_purple_wherever_mute_is_what_the_led_shows);
  RUN_TEST(a_shift_mode_shows_its_own_setting_and_never_the_output_level);
  RUN_TEST(a_mode_with_no_channel_setting_leaves_the_channel_leds_dark);
  RUN_TEST(the_mute_page_shows_green_for_passing_and_purple_for_gated);
  RUN_TEST(pattern_length_only_lights_the_channels_it_applies_to);
  RUN_TEST(the_division_hue_codes_the_prime_limit_and_is_shared_by_both_pages);
  RUN_TEST(every_pattern_length_lands_on_a_division_class);
  RUN_TEST(a_pattern_stepping_past_the_ceiling_does_not_follow_its_own_phase);
  RUN_TEST(the_output_clamp_reads_as_polarity_by_hue_and_range_by_brightness);
  RUN_TEST(holding_a_clear_dips_once_as_it_crosses_each_threshold);
  RUN_TEST(clearing_and_selecting_do_not_share_a_colour);
  RUN_TEST(a_one_stage_press_does_not_get_brighter_when_held);
  RUN_TEST(a_pickable_element_washes_toward_white_without_reaching_it);
  RUN_TEST(while_a_source_is_held_only_valid_destinations_light);
  RUN_TEST(the_frequency_colour_ignores_the_blink_timers);
  RUN_TEST(the_frequency_ring_pulses_with_the_channel_phase);
  RUN_TEST(the_frequency_pulse_stays_inside_its_brightness_band);
  RUN_TEST(a_channel_above_the_pulse_ceiling_does_not_follow_its_own_phase);
  RUN_TEST(the_frequency_hue_codes_the_prime_limit_of_the_ratio);
  RUN_TEST(the_frequency_pulse_warms_the_hue_as_it_brightens);
  RUN_TEST(the_pulse_never_swings_one_frequency_class_into_another);
  RUN_TEST(a_value_off_the_frequency_grid_desaturates);
  RUN_TEST(the_selected_parameter_shows_on_touch_and_decays_back_to_the_output_level);
  RUN_TEST(equal_magnitudes_read_equally_bright_on_both_polarities);
  RUN_TEST(the_ramp_rises_monotonically_and_spends_most_of_itself_below_5v);
  RUN_TEST(no_output_level_lights_the_blue_die);
  RUN_TEST(zero_volts_is_dark_and_polarity_emerges_either_side_of_it);
  RUN_TEST(the_top_of_the_range_warms_without_crossing_polarities);
  RUN_TEST(a_level_past_full_scale_saturates_rather_than_wrapping);
  RUN_TEST(the_dither_averages_to_the_framebuffer_over_a_run_of_frames);
  RUN_TEST(the_dither_never_swings_more_than_one_duty_step);
  RUN_TEST(a_whole_step_colour_quantizes_to_the_same_bytes_every_frame);
  RUN_TEST(the_dither_saturates_instead_of_wrapping_past_full_duty);
  RUN_TEST(the_marker_flashes_every_light_together_for_whatever_its_duty_leaves);
  RUN_TEST(the_marker_does_not_light_every_led_alike);
  RUN_TEST(the_marker_moves_continuously_across_the_whole_period);
  RUN_TEST(the_flash_is_what_bounds_how_long_a_candidate_sits_unlit);
  RUN_TEST(every_led_reaches_the_bright_end_of_the_field_eventually);
  RUN_TEST(a_held_source_target_never_rests_where_a_candidate_mark_does);
  RUN_TEST(the_field_bounds_are_the_leds_own_extent);
  RUN_TEST(the_field_travels_left_to_right_and_holds_together_doing_it);
  RUN_TEST(the_marker_ignores_an_led_index_off_the_panel);
  return TESTKIT_SUMMARY();
}
