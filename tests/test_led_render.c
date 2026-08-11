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

static LedRgb led_of_channel(Fixture* f, uint8_t ch) { return f->engine_state.leds[f->ux_setup->channels[ch].led]; }
static LedRgb led_of_scene(Fixture* f, uint8_t s) { return f->engine_state.leds[f->ux_setup->scenes[s].led]; }
static int lit(LedRgb c) { return c.r > 0 || c.g > 0 || c.b > 0; }
static int purple(LedRgb c) { return c.b > 0 && c.r > 0 && c.g == 0; }
// Which primary dominates, for colours bright enough that the others are not
// quite zero.
static int bluest(LedRgb c) { return c.b > c.r && c.b > c.g; }
static int greenest(LedRgb c) { return c.g > c.r && c.g > c.b; }

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
  f.ui_state.shift_state         = SHIFT_STATE_MON;
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
  CHECK(bluest(confirmed) && confirmed.b > candidate.b);
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
  f.engine_state.channels_output_level[1]     = -DAC_5V;   // red, were it shown
  f.engine_config.channel_state[1].shape_mode = SHAPE_PWM; // yellow as a value

  ui_render(&f.ux);
  LedRgb l = led_of_channel(&f, 1);
  CHECK(l.g > 0 && l.b == 0); // the shape colour...

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
  CHECK(passing.g > 0 && passing.b == 0);
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

// A press that acts on release used to show nothing at all until it had
// already happened. It dips out once as it crosses each of its thresholds -
// off, then back on - which says "that registered" without a pulse that keeps
// going and implies something is still in progress.
TEST_CASE(holding_a_clear_dips_once_as_it_crosses_each_threshold)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CLR;
  f.ui_state.blink_mark  = 0;

  int8_t btn = f.ux_setup->channels[2].button;

  ui_render(&f.ux);
  LedRgb idle = led_of_channel(&f, 2);
  CHECK(lit(idle));

  // fixture_hold adds to the press, so these are increments, not totals.
  // Just past the point where a release would clear: dark, briefly.
  fixture_hold(&f, btn, MS(20));
  ui_render(&f.ux);
  CHECK(!lit(led_of_channel(&f, 2)));

  // Then back to exactly what the page was showing - the dip was the message.
  fixture_hold(&f, btn, MS(150));
  ui_render(&f.ux);
  LedRgb after = led_of_channel(&f, 2);
  CHECK(after.r == idle.r && after.g == idle.g && after.b == idle.b);

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
  f.ui_state.blink_mark  = 0;

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

static uint8_t brightest(LedRgb c)
{
  uint8_t m = c.r > c.g ? c.r : c.g;
  return m > c.b ? m : c.b;
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
  f.ui_state.blink_mark = 1;
  ui_render(&f.ux);
  LedRgb on = led_of_channel(&f, 0);

  f.ui_state.blink_slow = 0;
  f.ui_state.blink_mark = 0;
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
  uint8_t peak = brightest(led_of_channel(&f, 0));

  set_pulse(&f, 0, 1.0f, 0.5f); // trough
  ui_render(&f.ux);
  uint8_t trough = brightest(led_of_channel(&f, 0));

  CHECK(peak > trough);
  CHECK(peak == FREQ_PULSE_V_MAX);
  CHECK(trough == FREQ_PULSE_V_MIN);
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
    uint8_t v = brightest(led_of_channel(&f, 0));
    CHECK(v >= FREQ_PULSE_V_MIN && v <= FREQ_PULSE_V_MAX);
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

  float fast = (float) FREQ_PULSE_MAX_HZ * 10.0f;

  set_pulse(&f, 0, fast, 0.0f);
  set_pulse(&f, 1, fast, 0.5f);
  ui_render(&f.ux);

  // Same instant, same rate: two channels a half cycle apart in their own
  // waveforms must still light identically, because neither phase is being read.
  CHECK(brightest(led_of_channel(&f, 0)) == brightest(led_of_channel(&f, 1)));
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
  CHECK(between.b * 2 < brightest(between));
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
  RUN_TEST(the_output_clamp_reads_as_polarity_by_hue_and_range_by_brightness);
  RUN_TEST(holding_a_clear_dips_once_as_it_crosses_each_threshold);
  RUN_TEST(clearing_and_selecting_do_not_share_a_colour);
  RUN_TEST(a_one_stage_press_does_not_get_brighter_when_held);
  RUN_TEST(a_pickable_element_keeps_its_state_colour_and_marks_itself_white);
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
  return TESTKIT_SUMMARY();
}
