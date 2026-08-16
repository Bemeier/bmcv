#include "ui_render.h"
#include "color_presets.h"
#include "config.h"
#include "error.h"
#include "helpers.h"
#include "hw_setup.h"
#include "led_curve.h"
#include "led_fb.h"
#include "stepped_random.h"       // sr_length_for_index
#include "stepped_random_table.h" // SR_LENGTH_COUNT
#include "ui_channel.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"
#include "ui_sparkle.h"
#include "ui_state.h"
#include "ux_state.h"
#include <math.h>
#include <stdint.h>

// Which hue each value of a per-channel or per-input setting reads as. The
// concept, not the mode, picks the hue - see the HUE_STATE_* table in
// color_presets.h - so index 0 is purple in every one of these, and "clocked"
// looks the same on the SYS page as it does on QNT.
static const uint8_t quantize_mode_color[QUANTIZE_MODE_COUNT]   = {HUE_STATE_DEFAULT, HUE_STATE_LEVEL, HUE_STATE_EVENT};
static const uint8_t input_amp_mode_color[INPUT_AMP_MODE_COUNT] = {HUE_STATE_DEFAULT, HUE_STATE_MIX, HUE_STATE_MULT};
// The wavetable is the default; stepped random is a continuously varying level,
// and PWM is a gate.
//
// Two axes, like clamp_mode_color below: the stepped modes are one family and
// share its hue, and which member is which is carried by brightness. Giving the
// second one a hue of its own would have meant either a fifth colour on a page
// with four meanings, or borrowing one that already means something else.
static const UiColor shape_mode_color[SHAPE_MODE_COUNT] = {
    [SHAPE_LFO]          = {HUE_STATE_DEFAULT, SAT_HIG, VAL_BASE},
    [SHAPE_STEPPED]      = {HUE_STATE_LEVEL, SAT_HIG, VAL_BASE},
    [SHAPE_PWM]          = {HUE_STATE_EVENT, SAT_HIG, VAL_BASE},
    [SHAPE_STEPPED_CTRL] = {HUE_STATE_LEVEL, SAT_HIG, VAL_LOW},
};
static const uint8_t input_mode_color[INPUT_MODE_COUNT] = {HUE_STATE_DEFAULT, HUE_STATE_EVENT, HUE_STATE_RESET, HUE_STATE_LEVEL};

// The output clamp is two facts - polarity and range - so it uses two axes:
// purple for bipolar and green for unipolar, dim for the half range. The one
// place brightness carries meaning in a base layer.
//
// The full-range settings sit at VAL_BASE, which is where they always sat -
// this page is where the base level came from, since it was already a step
// above the rest and the rest were levelled up to meet it.
static const UiColor clamp_mode_color[CLAMP_MODE_COUNT] = {
    [CLAMP_BI_10]  = {HUE_PURPLE, SAT_HIG, VAL_BASE},
    [CLAMP_BI_5]   = {HUE_PURPLE, SAT_HIG, VAL_LOW},
    [CLAMP_UNI_10] = {HUE_GREEN, SAT_HIG, VAL_BASE},
    [CLAMP_UNI_5]  = {HUE_GREEN, SAT_HIG, VAL_LOW},
};

static void set(UxState* s, int16_t led, UiColor c) { led_set_hsv(s, led, c.h, c.s, c.v); }

// A sparkle level as a framebuffer value. Through the panel's own curve on the
// way: the level is a perceived brightness, and a swell drawn linearly in duty
// spends its whole first half looking like nothing and then arrives.
static uint16_t sparkle_duty(float level, SparkleKind kind)
{
  uint16_t peak = kind == SPARKLE_TARGET ? TARGET_SPARKLE_V : MARK_SPARKLE_V;
  return (uint16_t) (powf(level, LED_GAMMA) * (float) (peak * LED_UNIT));
}

// How much of the element's own colour to keep at that level. Brightness alone
// took a lit element all the way to white at the top of every sparkle, which is
// a lot of contrast to leave running and costs the element the colour that says
// what it is - so the marker washes toward MARK_SPARKLE_KEEP instead, and the
// saturation moves with the light rather than being spent in one go.
static uint8_t sparkle_keep(float level)
{
  float keep = 255.0f - level * (255.0f - (float) MARK_SPARKLE_KEEP);
  return (uint8_t) (keep < 0.0f ? 0.0f : keep);
}

// One element's share of the marker, laid over whatever it already shows.
static void sparkle_over(UxState* s, int16_t led, SparkleKind kind)
{
  float level = ui_sparkle_level(s->hw_state->time, led, kind);
  if (level > 0.0f)
    led_wash(s, led, sparkle_duty(level, kind), sparkle_keep(level));
}

// One setting value, at the one brightness every base layer uses.
static void set_state(UxState* s, int16_t led, uint8_t hue) { led_set_hsv(s, led, hue, SAT_HIG, VAL_BASE); }

/* ---- layer 1: context ------------------------------------------------- */

void ui_render_context(UxState* state, int16_t led, TargetKind kind, int8_t id)
{
  // Nothing in this mode can be picked, so the base layer is the whole story.
  if (ui_mode(state->ui->shift_state)->action == ACT_NONE)
    return;

  if (ui_sel_is_src(state->ui, kind, id))
  {
    set(state, led, UI_COL_SOURCE);
    return;
  }

  if (ui_sel_pending(state->ui))
  {
    // Something is held and is looking for somewhere to go. Only the places it
    // can go light, and they light white; everything else goes dark, because an
    // element still showing its own state reads as pressable when it is not.
    //
    // Cleared first and then lit, rather than laid over: here the white is the
    // element's whole meaning and there is nothing underneath worth keeping.
    set(state, led, UI_COL_DARK);
    if (ui_sel_is_candidate(state, kind, id))
      sparkle_over(state, led, SPARKLE_TARGET);
    return;
  }

  // Nothing held yet: the element keeps showing its own state, and only marks
  // itself as pickable - white light laid over the top rather than a colour
  // that replaces what is underneath. Sampled from a field across the panel
  // instead of a shared gate, so the marker travels rather than arriving
  // everywhere at once.
  if (!ui_sel_is_candidate(state, kind, id))
    return;

  sparkle_over(state, led, SPARKLE_MARK);
}

/* ---- layer 3: confirmation -------------------------------------------- */

void ui_render_feedback(UxState* state, int16_t led, TargetKind kind, int8_t id)
{
  FeedbackKind fk;
  if (!ui_feedback_active(state->ui, kind, id, &fk))
    return;

  UiColor c = ui_feedback_color(fk);
  if (fk == FB_ERROR)
    c.v = state->ui->blink_slow ? c.v : 0;
  set(state, led, c);
}

/* ---- layer 2: transient value display ---------------------------------- */

// Where in its cycle something is, for the purpose of pulsing a ring.
//
// The real phase while the rate is slow enough to be drawn, so the row shows
// the actual polyrhythm and each ring is in step with what that channel is
// doing. Past RING_PULSE_MAX_HZ the panel cannot resolve it: the redraw would
// undersample the phase and alias it into a slow pulse, drawing the fastest
// channel as one of the slowest. Those free-run at the ceiling instead, all
// together, which reads as "off the top of the scale" rather than as a lie.
//
// The phase is wrapped here rather than trusted, because the stepped-random
// page hands in a phase multiplied by its step count.
static float pulse_phase(const UxState* s, float rate_hz, float phase)
{
  if (rate_hz > 0.0f && rate_hz <= (float) RING_PULSE_MAX_HZ)
    return phase - floorf(phase);

  // Integer modulo first: RING_PULSE_MAX_HZ divides 1000000 exactly, so this
  // wraps on a whole cycle and never steps at the microsecond counter's own
  // wrap.
  uint32_t period = 1000000u / RING_PULSE_MAX_HZ;
  return (float) (s->hw_state->time % period) / (float) period;
}

// A division's colour, pulsed at the rate that division produces.
//
// Brightness is the third fact after hue and saturation, and it is the only
// thing on either ring that says whether a division is fast or slow. Not on a
// blink timer: the row used to be multiplied by the fast blink, and eight rings
// flashing in unison say nothing about any of them.
//
// The peak also runs slightly warm - down the wheel as it brightens, which is
// what a filament does and reads as more contrast than brightness alone gives.
// Subtracted rather than added so the peak is the warm end, and bounded by
// RING_PULSE_HUE_SWING so no division class can pulse into another one's
// colour. Every HUE_FREQ_* is well clear of zero, so this cannot wrap.
static UiColor ring_pulse(const UxState* s, UiColor c, float rate_hz, float phase)
{
  float pulse = 0.5f * (1.0f + cosf(2.0f * 3.14159265f * pulse_phase(s, rate_hz, phase)));
  c.v         = (uint8_t) (RING_PULSE_V_MIN + (RING_PULSE_V_MAX - RING_PULSE_V_MIN) * pulse);
  c.h         = (uint8_t) (c.h - (uint8_t) (RING_PULSE_HUE_SWING * pulse));
  return c;
}

// Only no-mode has anything transient to show: a shift mode's channel LED shows
// that mode's setting permanently, so there is nothing to reveal on touch. The
// selected parameter is the exception - it is a number, not a mode, and the LED
// is needed for the output level the rest of the time.
//
// All eight at once, on one timer: seeing where a parameter sits is a
// comparison, and comparing needs the row lit together.
static void render_channel_param_edit(UxState* s, const ChannelSetup* ch)
{
  if (ui_mode(s->ui->shift_state)->channel_enc_target != ENC_PARAM)
    return;
  if (s->ui->param_display_hold == 0)
    return;

  int16_t value = s->engine_config->channel_state[ch->id].params[s->engine_state->active_scene][s->engine_config->selected_param];

  // Frequency is a ratio, not a level, so it reads as a coded colour rather
  // than a bipolar bar: hue for which kind of division, saturation for how far
  // off the grid it sits, brightness pulsing at the rate it produces.
  if (s->engine_config->selected_param == CH_PARAM_FRQ)
  {
    const ChannelEffective* eff = &s->engine_state->channels_effective[ch->id];
    set(s, ch->led, ring_pulse(s, ui_channel_freq_color(value), eff->freq_hz, eff->phase));
  }
  else
    led_set_adcr(s, ch->led, value);
}

/* ---- layer 1b: what letting go would do -------------------------------- */

// The colour of the act a release would perform. Taken from the confirmation
// palette rather than from the mode's tint, so "about to clear" and "cleared"
// are the same colour - which on the clear page is also what the page is
// tinted, because that page is nothing but that one act.
static uint8_t held_action_hue(const UiModeDesc* m) { return ui_feedback_color(m->action == ACT_CLEAR ? FB_CLEAR : FB_WRITE).h; }

// A press that acts on release used to show nothing until it had already
// happened. While one is held, the element wears the colour of what the release
// would do and dips out once - off, then back on - as each threshold is
// crossed. Nothing commits here: the handlers still act on the release, this
// only says what is about to happen.
//
// A dip rather than a blink, and the same brightness as the page underneath it
// at the first stage, so the only thing being said is "that registered". A
// press with a second, wider stage says so by getting brighter, and dips again
// as it gets there.
static void render_held_action(UxState* s, int16_t led, int8_t button, uint8_t hue, uint8_t stages)
{
  if (stages == 0 || !btn_down(&s->ui->in, button))
    return;

  uint32_t held = btn_held(&s->ui->in, button);
  if (held < UI_T_DEBOUNCE)
    return;

  uint32_t stage_start = UI_T_DEBOUNCE;
  uint8_t value        = VAL_BASE;

  if (stages > 1 && held >= UI_T_LONG)
  {
    stage_start = UI_T_LONG;
    value       = VAL_HIG;
  }

  if (held - stage_start < UI_HELD_DIP)
    value = VAL_OFF;

  led_set_hsv(s, led, hue, SAT_HIG, value);
}

// The same, for the parameter clear in no-mode. Not routed through the
// selection model - that press is not a selection - and gated on the same
// condition the handler applies, so a button being held as the fine-adjust
// modifier does not advertise a clear it will not perform.
static void render_held_param_clear(UxState* s, const ChannelSetup* ch)
{
  if (ui_mode(s->ui->shift_state)->channel_btn_action != CHB_RESET_PARAM)
    return;

  uint32_t since_edit = s->hw_state->time - s->engine_state->channels_last_delta[ch->id];
  if (btn_held(&s->ui->in, ch->button) >= since_edit)
    return;

  render_held_action(s, ch->led, ch->button, ui_feedback_color(FB_CLEAR).h, 2);
}

/* ---- per element ------------------------------------------------------- */

// Layer 0 for a channel. Which of these runs is the mode descriptor's call.
//
// Output level appears in exactly one arm, and only no-mode selects it: in a
// shift mode the encoder ring is showing what that mode edits, and nothing
// else. Two facts on one LED - "what this channel is putting out" underneath
// "what this page does to it" - is what made the shift pages hard to read.
static void render_channel_base(UxState* s, const ChannelSetup* ch, const UiModeDesc* m)
{
  const ChannelConfig* cfg = &s->engine_config->channel_state[ch->id];

  switch (m->channel_base)
  {
  case CHBASE_OUTPUT:
    if (s->ui->muted[ch->id])
      set(s, ch->led, UI_COL_MUTED);
    else
      led_set_dac(s, ch->led, s->engine_state->channels_output_level[ch->id]);
    break;

  case CHBASE_SHAPE:
    set(s, ch->led, shape_mode_color[iclamp(cfg->shape_mode, 0, SHAPE_MODE_COUNT - 1)]);
    break;

  case CHBASE_QUANTIZE:
    set_state(s, ch->led, quantize_mode_color[cfg->quantize_mode]);
    break;

  case CHBASE_AMPMODE:
    set_state(s, ch->led, input_amp_mode_color[cfg->input_amp_mode]);
    break;

  case CHBASE_MUTE:
    // Both states are lit here: on the page whose subject is mute, "passing"
    // is as much a state as "gated" and a dark ring would read as neither.
    set(s, ch->led, s->ui->muted[ch->id] ? UI_COL_MUTED : UI_COL_UNMUTED);
    break;

  case CHBASE_SR_LENGTH:
  {
    // Dark where it would do nothing: pattern length is a stepped-mode setting,
    // and the encoder is inert on the other channels for the same reason.
    if (!shape_mode_is_stepped(cfg->shape_mode))
    {
      set(s, ch->led, UI_COL_DARK);
      break;
    }

    // A pattern length is a division of the beat in the same sense FRQ's ratio
    // is, so it is read the same way: hue for which kind of division, and a
    // pulse at the rate it produces.
    //
    // It used to be a walk around the colour wheel, one step per index. That
    // said which of twelve positions the knob was on and nothing about what any
    // of them did - 12 and 16 sat next to each other in two unrelated colours,
    // where one subdivides in threes and the other does not. The prime limit is
    // the fact worth colouring, and it is the fact the FRQ page already
    // colours.
    // SAT_MAX, not the SAT_HIG a base layer usually wears: this is the FRQ
    // page's scale, and the same division has to be the same colour on both.
    // SAT_HIG leaves a tenth of the value on the primaries the hue does not
    // use, which on a warm hue is a blue floor - enough to pull orange and
    // yellow together until they were hard to tell apart.
    int length  = sr_length_for_index(cfg->sr_length_idx);
    UiColor col = {ui_division_hue((uint32_t) length), SAT_MAX, VAL_BASE};

    // The steps run at the channel's own rate times the number of them, which
    // is what the setting is for: the same LFO subdivided finer. Phase likewise
    // - the position within a step, not within the cycle - so a long pattern
    // pulses fast and a short one slowly.
    const ChannelEffective* eff = &s->engine_state->channels_effective[ch->id];
    set(s, ch->led, ring_pulse(s, col, eff->freq_hz * (float) length, eff->phase * (float) length));
    break;
  }

  case CHBASE_CLAMP:
    set(s, ch->led, clamp_mode_color[iclamp(cfg->clamp_mode, 0, CLAMP_MODE_COUNT - 1)]);
    break;

  case CHBASE_TINT:
    set_state(s, ch->led, m->tint_hue);
    break;

  case CHBASE_OFF:
  default:
    set(s, ch->led, UI_COL_DARK);
    break;
  }
}

static void render_channel(UxState* s, const ChannelSetup* ch)
{
  const UiModeDesc* m = ui_mode(s->ui->shift_state);

  render_channel_base(s, ch, m);
  ui_render_context(s, ch->led, TGT_CHANNEL, ch->id);
  render_held_action(s, ch->led, ch->button, held_action_hue(m), ui_sel_press_stages(s, TGT_CHANNEL, ch->id));
  render_held_param_clear(s, ch);
  render_channel_param_edit(s, ch);
  ui_render_feedback(s, ch->led, TGT_CHANNEL, ch->id);
}

// Layer 0 for a scene button. Which of these runs is the mode descriptor's
// call, not a shift_state comparison - adding a mode adds a row to ui_mode.c,
// not an arm here.
static void render_scene_base(UxState* s, const SceneSetup* scene, const UiModeDesc* m)
{
  switch (m->scene_btn_base)
  {
  case SCB_INPUT_LEVEL:
    // input_state[] is the jack reading already scaled into DAC units by the
    // input layer, so this needs no driver call and stays assertable in a test.
    led_set_dac(s, scene->led, s->hw_state->input_state[scene->id]);
    break;

  case SCB_INPUT_MODE:
    set_state(s, scene->led, input_mode_color[s->engine_config->input_mode[scene->id]]);
    break;

  case SCB_MODE_TINT:
    set_state(s, scene->led, m->tint_hue);
    break;

  case SCB_PRESET:
  {
    // Held shows which slot is armed; past UI_T_VLONG it turns red to say the
    // release will store rather than load.
    int8_t held = btn_down(&s->ui->in, scene->button);
    int8_t save = btn_holding(&s->ui->in, scene->button, UI_T_VLONG);
    led_set_hsv(s, scene->led, save ? HUE_RED : HUE_GREEN, SAT_MAX, held ? VAL_HIG : VAL_BASE);
    break;
  }

  case SCB_CROSSFADE:
  default:
  {
    // Scene crossfade weight, so the blend stays readable in every mode.
    uint8_t val = s->engine_state->scenes_contribution[scene->id] / 8;
    if (s->engine_state->active_scene == scene->id)
      val = imax(val, VAL_LOW);
    led_set_hsv(s, scene->led, 0, SAT_OFF, val);

    // In STA/STB the scene currently wired to that end of the crossfader is
    // the "source" of what the fader does.
    uint8_t wired = (m->xfade_end == XFADE_A && s->engine_config->scene_a == scene->id) ||
                    (m->xfade_end == XFADE_B && s->engine_config->scene_b == scene->id);
    if (wired)
      set(s, scene->led, UI_COL_SOURCE);
    break;
  }
  }
}

static void render_scene(UxState* s, const SceneSetup* scene)
{
  const UiModeDesc* m = ui_mode(s->ui->shift_state);

  // Modes where the scene buttons address inputs only have N_INPUTS of them;
  // the rest go dark rather than showing a stale base layer. Same guard the
  // handler uses, in ui_scene_button.
  if (m->scene_btn_kind == TGT_INPUT && scene->id >= N_INPUTS)
  {
    led_set_hsv(s, scene->led, 0, SAT_OFF, VAL_OFF);
    return;
  }

  render_scene_base(s, scene, m);
  ui_render_context(s, scene->led, m->scene_btn_kind, scene->id);
  render_held_action(s, scene->led, scene->button, held_action_hue(m), ui_sel_press_stages(s, m->scene_btn_kind, scene->id));
  ui_render_feedback(s, scene->led, m->scene_btn_kind, scene->id);
}

static void render_ctrl_button(UxState* s, const CtrlButtonSetup* btn)
{
  if (btn->led < 0)
    return;

  if (s->ui->shift_state == btn->id)
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, s->ui->blink_slow ? VAL_BASE : 0);
  else if (s->ui->shift_state != SHIFT_STATE_NONE)
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, VAL_OFF); // a mode is running; its button is the only lit one
  else if (s->engine_config->selected_param == btn->id)
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, VAL_HIG);
  else
    // Barely on, but on: it says which colour belongs to which parameter, so
    // the row can be read without pressing anything. Far enough below the
    // selected one that which is which is obvious across the room.
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, VAL_DIM);
}

static void render_quantizer(UxState* s)
{
  if (!ui_mode(s->ui->shift_state)->keyboard_overlay)
    return;
  // While a trigger source is being picked the keyboard is not a keyboard,
  // so the scene/ctrl renderers own those LEDs for the duration.
  if (ui_sel_pending(s->ui))
    return;

  for (uint16_t st = 0; st < N_SEMITONES; st++)
  {
    uint8_t val = (s->engine_config->quantize_mask & (1u << st)) ? VAL_BASE : VAL_OFF;
    led_set_hsv(s, s->ux_setup->quantizer_semitones[st].led, 0, SAT_OFF, val);
  }
}

// Error codes are a bit per scene button. Drawn as a final overlay rather
// than blitted from the hardware poll with an early return, which used to
// suppress the whole UX pass for as long as an error was showing.
static void render_error(UxState* state)
{
  if (!error_any(state->engine_state))
    return;

  led_clear_all(state);
  UiColor c = UI_COL_ERROR;

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    uint8_t on = error_get(state->engine_state, s) && state->ui->blink_slow;
    led_set_hsv(state, state->ux_setup->scenes[s].led, c.h, c.s, on ? c.v : VAL_OFF);
  }
}

void ui_render(UxState* state)
{
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    render_channel(state, &state->ux_setup->channels[c]);
  }

  for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
  {
    render_ctrl_button(state, &state->ux_setup->ctrl_buttons[b]);
  }

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    render_scene(state, &state->ux_setup->scenes[s]);
  }

  render_quantizer(state);
  render_error(state);
}
