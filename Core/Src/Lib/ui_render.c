#include "ui_render.h"
#include "color_presets.h"
#include "config.h"
#include "error.h"
#include "helpers.h"
#include "hw_setup.h"
#include "led_fb.h"
#include "stepped_random_table.h" // SR_LENGTH_COUNT
#include "ui_channel.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"
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
static const uint8_t shape_mode_color[SHAPE_MODE_COUNT] = {HUE_STATE_DEFAULT, HUE_STATE_LEVEL, HUE_STATE_EVENT};
static const uint8_t input_mode_color[INPUT_MODE_COUNT] = {HUE_STATE_DEFAULT, HUE_STATE_EVENT, HUE_STATE_RESET, HUE_STATE_LEVEL};

// The output clamp is two facts - polarity and range - so it uses two axes:
// purple for bipolar and green for unipolar, dim for the half range. The one
// place brightness carries meaning in a base layer.
static const UiColor clamp_mode_color[CLAMP_MODE_COUNT] = {
    [CLAMP_BI_10]  = {HUE_PURPLE, SAT_HIG, VAL_MED},
    [CLAMP_BI_5]   = {HUE_PURPLE, SAT_HIG, VAL_LOW},
    [CLAMP_UNI_10] = {HUE_GREEN, SAT_HIG, VAL_MED},
    [CLAMP_UNI_5]  = {HUE_GREEN, SAT_HIG, VAL_LOW},
};

// Pattern length is a ramp of twelve values, not a handful of named modes, so
// it reads as a position on the colour wheel: one turn across the whole set,
// starting from the purple that every other setting's first value wears.
static uint8_t sr_length_hue(int8_t idx) { return (uint8_t) (HUE_PURPLE + iclamp(idx, 0, SR_LENGTH_COUNT - 1) * (256 / SR_LENGTH_COUNT)); }

static void set(UxState* s, int16_t led, UiColor c) { led_set_hsv(s, led, c.h, c.s, c.v); }

// One setting value, at the one brightness every base layer uses.
static void set_state(UxState* s, int16_t led, uint8_t hue) { led_set_hsv(s, led, hue, SAT_HIG, VAL_LOW); }

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
    if (!ui_sel_is_candidate(state, kind, id))
      set(state, led, UI_COL_DARK);
    else
      set(state, led, state->ui->blink_mark ? UI_COL_DARK : UI_COL_TARGET);
    return;
  }

  // Nothing held yet: the element keeps showing its own state, and only marks
  // itself as pickable - a brief white flash over the top rather than a colour
  // that replaces what is underneath.
  if (state->ui->blink_mark && ui_sel_is_candidate(state, kind, id))
    set(state, led, UI_COL_MARK);
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

// Where in its own cycle a channel is, for the purpose of pulsing a ring.
//
// The oscillator's real phase while that is slow enough to be drawn, so the row
// shows the actual polyrhythm and each ring is in step with what that channel is
// putting out. Past FREQ_PULSE_MAX_HZ the panel cannot resolve it: the redraw
// would undersample the phase and alias it into a slow pulse, drawing the
// fastest channel as one of the slowest. Those free-run at the ceiling instead,
// all together, which reads as "off the top of the scale" rather than as a lie.
static float freq_pulse_phase(const UxState* s, const ChannelSetup* ch)
{
  const ChannelEffective* eff = &s->engine_state->channels_effective[ch->id];

  if (eff->freq_hz > 0.0f && eff->freq_hz <= (float) FREQ_PULSE_MAX_HZ)
    return eff->phase;

  // Integer modulo first: FREQ_PULSE_MAX_HZ divides 1000000 exactly, so this
  // wraps on a whole cycle and never steps at the microsecond counter's own
  // wrap.
  uint32_t period = 1000000u / FREQ_PULSE_MAX_HZ;
  return (float) (s->hw_state->time % period) / (float) period;
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
  // off the grid it sits.
  //
  // Brightness is the third fact - a shallow pulse at the channel's own output
  // rate, which is the only thing on the ring that says whether a ratio is fast
  // or slow. Not on a blink timer: the row used to be multiplied by the fast
  // blink, and eight rings flashing in unison say nothing about any of them.
  if (s->engine_config->selected_param == CH_PARAM_FRQ)
  {
    UiColor c   = ui_channel_freq_color(value);
    float pulse = 0.5f * (1.0f + cosf(2.0f * 3.14159265f * freq_pulse_phase(s, ch)));
    c.v         = (uint8_t) (FREQ_PULSE_V_MIN + (FREQ_PULSE_V_MAX - FREQ_PULSE_V_MIN) * pulse);

    // Down the wheel toward the warm end as it brightens. Subtracted rather
    // than added so the peak runs warm, and bounded by FREQ_PULSE_HUE_SWING so
    // no class can pulse into another one's colour. Every FREQ_* hue is well
    // clear of zero, so this cannot wrap.
    c.h = (uint8_t) (c.h - (uint8_t) (FREQ_PULSE_HUE_SWING * pulse));
    set(s, ch->led, c);
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
  uint8_t value        = VAL_LOW;

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
    set_state(s, ch->led, shape_mode_color[cfg->shape_mode]);
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
    // Dark where it would do nothing: pattern length is a stepped-mode setting,
    // and the encoder is inert on the other channels for the same reason.
    if (shape_mode_is_stepped(cfg->shape_mode))
      set_state(s, ch->led, sr_length_hue(cfg->sr_length_idx));
    else
      set(s, ch->led, UI_COL_DARK);
    break;

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
    led_set_hsv(s, scene->led, save ? HUE_RED : HUE_GREEN, SAT_MAX, held ? VAL_MED : VAL_LOW);
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
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, s->ui->blink_slow ? VAL_MED : 0);
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
    uint8_t val = (s->engine_config->quantize_mask & (1u << st)) ? VAL_LOW : VAL_OFF;
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
