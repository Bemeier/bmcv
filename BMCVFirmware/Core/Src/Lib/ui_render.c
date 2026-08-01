#include "ui_render.h"
#include "color_presets.h"
#include "dac_adc.h"
#include "error.h"
#include "helpers.h"
#include "hw_setup.h"
#include "led_fb.h"
#include "state.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"
#include "ui_state.h"
#include "ux_state.h"
#include <stdint.h>

// Value colours for the transient display. Moved here from channel.c: they
// are presentation, and one renderer now shows all of them.
static const uint8_t quantize_mode_color[QUANTIZE_MODE_COUNT]   = {HUE_RED, HUE_MAGENTA, HUE_CYAN};
static const uint8_t input_amp_mode_color[INPUT_AMP_MODE_COUNT] = {HUE_RED, HUE_GREEN, HUE_YELLOW};
static const uint8_t shape_mode_color[SHAPE_MODE_COUNT]         = {HUE_GREEN, HUE_MAGENTA, HUE_BLUE, HUE_CYAN};
static const uint8_t input_mode_color[INPUT_MODE_COUNT]         = {HUE_GREEN, HUE_RED, HUE_CYAN, HUE_MAGENTA};

static void set(UxState* s, int16_t led, UiColor c) { led_set_hsv(s, led, c.h, c.s, c.v); }

/* ---- layer 1: context ------------------------------------------------- */

void ui_render_context(UxState* state, int16_t led, TargetKind kind, int8_t id)
{
  if (ui_sel_is_src(state->ui, kind, id))
  {
    set(state, led, UI_COL_SOURCE);
    return;
  }

  // Pulses over the base layer rather than to black: in the gaps the element
  // still shows its own state, so a muted channel stays recognisably muted
  // and a live one still shows its level while you pick a target.
  if (state->ui->blink_fast && ui_sel_is_candidate(state, kind, id))
  {
    set(state, led, UI_COL_CANDIDATE);
  }
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

void ui_render_arm_all_edits(UxState* state)
{
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    state->ui->channels_edit_hold[c] = UI_EDIT_DISPLAY;
  }
}

static void render_channel_edit(UxState* s, const ChannelSetup* ch)
{
  if (s->ui->channels_edit_hold[ch->id] == 0)
    return;

  const ChannelConfig* cfg = &s->engine_config->channel_state[ch->id];

  switch (ui_mode(s->ui->shift_state)->channel_enc_target)
  {
  case ENC_SHAPE:
    led_set_hsv(s, ch->led, shape_mode_color[cfg->shape_mode], SAT_HIG, VAL_LOW);
    break;
  case ENC_QUANT:
    led_set_hsv(s, ch->led, quantize_mode_color[cfg->quantize_mode], SAT_HIG, VAL_LOW);
    break;
  case ENC_AMPMODE:
    led_set_hsv(s, ch->led, input_amp_mode_color[cfg->input_amp_mode], SAT_HIG, VAL_LOW);
    break;
  case ENC_PARAM:
    // Frequency is a ratio, not a level, so it reads as a coded hue rather
    // than a bipolar bar.
    if (s->ui->selected_param == CH_PARAM_FRQ)
      led_set_hsv(s, ch->led, s->ui->channels_edit_hue[ch->id], SAT_MAX, s->ui->blink_fast * VAL_MED);
    else
      led_set_adcr(s, ch->led, cfg->params[s->engine_state->active_scene][s->ui->selected_param]);
    break;
  default:
    break;
  }
}

/* ---- per element ------------------------------------------------------- */

static void render_channel(UxState* s, const ChannelSetup* ch)
{
  // 0: base. Always writes.
  if (s->ui->muted[ch->id])
    set(s, ch->led, UI_COL_MUTED);
  else
    led_set_dac(s, ch->led, s->engine_state->channels_output_level[ch->id]);

  ui_render_context(s, ch->led, TGT_CHANNEL, ch->id);
  render_channel_edit(s, ch);
  ui_render_feedback(s, ch->led, TGT_CHANNEL, ch->id);
}

static void render_scene(UxState* s, const SceneSetup* scene)
{
  const UiModeDesc* m = ui_mode(s->ui->shift_state);

  // 0: base - what this button stands for in this mode.
  if (m->scene_btn_kind == TGT_INPUT)
  {
    if (scene->id >= N_INPUTS)
      led_set_hsv(s, scene->led, 0, SAT_OFF, VAL_OFF);
    else if (s->ui->shift_state == SHIFT_STATE_SYS)
      led_set_hsv(s, scene->led, input_mode_color[s->engine_config->input_mode[scene->id]], SAT_HIG, VAL_LOW);
    else
      led_set_adcr(s, scene->led, get_adc(s->hw_setup->input_adc_idx[scene->id]));
  }
  else if (s->ui->shift_state == SHIFT_STATE_SAV)
  {
    // Held shows which slot is armed; past UI_T_VLONG it turns red to say the
    // release will store rather than load.
    int8_t held = btn_down(&s->ui->in, scene->button);
    int8_t save = btn_holding(&s->ui->in, scene->button, UI_T_VLONG);
    led_set_hsv(s, scene->led, save ? HUE_RED : HUE_GREEN, SAT_MAX, held ? VAL_MED : VAL_LOW);
  }
  else
  {
    // Scene crossfade weight, so the blend stays readable in every mode.
    uint8_t val = s->engine_state->scenes_contribution[scene->id] / 8;
    if (s->engine_state->active_scene == scene->id)
      val = imax(val, VAL_LOW);
    led_set_hsv(s, scene->led, 0, SAT_OFF, val);

    // In STA/STB the pair currently wired to the crossfader is the "source".
    if ((s->ui->shift_state == SHIFT_STATE_STA && s->engine_config->scene_a == scene->id) ||
        (s->ui->shift_state == SHIFT_STATE_STB && s->engine_config->scene_b == scene->id))
      set(s, scene->led, UI_COL_SOURCE);
  }

  ui_render_context(s, scene->led, (TargetKind) m->scene_btn_kind, scene->id);
  ui_render_feedback(s, scene->led, (TargetKind) m->scene_btn_kind, scene->id);
}

static void render_ctrl_button(UxState* s, const CtrlButtonSetup* btn)
{
  if (btn->led < 0)
    return;

  if (s->ui->shift_state == btn->id)
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, s->ui->blink_slow ? VAL_MED : 0);
  else if (s->ui->selected_param == btn->id && s->ui->shift_state == SHIFT_STATE_NONE)
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, VAL_MED);
  else
    led_set_hsv(s, btn->led, btn->color, SAT_MAX, VAL_OFF);
}

static void render_quantizer(UxState* s)
{
  if (s->ui->shift_state != SHIFT_STATE_QNT)
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
  if (!error_any())
    return;

  led_clear_all(state);
  UiColor c = UI_COL_ERROR;

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    uint8_t on = error_get(s) && state->ui->blink_slow;
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
