#include "ui_channel.h"
#include "channel.h"
#include "color_presets.h"
#include "config.h"
#include "helpers.h"
#include "hw_setup.h"
#include "stepped_random_table.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"
#include "ui_state.h"
#include "ux_state.h"
#include <stdint.h>

#define N_FREQ_MULTIPLIERS 31

// The frequency parameter steps through ratios of the beat rather than sweeping
// continuously, so the encoder snaps to this table. Values are in the same
// units as ChannelConfig.params.
static const int16_t quantized_multipliers[N_FREQ_MULTIPLIERS] = {
    -127 * 255, // 1/128
    -63 * 255,  // 1/64
    -31 * 255,  // 1/32
    -23 * 255,  // 1/24
    -15 * 255,  // 1/16
    -11 * 255,  // 1/12
    -7 * 255,   // 1/8
    -5 * 255,   // 1/6
    -4 * 255,   // 1/5
    -3 * 255,   // 1/4
    -2 * 255,   // 1/3
    -1 * 255,   // 1/2
    -127,       // 2/3
    -85,        // 3/4
    -64,        // 4/5
    0,          // 1
    64,         // 5/4
    85,         // 4/3
    127,        // 3/2
    1 * 255,    // 2
    2 * 255,    // 3
    3 * 255,    // 4
    4 * 255,    // 5
    5 * 255,    // 6
    7 * 255,    // 8
    11 * 255,   // 12
    15 * 255,   // 16
    23 * 255,   // 24
    31 * 255,   // 32
    63 * 255,   // 64
    127 * 255,  // 128
};

// Same order and length as quantized_multipliers, read with the same index:
// a frequency ratio is shown as a coded hue rather than a level, because it is
// a ratio and not a magnitude.
// TODO: Even dividers: green
static const uint8_t quantized_multipliers_colors[N_FREQ_MULTIPLIERS] = {
    HUE_GREEN,  // 1/128
    HUE_CYAN,   // 1/64
    HUE_GREEN,  // 1/32
    HUE_RED,    // 1/24
    HUE_GREEN,  // 1/16
    HUE_RED,    // 1/12
    HUE_GREEN,  // 1/8
    HUE_RED,    // 1/6
    HUE_YELLOW, // 1/5
    HUE_GREEN,  // 1/4
    HUE_RED,    // 1/3
    HUE_GREEN,  // 1/2
    HUE_CYAN,   // 2/3
    HUE_CYAN,   // 3/4
    HUE_CYAN,   // 4/5
    HUE_GREEN,  // 1
    HUE_CYAN,   // 5/4
    HUE_CYAN,   // 4/3
    HUE_CYAN,   // 3/2
    HUE_GREEN,  // 2
    HUE_RED,    // 3
    HUE_GREEN,  // 4
    HUE_YELLOW, // 5
    HUE_RED,    // 6
    HUE_GREEN,  // 8
    HUE_RED,    // 12
    HUE_GREEN,  // 16
    HUE_RED,    // 24
    HUE_CYAN,   // 32
    HUE_GREEN,  // 64
    HUE_CYAN    // 128
};

_Static_assert(sizeof quantized_multipliers_colors / sizeof quantized_multipliers_colors[0] == N_FREQ_MULTIPLIERS,
               "one colour per multiplier, read with the same index");

static void ui_channel_param(const ChannelSetup* ch, UxState* state)
{
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  int8_t param         = state->ui->selected_param;
  int16_t delta        = enc_delta(&state->ui->in, ch->encoder);
  int8_t alt           = btn_down(&state->ui->in, ch->button);
  int8_t scene         = state->engine_state->active_scene;
  if (delta == 0)
    return;

  ux_note_channel_edit(state, ch->id);
  ui_show_channel_edit(state->ui, ch->id);

  // By value: ChannelConfig is a packed FRAM record, so a pointer into it is
  // potentially unaligned.
  int16_t value = chcfg->params[scene][param];

  if (alt)
  {
    // Holding the channel button is the fine-adjust modifier.
    value += 32 * delta;
  }
  else if (param == CH_PARAM_FRQ)
  {
    size_t idx                           = 0;
    value                                = val_neighbour(value, delta, quantized_multipliers, N_FREQ_MULTIPLIERS, &idx);
    state->ui->channels_edit_hue[ch->id] = quantized_multipliers_colors[idx];
  }
  else if (param == CH_PARAM_MOD && shape_mode_is_stepped(chcfg->shape_mode))
  {
    // In the stepped modes MOD picks a pattern length from a discrete set, so
    // step straight to the next one. Treating it as a continuous parameter
    // meant ~22 detents of dead travel between divisions.
    size_t idx = 0;
    value      = val_neighbour(value, delta, sr_length_param, SR_LENGTH_COUNT, &idx);
  }
  else
  {
    value += delta * 256;
  }

  chcfg->params[scene][param] = value;
}

// Three of the four encoder targets cycle a discrete per-channel setting and
// arm the value display. By value rather than through a pointer, because the
// three fields have three different types - two of them enums, which are not
// int8_t and must not be aliased as one.
static int cycle_mode(UxState* state, uint8_t ch_id, int current, int16_t delta, int count)
{
  ui_show_channel_edit(state->ui, ch_id);
  return delta_modulo_step(current, delta, count);
}

void ui_channel_update(const ChannelSetup* ch, UxState* state)
{
  const UiModeDesc* m  = ui_mode(state->ui->shift_state);
  int8_t long_pressed  = btn_released_after(&state->ui->in, ch->button, UI_T_LONG);
  int8_t pressed       = btn_ev(&state->ui->in, ch->button, BTN_EV_UP);
  int8_t pressing      = btn_down(&state->ui->in, ch->button);
  int16_t delta        = enc_delta(&state->ui->in, ch->encoder);
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];

  switch (m->channel_btn_action)
  {
  case CHB_SELECT:
    if (pressed)
      ui_sel_press(state, m->channel_btn_kind, ch->id, long_pressed);
    break;

  case CHB_MUTE_TOGGLE:
    // On release rather than press: easier to perform accurately, and it
    // matches every other momentary action in the UI.
    if (pressed)
      state->ui->muted[ch->id] = !state->ui->muted[ch->id];
    break;

  case CHB_RESET_PARAM:
  {
    // Only a long press that spanned no encoder movement resets the param -
    // otherwise holding the button as a fine-adjust modifier would wipe the
    // value the user was just adjusting.
    uint32_t since_edit = state->hw_state->time - state->engine_state->channels_last_delta[ch->id];
    if (long_pressed && btn_held(&state->ui->in, ch->button) < since_edit)
    {
      channel_reset_param(ch->id, state->engine_config, state->engine_state->active_scene, state->ui->selected_param);
      return;
    }
    break;
  }

  default:
    break;
  }

  if (delta == 0)
    return;

  switch (m->channel_enc_target)
  {
  case ENC_PARAM:
    ui_channel_param(ch, state);
    break;

  case ENC_SHAPE:
    chcfg->shape_mode = (int8_t) cycle_mode(state, ch->id, chcfg->shape_mode, delta, SHAPE_MODE_COUNT);
    break;

  case ENC_QUANT:
    // While a trigger source is being picked the encoder would fight the
    // assignment for the same value.
    if (ui_sel_pending(state->ui))
      break;
    chcfg->quantize_mode = (ChannelQuantizeMode) cycle_mode(state, ch->id, chcfg->quantize_mode, delta, QUANTIZE_MODE_COUNT);
    break;

  case ENC_AMPMODE:
    if (pressing)
      break; // the button is picking a source, not modifying the encoder
    chcfg->input_amp_mode = (ChannelInputAmpMode) cycle_mode(state, ch->id, chcfg->input_amp_mode, delta, INPUT_AMP_MODE_COUNT);
    break;

  default:
    break;
  }
}
