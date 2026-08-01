#include "channel.h"
#include "assign.h"
#include "clock_sync.h"
#include "color_presets.h"
#include "dac_adc.h"
#include "helpers.h"
#include "hw_setup.h"
#include "led_fb.h"
#include "math.h"
#include "state.h"
#include "stepped_random.h"
#include "stepped_random_table.h"
#include "ui_input.h"
#include "ux_state.h"
#include "wave_fn.h"
#include <stdint.h>

#define N_SHP_LEVELS 8
#define N_AMP_LEVELS 11
#define N_FREQ_MULTIPLIERS 31
#define N_FREQ_SCALE 255
// #define SHAPE_INTERVAL INT16_MAX / M

// Output-side (DAC-domain) trigger detection thresholds for this channel's own
// signal. Deliberately distinct from dac_adc.h's identically-valued
// TRIG_THRESH*, which are input-side (ADC-domain) - the two are independent
// and should be tunable separately.
#define CHANNEL_TRIG_THRESH 1024
#define CHANNEL_TRIG_THRESH_LOW 800

// How long after the last encoder movement a channel still counts as being
// actively edited, during which a pattern-length change applies immediately
// rather than waiting for the cycle wrap.
#define MOD_EDIT_WINDOW MS(500)

/*
static const int16_t quantized_amp_levels[N_AMP_LEVELS] = {
    -INT16_MAX,     // *-1
    -INT16_MAX / 2, // *-1/2
    -INT16_MAX / 3, // *-1/3
    -INT16_MAX / 4, // *-1/4
    -INT16_MAX / 8, // *-1/8
    0,              // 0
    INT16_MAX / 8,  // *1/4
    INT16_MAX / 4,  // *1/4
    INT16_MAX / 3,  // *1/3
    INT16_MAX / 2,  // *1/2
    INT16_MAX       // *1
};

static const int16_t quantized_shp_levels[N_SHP_LEVELS] = {
    -SHAPE_INTERVAL * 4, -SHAPE_INTERVAL * 3, -SHAPE_INTERVAL * 2, -SHAPE_INTERVAL * 1,
    -SHAPE_INTERVAL * 0, SHAPE_INTERVAL * 1,  SHAPE_INTERVAL * 2,  SHAPE_INTERVAL * 3,
};
*/

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
    63 * 255,   // 32
    127 * 255,  // 128
};

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
    HUE_CYAN,   // 5/4
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
    HUE_CYAN    // 64
};

// Per-channel mutable state (prev_out / trig_state / trig_flag / last_delta)
// lives in EngineState so it resets with the rest of the engine and does not
// leak between tests or engine instances.

static const uint8_t quantize_mode_color[QUANTIZE_MODE_COUNT]   = {HUE_RED, HUE_MAGENTA, HUE_CYAN};
static const uint8_t input_amp_mode_color[INPUT_AMP_MODE_COUNT] = {HUE_RED, HUE_GREEN, HUE_YELLOW};
static const uint8_t shape_mode_color[SHAPE_MODE_COUNT]         = {HUE_GREEN, HUE_MAGENTA, HUE_BLUE, HUE_CYAN};

static const float k_sync = 0.075f;

void update_channel_param(const ChannelSetup* ch, UxState* state)
{
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  int8_t param         = state->ui->selected_param;
  int16_t delta        = enc_delta(&state->ui->in, ch->encoder);
  int8_t alt           = btn_down(&state->ui->in, ch->button);
  if (delta == 0)
    return;

  ui_channel_note_edit(state, ch->id);

  state->ui->channels_edit_hold[ch->id] = UI_EDIT_DISPLAY;

  if (alt)
  {
    chcfg->params[state->engine_state->active_scene][param] += 32 * delta;
  }
  else if (param == CH_PARAM_FRQ)
  {
    size_t idx = 0;
    chcfg->params[state->engine_state->active_scene][param] =
        val_neighbour(chcfg->params[state->engine_state->active_scene][param], delta, quantized_multipliers, N_FREQ_MULTIPLIERS, &idx);
    state->ui->channels_edit_hue[ch->id] = quantized_multipliers_colors[idx];
  }
  else if (param == CH_PARAM_MOD && shape_mode_is_stepped(chcfg->shape_mode))
  {
    // In the stepped modes MOD picks a pattern length from a discrete set, so
    // step straight to the next one. Treating it as a continuous parameter
    // meant ~22 detents of dead travel between divisions.
    size_t idx = 0;
    chcfg->params[state->engine_state->active_scene][param] =
        val_neighbour(chcfg->params[state->engine_state->active_scene][param], delta, sr_length_param, SR_LENGTH_COUNT, &idx);
  }
  /*
  else if (param == CH_PARAM_SHP)
  {
      size_t idx = 0;
      chcfg->params[state->engine_state->active_scene][param] =
          val_neighbour(chcfg->params[state->engine_state->active_scene][param], delta, quantized_shp_levels, N_SHP_LEVELS, &idx);
  }
  */
  else
  {
    chcfg->params[state->engine_state->active_scene][param] += delta * 256;
  }
}

void init_channel(const ChannelSetup* ch, UxState* state)
{
  state->engine_state->channels_shared_phase[ch->id]     = 0;
  state->engine_state->channels_phase_correction[ch->id] = 0;
  state->engine_state->channels_prev_phase[ch->id]       = 0;
  state->engine_state->channels_length_idx[ch->id]       = -1; // latch on the first tick
}

void reset_channel_param(const ChannelSetup* ch, UxState* state, int8_t scene, int8_t param)
{
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  if (param == CH_PARAM_FRQ)
  {
    chcfg->params[scene][param] = -255;
  }
  else
  {
    chcfg->params[scene][param] = 0;
  }
}

void reset_channel(const ChannelSetup* ch, UxState* state, int8_t scene)
{
  init_channel(ch, state);

  if (scene < 0)
  {
    for (uint8_t s = 0; s < N_SCENES; s++)
    {
      for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
      {
        reset_channel_param(ch, state, s, p);
      }
    }
  }
  else
  {
    for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
    {
      reset_channel_param(ch, state, scene, p);
    }
  }
}

void reset_channel_phase(const ChannelSetup* ch, UxState* state)
{
  state->engine_state->channels_shared_phase[ch->id]     = 0;
  state->engine_state->channels_phase_correction[ch->id] = 0;
}

void update_channel(const ChannelSetup* ch, UxState* state)
{
  int8_t long_pressed  = btn_released_after(&state->ui->in, ch->button, UI_T_LONG);
  int8_t pressed       = btn_ev(&state->ui->in, ch->button, BTN_EV_UP);
  int8_t pressing      = btn_down(&state->ui->in, ch->button);
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  switch (state->ui->shift_state)
  {
  case SHIFT_STATE_SYS:
    chcfg->shape_mode = delta_modulo_step(chcfg->shape_mode, enc_delta(&state->ui->in, ch->encoder), SHAPE_MODE_COUNT);
    break;
  case SHIFT_STATE_QNT:
    if (pressed && assign_state(state) == ASSIGN_NONE)
    {
      chcfg->quantize_mode = QUANTIZE_TRIG_SRC;
      assign_event(ASSIGN_TRIG_SRC, ch->id, state);
    }
    else if (pressed && assign_state(state) == ASSIGN_TRIG_SRC)
    {
      assign_event(ASSIGN_CHANNEL, ch->id, state);
    }
    else if (assign_state(state) == ASSIGN_NONE)
    {
      chcfg->quantize_mode = delta_modulo_step(chcfg->quantize_mode, enc_delta(&state->ui->in, ch->encoder), QUANTIZE_MODE_COUNT);
    }
    break;
  case SHIFT_STATE_MON:
    if (pressed)
    {
      if (assign_state(state) == ASSIGN_NONE)
      {
        assign_reset(state);
        assign_event(ASSIGN_CHANNEL, ch->id, state);
      }
      else if (assign_state(state) == ASSIGN_CHANNEL && assign_src(state) == ch->id)
      {
        assign_reset(state);
        chcfg->src_input = -1;
      }
    }

    if (!pressing)
    {
      chcfg->input_amp_mode = delta_modulo_step(chcfg->input_amp_mode, enc_delta(&state->ui->in, ch->encoder), INPUT_AMP_MODE_COUNT);
    }
    break;
  case SHIFT_STATE_CPY:
    if (pressed)
    {
      assign_event(ASSIGN_CHANNEL, ch->id, state);
    }
    break;
  case SHIFT_STATE_CLR:
    if (long_pressed)
    {
      clear_channel(ch->id, 1, state);
    }
    else if (pressed)
    {
      clear_channel(ch->id, 0, state);
    }
    break;
  case SHIFT_STATE_NONE:
    // Only a long press that spanned no encoder movement resets the param -
    // otherwise holding the button as an encoder modifier would wipe the
    // value the user was just adjusting.
    uint32_t t_no_rotation = state->hw_state->time - state->engine_state->channels_last_delta[ch->id];
    if (long_pressed && btn_held(&state->ui->in, ch->button) < t_no_rotation)
    {
      reset_channel_param(ch, state, state->engine_state->active_scene, state->ui->selected_param);
    }
    else
    {
      update_channel_param(ch, state);
    }
    break;
  default:
    break;
  }
}

void compute_channel_scene(const ChannelSetup* ch, UxState* state)
{
  (void) ch;
  (void) state;
}

void compute_channel(const ChannelSetup* ch, UxState* state)
{
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  float dt_s           = state->hw_state->dt * US_TO_S;

  int16_t avg[CH_PARAM_COUNT] = {0};
  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    if (state->engine_state->scenes_contribution[s] == 0)
    {
      continue;
    }

    for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
    {
      int16_t relative = (int16_t) (((int32_t) chcfg->params[s][p] * state->engine_state->scenes_contribution[s]) / 255);
      avg[p] += relative;
    }
  }

  float freq_param = avg[CH_PARAM_FRQ] / (float) N_FREQ_SCALE;
  float offset     = (float) avg[CH_PARAM_OFS];
  float amp        = (float) avg[CH_PARAM_AMP] * 0.5f;
  float shape      = (float) avg[CH_PARAM_SHP] / INT16_MAX;
  float phs        = (float) avg[CH_PARAM_PHS] / INT16_MAX;

  float freq_multiplier = freq_param >= 0 ? freq_param + 1.0f : -1.0f / (freq_param - 1.0f);
  float freq            = g_clk.beat_freq_smooth * freq_multiplier;

  int16_t gcd        = find_denominator(freq_multiplier, 8, 0.025f);
  float phase_delta  = dt_s * (freq + state->engine_state->channels_phase_correction[ch->id]);
  float phase_length = gcd > 0 ? gcd * freq_multiplier : 1.0f;
  float diff         = 0;

  float phase_next = state->engine_state->channels_shared_phase[ch->id] + phase_delta;

  if (phase_next >= phase_length)
    phase_next -= phase_length;
  else if (phase_next < 0.0f)
    phase_next += phase_length;

  state->engine_state->channels_shared_phase[ch->id] = phase_next;
  if (gcd > 0 && g_clk.have_beat)
  {
    float beat_mode    = (float) (g_clk.beat_counter % gcd) + g_clk.beat_phase;
    float target_phase = beat_mode * freq_multiplier;
    if (target_phase >= phase_length)
      target_phase -= phase_length;
    diff = gcd > 0 ? phase_error(target_phase, state->engine_state->channels_shared_phase[ch->id], phase_length) : 0;
  }

  state->engine_state->channels_phase_correction[ch->id] =
      (state->engine_state->channels_phase_correction[ch->id] * (1.0f - k_sync) + diff * k_sync);

  float phase = fmodf(state->engine_state->channels_shared_phase[ch->id] + phs, 1.0f);
  if (phase < 0.0f)
    phase += 1.0f;
  while (phase >= 1.0f)
  {
    phase -= 1.0f;
  }

  float mod = (float) avg[CH_PARAM_MOD] / INT16_MAX;

  float raw = 0;

  // Pattern length is held steady for the rest of the cycle: switching it
  // mid-cycle moves the step grid under the playhead and jumps the output by
  // up to 1.8 of a 2.0 range. Re-latched on the cycle wrap - where it is
  // seamless, because slot 0 reads the same at every length - and immediately
  // while the encoder is being turned, so auditioning stays responsive even on
  // a slow LFO.
  int8_t* latched_idx = &state->engine_state->channels_length_idx[ch->id];
  float prev_phase    = state->engine_state->channels_prev_phase[ch->id];
  uint8_t wrapped     = phase < prev_phase;
  uint8_t editing     = (state->hw_state->time - state->engine_state->channels_last_delta[ch->id]) < MOD_EDIT_WINDOW;

  state->engine_state->channels_prev_phase[ch->id] = phase;

  if (*latched_idx < 0 || wrapped || editing)
  {
    *latched_idx = (int8_t) sr_length_index_from_mod(mod);
  }

  switch (chcfg->shape_mode)
  {
  case SHAPE_STEPPED_SMOOTH:
    raw = stepped_random(phase, shape, *latched_idx, SR_HOLD_SMOOTH);
    break;
  case SHAPE_STEPPED_SEMI:
    raw = stepped_random(phase, shape, *latched_idx, SR_HOLD_SEMI);
    break;
  case SHAPE_STEPPED_HARD:
    raw = stepped_random(phase, shape, *latched_idx, SR_HOLD_HARD);
    break;
  case SHAPE_LFO:
  default:
    raw = wavetable_lookup(phase_mod(phase, mod), shape) / (float) INT16_MAX;
    // raw = wave_fn(phase, shape, mod);
    break;
  }

  float value = offset + amp * raw;

  state->engine_state->cfrm[ch->id]  = freq_multiplier;
  state->engine_state->cgcd[ch->id]  = gcd;
  state->engine_state->cphsc[ch->id] = state->engine_state->channels_shared_phase[ch->id];
  state->engine_state->csphs[ch->id] = phase;
  state->engine_state->cshp[ch->id]  = shape;
  state->engine_state->cmod[ch->id]  = mod;

  if (chcfg->src_input >= 0 && chcfg->input_amp_mode != INPUT_AMP_DISABLED)
  {
    int16_t input_val = state->hw_state->input_state[chcfg->src_input];
    if (chcfg->input_amp_mode == INPUT_AMP_ADD)
    {
      value += input_val;
    }
    else if (chcfg->input_amp_mode == INPUT_AMP_MULT)
    {
      value *= (float) iclamp(input_val, INT16_MIN / 4, INT16_MAX / 4) / (float) (INT16_MAX / 4);
    }
  }

  if (value > INT16_MAX)
    value = INT16_MAX;
  else if (value < INT16_MIN)
    value = INT16_MIN;

  switch (chcfg->quantize_mode)
  {
  case QUANTIZE_CONTINUOUS:
    state->engine_state->channels_output_level[ch->id] = quantize_value((int16_t) value, state->engine_config->quantize_mask);
    break;
  case QUANTIZE_TRIG_SRC:
    if (chcfg->src_trig >= 0 && state->hw_state->trigger_src[chcfg->src_trig])
    {
      state->engine_state->channels_output_level[ch->id] = quantize_value((int16_t) value, state->engine_config->quantize_mask);
    }
    break;
  default:
    state->engine_state->channels_output_level[ch->id] = (int16_t) value;
  }
}

void write_channel_led(const ChannelSetup* ch, UxState* state)
{
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  if (state->dt < state->ui->channels_edit_hold[ch->id])
  {
    state->ui->channels_edit_hold[ch->id] -= state->dt;
  }
  else
  {
    state->ui->channels_edit_hold[ch->id] = 0;
  }

  uint8_t mark = state->ui->channels_edit_hold[ch->id] > 0;

  switch (state->ui->shift_state)
  {
  case SHIFT_STATE_SYS:
    led_set_hsv(state, ch->led, shape_mode_color[chcfg->shape_mode], SAT_HIG, VAL_LOW);
    break;
  case SHIFT_STATE_QNT:
    if (assign_state(state) == ASSIGN_TRIG_SRC)
    {
      if (assign_src(state) == ch->id)
      {
        led_set_hsv(state, ch->led, HUE_CYAN, SAT_HIG, VAL_LOW);
      }
      else
      {
        led_set_hsv(state, ch->led, HUE_CYAN, SAT_OFF, state->ui->blink_fast * VAL_LOW);
      }
    }
    else
    {
      led_set_hsv(state, ch->led, quantize_mode_color[chcfg->quantize_mode], SAT_HIG, VAL_LOW);
    }
    break;
    //
  case SHIFT_STATE_CLR:
    int8_t alt = btn_holding(&state->ui->in, ch->button, UI_T_VLONG);
    led_set_hsv(state, ch->led, HUE_RED, SAT_HIG, (state->ui->blink_fast || alt) * VAL_LOW);
    break;
  case SHIFT_STATE_CPY:
    if (assign_state(state) == ASSIGN_NONE)
    {
      led_set_hsv(state, ch->led, 0, SAT_OFF, state->ui->blink_fast * VAL_LOW);
    }
    else if (assign_state(state) == ASSIGN_CHANNEL)
    {
      if (assign_src(state) == ch->id)
      {
        led_set_hsv(state, ch->led, HUE_GREEN, SAT_HIG, VAL_LOW);
      }
      else
      {
        led_set_hsv(state, ch->led, 0, SAT_OFF, state->ui->blink_fast * VAL_LOW);
      }
    }
    else
    {
      led_set_hsv(state, ch->led, 0, SAT_OFF, VAL_OFF);
    }
    break;
  case SHIFT_STATE_MON:
    if (assign_state(state) == ASSIGN_CHANNEL)
    {
      if (assign_src(state) == ch->id)
      {
        led_set_hsv(state, ch->led, HUE_RED, SAT_MED, state->ui->blink_fast * VAL_LOW);
      }
      else
      {
        led_set_hsv(state, ch->led, 0, SAT_OFF, VAL_OFF);
      }
      break;
    }
    led_set_hsv(state, ch->led, input_amp_mode_color[chcfg->input_amp_mode], SAT_HIG, VAL_LOW);
    /* fall through */
  default:
    if (mark)
      break;
    led_set_dac(state, ch->led, state->engine_state->channels_output_level[ch->id]);
    break;
  }

  if (mark)
  {
    switch (state->ui->selected_param)
    {
    case CH_PARAM_FRQ:
      led_set_hsv(state, ch->led, state->ui->channels_edit_hue[ch->id], SAT_MAX, state->ui->blink_fast * VAL_MED);
      break;
    default:
      led_set_adcr(state, ch->led, chcfg->params[state->engine_state->active_scene][state->ui->selected_param]);
      break;
    }
  }
}

// Pure: updates this channel's output trigger edge state. Split out of
// write_channel_dac so the trigger logic is testable without a DAC.
void detect_channel_trigger(const ChannelSetup* ch, UxState* state)
{
  EngineState* es  = state->engine_state;
  int16_t curr_out = es->channels_output_level[ch->id];

  if (es->channels_trig_state[ch->id] < 1 && es->channels_prev_out[ch->id] < CHANNEL_TRIG_THRESH && curr_out >= CHANNEL_TRIG_THRESH)
  {
    es->channels_trig_state[ch->id] = 1;
    es->channels_trig_flag[ch->id]  = 1;
  }
  else if (curr_out < CHANNEL_TRIG_THRESH_LOW)
  {
    es->channels_trig_state[ch->id] = 0;
  }
  es->channels_prev_out[ch->id] = curr_out;
}

void write_channel_dac(const ChannelSetup* ch, UxState* state)
{
  dacadc_write(ch->dac_channel, state->engine_state->channels_output_level[ch->id]);
}

uint8_t read_channel_trig_state(const ChannelSetup* ch, UxState* state)
{
  if (state->engine_state->channels_trig_flag[ch->id])
  {
    state->engine_state->channels_trig_flag[ch->id] = 0;
    return 1;
  }
  return 0;
}
