#include "channel.h"
#include "assign.h"
#include "clock_sync.h"
#include "color_presets.h"
#include "dac_adc.h"
#include "helpers.h"
#include "hw_setup.h"
#include "math.h"
#include "state.h"
#include "stepped_random.h"
#include "ux_state.h"
#include "wave_fn.h"
#include "ws2811.h"
#include <stdint.h>

#define N_SHP_LEVELS 8
#define N_AMP_LEVELS 11
#define N_FREQ_MULTIPLIERS 31
#define N_FREQ_SCALE 255
// #define SHAPE_INTERVAL INT16_MAX / M

#define TRIG_THRESH 1024
#define TRIG_THRESH_LOW 800

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

static int16_t prev_out[N_CHANNELS];

static int16_t trig_state[N_CHANNELS];

static int16_t trig_flag[N_CHANNELS];

static uint32_t last_delta[N_CHANNELS];

static uint8_t quantize_mode_color[QUANTIZE_MODE_COUNT]   = {HUE_RED, HUE_MAGENTA, HUE_CYAN};
static uint8_t input_amp_mode_color[INPUT_AMP_MODE_COUNT] = {HUE_RED, HUE_GREEN, HUE_YELLOW};
static uint8_t shape_mode_color[SHAPE_MODE_COUNT]         = {HUE_GREEN, HUE_MAGENTA};

static float k_sync = 0.075f;

void update_channel_param(const ChannelSetup* ch, UxState* state)
{
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  int8_t param         = state->engine_state->selected_param;
  int16_t delta        = state->hw_state->encoder_delta[ch->encoder];
  int8_t alt           = state->hw_state->button_pressed_t[ch->button] > 0;
  if (delta == 0)
    return;

  last_delta[ch->id] = state->hw_state->time;

  state->engine_state->channels_mark_for[ch->id] = MS(1000);

  if (alt)
  {
    chcfg->params[state->engine_state->active_scene][param] += 32 * delta;
  }
  else if (param == CH_PARAM_FRQ)
  {
    size_t idx = 0;
    chcfg->params[state->engine_state->active_scene][param] =
        val_neighbour(chcfg->params[state->engine_state->active_scene][param], delta, quantized_multipliers, N_FREQ_MULTIPLIERS, &idx);
    state->engine_state->channels_mark_hue[ch->id] = quantized_multipliers_colors[idx];
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
  int8_t long_pressed  = state->hw_state->button_released_t[ch->button] > MS(500);
  int8_t pressed       = state->hw_state->button_released_t[ch->button] > MS(10);
  int8_t pressing      = state->hw_state->button_pressed_t[ch->button] > 0;
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  switch (state->engine_state->shift_state)
  {
  case SHIFT_STATE_SYS:
    chcfg->shape_mode = delta_modulo_step(chcfg->shape_mode, state->hw_state->encoder_delta[ch->encoder], SHAPE_MODE_COUNT);
    break;
  case SHIFT_STATE_QNT:
    if (pressed && assign_state() == ASSIGN_NONE)
    {
      chcfg->quantize_mode = QUANTIZE_TRIG_SRC;
      assign_event(ASSIGN_TRIG_SRC, ch->id, state);
    }
    else if (pressed && assign_state() == ASSIGN_TRIG_SRC)
    {
      assign_event(ASSIGN_CHANNEL, ch->id, state);
    }
    else if (assign_state() == ASSIGN_NONE)
    {
      chcfg->quantize_mode = delta_modulo_step(chcfg->quantize_mode, state->hw_state->encoder_delta[ch->encoder], QUANTIZE_MODE_COUNT);
    }
    break;
  case SHIFT_STATE_MON:
    if (pressed)
    {
      if (assign_state() == ASSIGN_NONE)
      {
        assign_reset();
        assign_event(ASSIGN_CHANNEL, ch->id, state);
      }
      else if (assign_state() == ASSIGN_CHANNEL && assign_src() == ch->id)
      {
        assign_reset();
        chcfg->src_input = -1;
      }
    }

    if (!pressing)
    {
      chcfg->input_amp_mode = delta_modulo_step(chcfg->input_amp_mode, state->hw_state->encoder_delta[ch->encoder], INPUT_AMP_MODE_COUNT);
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
    uint32_t t_no_rotation = state->hw_state->time - last_delta[ch->id];
    if (long_pressed && state->hw_state->button_released_t[ch->button] < t_no_rotation)
    {
      reset_channel_param(ch, state, state->engine_state->active_scene, state->engine_state->selected_param);
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

  if (chcfg->shape_mode == SHAPE_LFO)
  {
    raw = wavetable_lookup(phase_mod(phase, mod), shape) / (float) INT16_MAX;
    // raw = wave_fn(phase, shape, mod);
  }
  else if (chcfg->shape_mode == SHAPE_STEPPED_RANDOM)
  {
    raw = stepped_random(phase, shape, mod);
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
  if (state->dt < state->engine_state->channels_mark_for[ch->id])
  {
    state->engine_state->channels_mark_for[ch->id] -= state->dt;
  }
  else
  {
    state->engine_state->channels_mark_for[ch->id] = 0;
  }

  uint8_t mark = state->engine_state->channels_mark_for[ch->id] > 0;

  switch (state->engine_state->shift_state)
  {
  case SHIFT_STATE_SYS:
    ws2811_setled_hsv(ch->led, shape_mode_color[chcfg->shape_mode], SAT_HIG, VAL_LOW);
    break;
  case SHIFT_STATE_QNT:
    if (assign_state() == ASSIGN_TRIG_SRC)
    {
      if (assign_src() == ch->id)
      {
        ws2811_setled_hsv(ch->led, HUE_CYAN, SAT_HIG, VAL_LOW);
      }
      else
      {
        ws2811_setled_hsv(ch->led, HUE_CYAN, SAT_OFF, state->engine_state->blink_fast * VAL_LOW);
      }
    }
    else
    {
      ws2811_setled_hsv(ch->led, quantize_mode_color[chcfg->quantize_mode], SAT_HIG, VAL_LOW);
    }
    break;
    //
  case SHIFT_STATE_CLR:
    int8_t alt = state->hw_state->button_pressed_t[ch->button] > MS(1000);
    ws2811_setled_hsv(ch->led, HUE_RED, SAT_HIG, (state->engine_state->blink_fast || alt) * VAL_LOW);
    break;
  case SHIFT_STATE_CPY:
    if (assign_state() == ASSIGN_NONE)
    {
      ws2811_setled_hsv(ch->led, 0, SAT_OFF, state->engine_state->blink_fast * VAL_LOW);
    }
    else if (assign_state() == ASSIGN_CHANNEL)
    {
      if (assign_src() == ch->id)
      {
        ws2811_setled_hsv(ch->led, HUE_GREEN, SAT_HIG, VAL_LOW);
      }
      else
      {
        ws2811_setled_hsv(ch->led, 0, SAT_OFF, state->engine_state->blink_fast * VAL_LOW);
      }
    }
    else
    {
      ws2811_setled_hsv(ch->led, 0, SAT_OFF, VAL_OFF);
    }
    break;
  case SHIFT_STATE_MON:
    if (assign_state() == ASSIGN_CHANNEL)
    {
      if (assign_src() == ch->id)
      {
        ws2811_setled_hsv(ch->led, HUE_RED, SAT_MED, state->engine_state->blink_fast * VAL_LOW);
      }
      else
      {
        ws2811_setled_hsv(ch->led, 0, SAT_OFF, VAL_OFF);
      }
      break;
    }
    ws2811_setled_hsv(ch->led, input_amp_mode_color[chcfg->input_amp_mode], SAT_HIG, VAL_LOW);
    /* fall through */
  default:
    if (mark)
      break;
    ws2811_setled_dac(ch->led, state->engine_state->channels_output_level[ch->id]);
    break;
  }

  if (mark)
  {
    switch (state->engine_state->selected_param)
    {
    case CH_PARAM_FRQ:
      ws2811_setled_hsv(ch->led, state->engine_state->channels_mark_hue[ch->id], SAT_MAX, state->engine_state->blink_fast * VAL_MED);
      break;
    default:
      ws2811_setled_adcr(ch->led, chcfg->params[state->engine_state->active_scene][state->engine_state->selected_param]);
      break;
    }
  }
}

void write_channel_dac(const ChannelSetup* ch, UxState* state)
{
  int16_t curr_out = state->engine_state->channels_output_level[ch->id];
  if (trig_state[ch->id] < 1 && prev_out[ch->id] < TRIG_THRESH && curr_out >= TRIG_THRESH)
  {
    trig_state[ch->id] = 1;
    trig_flag[ch->id]  = 1;
  }
  else if (curr_out < TRIG_THRESH_LOW)
  {
    trig_state[ch->id] = 0;
  }
  prev_out[ch->id] = curr_out;
  dacadc_write(ch->dac_channel, curr_out);
}

uint8_t read_channel_trig_state(const ChannelSetup* ch)
{
  if (trig_flag[ch->id])
  {
    trig_flag[ch->id] = 0;
    return 1;
  }
  return 0;
}
