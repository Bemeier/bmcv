#include "channel.h"
#include "clock_sync.h"
#include "config.h"
#include "engine_state.h"
#include "helpers.h"
#include "hw_setup.h"
#include "hw_state.h"
#include "math.h"
#include "stepped_random.h"
#include "stepped_random_table.h"
#include "wave_fn.h"
#include <stdint.h>

#define N_FREQ_SCALE 255

// A channel's own output can trigger another channel's sample & hold, so it
// gets the same treatment an input jack does: the same two voltages (~1.25V
// rising, ~0.98V falling), expressed in the DAC domain this signal lives in.
//
// These used to be 1024/800 written out again - the input-side numbers copied
// across without rescaling, which in DAC units is 0.31V, so a channel counted
// as triggering almost the moment it left zero.
#define CHANNEL_TRIG_THRESH TRIG_THRESH_DAC
#define CHANNEL_TRIG_THRESH_LOW TRIG_THRESH_LOW_DAC

// How long after the last encoder movement a channel still counts as being
// actively edited, during which a pattern-length change applies immediately
// rather than waiting for the cycle wrap.
#define MOD_EDIT_WINDOW MS(500)

// Mute fade, in milliseconds. At the ~3kHz tick rate this is about 15 steps,
// and the converter is band-limited well below that, so it is comfortably
// enough to keep the transition silent.
#define MUTE_RAMP_MS 5.0f

// Per-channel mutable state (prev_out / trig_state / trig_flag / last_delta)
// lives in EngineState so it resets with the rest of the engine and does not
// leak between tests or engine instances.

static const float k_sync = 0.075f;

void channel_init(uint8_t ch, EngineState* es)
{
  es->channels_shared_phase[ch]     = 0;
  es->channels_phase_correction[ch] = 0;
  es->channels_prev_phase[ch]       = 0;
  es->channels_length_idx[ch]       = -1;   // latch on the first tick
  es->channels_mute_gain[ch]        = 1.0f; // open, not fading in from silence
}

void channel_reset_phase(uint8_t ch, EngineState* es)
{
  es->channels_shared_phase[ch]     = 0;
  es->channels_phase_correction[ch] = 0;
}

void channel_reset_param(uint8_t ch, EngineConfig* cfg, int8_t scene, int8_t param)
{
  if (scene < 0 || scene >= N_SCENES || param < 0 || param >= CH_PARAM_COUNT)
    return;

  // Frequency is a ratio: its neutral value is "one beat", which is -255 in
  // the multiplier table, not zero.
  cfg->channel_state[ch].params[scene][param] = (param == CH_PARAM_FRQ) ? -255 : 0;
}

void channel_reset(uint8_t ch, EngineState* es, EngineConfig* cfg, int8_t scene)
{
  channel_init(ch, es);

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    if (scene >= 0 && s != (uint8_t) scene)
      continue;

    for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
    {
      channel_reset_param(ch, cfg, (int8_t) s, (int8_t) p);
    }
  }
}

void channel_compute(uint8_t ch, EngineState* es, const EngineConfig* cfg, const HwState* hw)
{
  const ChannelConfig* chcfg = &cfg->channel_state[ch];
  float dt_s                 = hw->dt * US_TO_S;

  int16_t avg[CH_PARAM_COUNT] = {0};
  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    if (es->scenes_contribution[s] == 0)
    {
      continue;
    }

    for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
    {
      int16_t relative = (int16_t) (((int32_t) chcfg->params[s][p] * es->scenes_contribution[s]) / 255);
      avg[p] += relative;
    }
  }

  float freq_param = avg[CH_PARAM_FRQ] / (float) N_FREQ_SCALE;
  float offset     = (float) avg[CH_PARAM_OFS];
  float amp        = (float) avg[CH_PARAM_AMP] * 0.5f;
  float shape      = (float) avg[CH_PARAM_SHP] / INT16_MAX;
  float phs        = (float) avg[CH_PARAM_PHS] / INT16_MAX;

  float freq_multiplier = freq_param >= 0 ? freq_param + 1.0f : -1.0f / (freq_param - 1.0f);
  float freq            = es->clock.beat_freq_smooth * freq_multiplier;

  int16_t gcd        = find_denominator(freq_multiplier, 8, 0.025f);
  float phase_delta  = dt_s * (freq + es->channels_phase_correction[ch]);
  float phase_length = gcd > 0 ? gcd * freq_multiplier : 1.0f;
  float diff         = 0;

  float phase_next = es->channels_shared_phase[ch] + phase_delta;

  if (phase_next >= phase_length)
    phase_next -= phase_length;
  else if (phase_next < 0.0f)
    phase_next += phase_length;

  es->channels_shared_phase[ch] = phase_next;
  if (gcd > 0 && es->clock.have_beat)
  {
    float beat_mode    = (float) (es->clock.beat_counter % gcd) + es->clock.beat_phase;
    float target_phase = beat_mode * freq_multiplier;
    if (target_phase >= phase_length)
      target_phase -= phase_length;
    diff = phase_error(target_phase, es->channels_shared_phase[ch], phase_length);
  }

  es->channels_phase_correction[ch] = (es->channels_phase_correction[ch] * (1.0f - k_sync) + diff * k_sync);

  float phase = fmodf(es->channels_shared_phase[ch] + phs, 1.0f);
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
  int8_t* latched_idx = &es->channels_length_idx[ch];
  float prev_phase    = es->channels_prev_phase[ch];
  uint8_t wrapped     = phase < prev_phase;
  uint8_t editing     = (hw->time - es->channels_last_delta[ch]) < MOD_EDIT_WINDOW;

  es->channels_prev_phase[ch] = phase;

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
    break;
  }

  float value = offset + amp * raw;

  ChannelEffective* eff = &es->channels_effective[ch];
  eff->freq_hz          = freq;
  eff->freq_ratio       = freq_multiplier;
  eff->phase            = phase;
  eff->shape            = shape;
  eff->mod              = mod;
  eff->amp              = amp;
  eff->offset           = offset;
  eff->gcd              = gcd;

  if (chcfg->src_input >= 0 && chcfg->input_amp_mode != INPUT_AMP_DISABLED)
  {
    int16_t input_val = hw->input_state[chcfg->src_input];
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
    es->channels_output_level[ch] = quantize_value((int16_t) value, cfg->quantize_mask);
    break;
  case QUANTIZE_TRIG_SRC:
    if (chcfg->src_trig >= 0 && hw->trigger_src[chcfg->src_trig])
    {
      es->channels_output_level[ch] = quantize_value((int16_t) value, cfg->quantize_mask);
    }
    break;
  default:
    es->channels_output_level[ch] = (int16_t) value;
  }
}

void channel_detect_trigger(uint8_t ch, EngineState* es)
{
  int16_t curr_out = es->channels_output_level[ch];

  if (es->channels_trig_state[ch] < 1 && es->channels_prev_out[ch] < CHANNEL_TRIG_THRESH && curr_out >= CHANNEL_TRIG_THRESH)
  {
    es->channels_trig_state[ch] = 1;
    es->channels_trig_flag[ch]  = 1;
  }
  else if (curr_out < CHANNEL_TRIG_THRESH_LOW)
  {
    es->channels_trig_state[ch] = 0;
  }
  es->channels_prev_out[ch] = curr_out;
}

// Gating happens here rather than by zeroing channels_output_level, so a muted
// channel keeps its real value for cross-modulation and for other channels
// using it as a trigger source. Mute is the output stage only.
//
// The gain ramps instead of jumping: a hard step to zero clicks.
void channel_apply_mute(uint8_t ch, EngineState* es, uint8_t muted, uint32_t dt_us)
{
  float* gain  = &es->channels_mute_gain[ch];
  float target = muted ? 0.0f : 1.0f;
  float step   = (dt_us * US_TO_S) / (MUTE_RAMP_MS * 0.001f);
  float to_go  = target - *gain;

  if (to_go > step)
    *gain += step;
  else if (to_go < -step)
    *gain -= step;
  else
    *gain = target;

  es->channels_gated_level[ch] = (int16_t) (*gain * (float) es->channels_output_level[ch]);
}

uint8_t channel_take_trig(uint8_t ch, EngineState* es)
{
  if (es->channels_trig_flag[ch])
  {
    es->channels_trig_flag[ch] = 0;
    return 1;
  }
  return 0;
}
