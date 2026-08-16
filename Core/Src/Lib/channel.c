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
#include "wavetable.h"
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

// The output stage's range limit. A clamp rather than a scaling: the parameters
// keep meaning what they say and the swing is cut off at the ends, so setting a
// channel to 0..5V does not quietly halve everything already dialled in.
static float clamp_output(float value, int8_t mode)
{
  switch (mode)
  {
  case CLAMP_BI_5:
    return fclamp(value, -(float) DAC_5V, (float) DAC_5V);
  case CLAMP_UNI_10:
    return fclamp(value, 0.0f, (float) INT16_MAX);
  case CLAMP_UNI_5:
    return fclamp(value, 0.0f, (float) DAC_5V);
  case CLAMP_BI_10:
  default:
    return fclamp(value, (float) INT16_MIN, (float) INT16_MAX);
  }
}

// PWM: a gate with an envelope on it.
//
// SHP is the pulse width. MOD splits ramp time between the two edges, each
// confined to its own segment - the rise inside the on-time, the fall inside
// the off-time - so the width still means what it says at either extreme. MOD 0
// is a hard gate, negative snaps up and decays, positive swells and then drops.
//
// The ramps are curved rather than linear, which is what makes the negative
// side read as an envelope rather than as a triangle: the attack is concave
// (quick off the floor, easing into the plateau) and the decay convex (steep,
// then a tail). One multiply each, and no expf on a path that runs eight times
// a tick.
static float pwm_shape(float phase, float shape, float mod)
{
  // Off both end stops, so the pulse never disappears entirely.
  float width = fclamp(0.5f + shape * 0.48f, 0.02f, 0.98f);

  if (phase < width)
  {
    float rise = (mod > 0.0f) ? mod * width : 0.0f;
    if (phase >= rise)
      return 1.0f; // covers rise == 0, where the edge is instant
    float x = phase / rise;
    return -1.0f + 2.0f * (2.0f * x - x * x);
  }

  float fall = (mod < 0.0f) ? -mod * (1.0f - width) : 0.0f;
  float t    = phase - width;
  if (t >= fall)
    return -1.0f;
  float y = 1.0f - t / fall;
  return -1.0f + 2.0f * y * y;
}

void channel_init(uint8_t ch, EngineState* es)
{
  es->channels_shared_phase[ch]      = 0;
  es->channels_phase_correction[ch]  = 0;
  es->channels_gcd[ch]               = 0; // not taken yet; see channel_compute
  es->channels_beat_origin[ch]       = 0;
  es->channels_ratio_seen[ch]        = 0;
  es->channels_ratio_still_since[ch] = 0;
  es->channels_cycle[ch]             = 0;
  es->channels_period_cycles[ch]     = 1;
  es->channels_prev_phase[ch]        = 0;
  es->channels_length_idx[ch]        = -1;   // latch on the first tick
  es->channels_mute_gain[ch]         = 1.0f; // open, not fading in from silence
}

// What a reset input does: back to phase zero. The alignment period goes with
// it - a reset re-establishes where the beat grid starts, and holding an origin
// measured from before it would leave the channel repeating on the old one.
void channel_reset_phase(uint8_t ch, EngineState* es)
{
  es->channels_shared_phase[ch]     = 0;
  es->channels_phase_correction[ch] = 0;
  es->channels_gcd[ch]              = 0;
  es->channels_beat_origin[ch]      = 0;
  es->channels_cycle[ch]            = 0;
  es->channels_period_cycles[ch]    = 1;
}

// scene < 0 means every scene, as the header has always said - it used to be
// rejected along with the out-of-range values, so the one caller that wanted
// "everywhere" had to loop for itself.
void channel_reset_param(uint8_t ch, EngineConfig* cfg, int8_t scene, int8_t param)
{
  if (ch >= N_CHANNELS || scene >= N_SCENES || param < 0 || param >= CH_PARAM_COUNT)
    return;

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    if (scene >= 0 && s != (uint8_t) scene)
      continue;

    // Frequency is a ratio: its neutral value is "one beat", which is -255 in
    // the multiplier table, not zero.
    cfg->channel_state[ch].params[s][param] = (param == CH_PARAM_FRQ) ? -255 : 0;
  }
}

void channel_reset(uint8_t ch, EngineState* es, EngineConfig* cfg, int8_t scene)
{
  channel_init(ch, es);

  for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
  {
    channel_reset_param(ch, cfg, scene, (int8_t) p);
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

  // Peak, not half of it. AMP at full scale swings the whole +/-10V the
  // converter can reach, so the range a channel can cover is the range the
  // module has - it used to top out at +/-5V unless offset or cross-modulation
  // pushed it further, which made the wider output clamps unreachable from the
  // oscillator alone.
  float amp   = (float) avg[CH_PARAM_AMP];
  float shape = (float) avg[CH_PARAM_SHP] / INT16_MAX;
  float phs   = (float) avg[CH_PARAM_PHS] / INT16_MAX;

  float freq_multiplier = freq_param >= 0 ? freq_param + 1.0f : -1.0f / (freq_param - 1.0f);
  float freq            = es->clock.beat_freq_smooth * freq_multiplier;

  // The alignment period, latched.
  //
  // find_denominator is a search for a rational approximation of the ratio, and
  // as the ratio moves its answer does not move with it - it jumps between 8,
  // 7, 6, none, 5 and back. The target phase is measured from an origin that
  // repeats every `gcd` beats, so every jump moves that origin and the target
  // teleports by whole beats. A one-second crossfade between two scenes at x1
  // and x2 did that 44 times, and the loop chased every one: 3.1 beats of phase
  // error and the oscillator pulled to 1.4x its own rate, which is heard.
  //
  // So it is taken once and held, and only re-taken when the super-period wraps
  // - the same rule the stepped pattern length follows, for the same reason.
  // The origin is re-taken with it, and that is what makes the change seamless:
  // at the wrap the channel is at phase 0 of its old period, and the new period
  // is defined to start there.
  int16_t gcd_now = find_denominator(freq_multiplier, 8, 0.025f);

  // ...and only taken while the ratio is holding still. A ratio on its way
  // through a crossfade is a rational multiple every few ticks and something
  // irrational the rest of the time, so acquiring on the instant meant taking
  // and discarding eleven alignment periods in a one-second fader move, with
  // the correction switching on and off along with them.
  if (fabsf(freq_multiplier - es->channels_ratio_seen[ch]) > PLL_RATIO_EPS)
  {
    es->channels_ratio_seen[ch]        = freq_multiplier;
    es->channels_ratio_still_since[ch] = hw->time;
  }
  uint8_t ratio_settled = (hw->time - es->channels_ratio_still_since[ch]) >= PLL_RATIO_STABLE_US;

  if (es->channels_gcd[ch] == 0)
  {
    es->channels_gcd[ch]         = gcd_now;
    es->channels_beat_origin[ch] = es->clock.beat_counter;
  }

  int16_t gcd       = es->channels_gcd[ch];
  float phase_delta = dt_s * (freq + es->channels_phase_correction[ch]);
  float diff        = 0;

  float phase_next = es->channels_shared_phase[ch] + phase_delta;

  // A non-finite phase never comes back on its own: every comparison below is
  // false for a NaN, so it survives the wrap, the fmodf and the next tick's
  // accumulate, and the channel is dead until the module is power-cycled. The
  // clock can no longer produce one, but a host is free to hand the engine any
  // dt and any beat_freq it likes, and one bad frame should cost one cycle.
  if (!isfinite(phase_next))
  {
    phase_next                        = 0.0f;
    es->channels_phase_correction[ch] = 0.0f;
  }

  // The accumulator is the oscillator's own phase, and it wraps at one cycle.
  // Never at anything else.
  //
  // It used to wrap at the super-period, gcd * ratio cycles - which is only
  // invisible in the output when that product is a whole number of cycles. It
  // was, near enough, while gcd was recomputed every tick: find_denominator
  // only returns a gcd for which gcd * ratio is within 0.025 of an integer, so
  // the step was there but bounded at 2.5% of a cycle. Latching gcd while the
  // ratio keeps moving removed that guarantee and the step became unbounded -
  // measured at 0.34 of a cycle through a crossfade, which is a jump in the
  // middle of a waveform and is heard as one.
  //
  // The alignment period belongs in the error term below, not in the
  // accumulator. Wrapping at a whole cycle costs nothing, because the waveform
  // either side of that wrap is identical.
  int16_t period_cycles = es->channels_period_cycles[ch] > 0 ? es->channels_period_cycles[ch] : 1;
  int16_t cycle         = es->channels_cycle[ch];

  while (phase_next >= 1.0f)
  {
    phase_next -= 1.0f;
    cycle = (int16_t) ((cycle + 1) % period_cycles);
  }
  while (phase_next < 0.0f)
  {
    phase_next += 1.0f;
    cycle = (int16_t) ((cycle + period_cycles - 1) % period_cycles);
  }

  es->channels_shared_phase[ch] = phase_next;
  es->channels_cycle[ch]        = cycle;

  // Re-take the alignment period at a super-period boundary, where the old
  // period and the new one agree on where the channel is: there
  // (beats_in % gcd) is 0, so the target is frac(beat_phase * ratio) both
  // before and after the origin moves, and nothing steps. It comes round within
  // gcd beats - four seconds at worst - and in the meantime the target stays
  // continuous under a moving ratio anyway, which is all the loop needs.
  //
  // With no alignment period there is no correction to disturb, so a latched
  // "none" is replaced as soon as there is something better.
  if (ratio_settled && gcd_now != gcd)
  {
    uint8_t seamless = (gcd <= 0) || ((es->clock.beat_counter - es->channels_beat_origin[ch]) % (uint64_t) gcd) == 0;
    if (seamless)
    {
      es->channels_gcd[ch]         = gcd_now;
      es->channels_beat_origin[ch] = es->clock.beat_counter;
      gcd                          = gcd_now;

      // The super-period in whole cycles. Whole because find_denominator only
      // returns a gcd for which gcd * ratio is within 0.025 of an integer -
      // that is the entire meaning of its answer.
      int16_t cycles                 = (gcd > 0) ? (int16_t) lrintf((float) gcd * freq_multiplier) : 1;
      es->channels_period_cycles[ch] = cycles > 0 ? cycles : 1;

      // The period restarts here, and the accumulator is at phase 0 of it.
      es->channels_cycle[ch] = 0;
      cycle                  = 0;
      period_cycles          = es->channels_period_cycles[ch];
    }
  }

  if (gcd > 0 && es->clock.have_beat)
  {
    // Modulo the alignment period before the multiply, not after: beat_counter
    // is unbounded, and a float that has to hold thousands of beats times a
    // ratio has no resolution left for the fraction that matters. This is what
    // keeps a channel exact after an hour of running.
    uint64_t beats_in  = es->clock.beat_counter - es->channels_beat_origin[ch];
    float beat_mode    = (float) (beats_in % (uint64_t) gcd) + es->clock.beat_phase;
    float target_phase = beat_mode * freq_multiplier;

    // Both sides measured over the whole super-period, not over one cycle.
    //
    // Reducing this to a single cycle would be tempting - it bounds the error
    // at half a cycle and costs nothing audible in the steady state, since the
    // waveform repeats. It does cost something musical: the loop would lock to
    // whichever of the `period_cycles` equivalent phases happened to be nearest,
    // so a x2/3 channel would still repeat every three beats but could sit a
    // third of a cycle off the bar. Which cycle of the pattern lands on the
    // downbeat is the point of aligning to a beat multiple at all.
    float period = (float) period_cycles;
    float here   = (float) cycle + es->channels_shared_phase[ch];

    target_phase = fmodf(target_phase, period);
    if (target_phase < 0.0f)
      target_phase += period;

    diff = phase_error(target_phase, here, period);
  }

  // A frequency offset proportional to the error: first order, so the error
  // decays exponentially and never overshoots. PLL_TAU_S is the time constant.
  float pull = diff / PLL_TAU_S;

  // Bounded, because the correction is a speed change and an unbounded one is
  // an unbounded lurch. The error wraps at half the super-period, so a channel
  // on a long alignment period could decide it was several beats out and act on
  // it at full gain. Past this limit the error closes at a constant rate rather
  // than an exponential one - slower for a large error, and silent.
  float pull_limit = fabsf(freq) * PLL_MAX_PULL;
  pull             = fclamp(pull, -pull_limit, pull_limit);

  // Smoothed toward it in seconds rather than in ticks, so the shape of the
  // loop is a property of the loop and not of the host's control rate.
  float smooth = fclamp(dt_s / PLL_SMOOTH_S, 0.0f, 1.0f);
  es->channels_phase_correction[ch] += (pull - es->channels_phase_correction[ch]) * smooth;

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
    *latched_idx = (int8_t) iclamp(chcfg->sr_length_idx, 0, SR_LENGTH_COUNT - 1);
  }

  switch (chcfg->shape_mode)
  {
  case SHAPE_STEPPED:
    // The correction is measured a slot at a time rather than worked out here,
    // because working it out means walking the whole pattern. Scanned first, so
    // a channel that has just arrived in this mode is corrected on its first
    // tick rather than after a cycle of it.
    sr_norm_scan(&es->channels_sr_scan[ch], shape, mod, *latched_idx, dt_s, ch == es->sr_scan_turn);
    raw = stepped_random_with(phase, shape, mod, *latched_idx, SR_HOLD_SMOOTH, &es->channels_sr_scan[ch].norm);
    break;
  case SHAPE_PWM:
    raw = pwm_shape(phase, shape, mod);
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
  eff->phase_offset     = phs;
  eff->shape            = shape;
  eff->mod              = mod;
  eff->amp              = amp;
  eff->offset           = offset;
  eff->gcd              = gcd;
  eff->phase_error      = diff;

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

  // Last, so it bounds the cross-modulated result and not just the oscillator.
  value = clamp_output(value, chcfg->clamp_mode);

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
