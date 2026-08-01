#include "stepped_random.h"
#include "helpers.h"
#include "stepped_random_table.h"

// Rhythmic random LFO.
//
// A cycle is divided into `length` steps, each holding a random value. The
// step lattice is read circularly, so the waveform closes seamlessly at the
// loop point and the pattern stays locked to the beat under the PLL. That
// requires `length` to be a whole number of steps per cycle - hence the
// curated table below rather than a continuously variable step count.
//
// Within a step the curve sits at its value for `hold` of the step, then eases
// to the next. The easing still reaches exactly 1.0 at the end of the step, so
// consecutive steps join continuously and, because the ease has zero slope at
// both ends, the derivative is zero at every boundary including the loop
// point. The result is a slewed sample-and-hold: audibly rhythmic, but a
// smooth curve with no clicks however hard the steps are set.

// sr_lengths[] (steps per cycle) lives in the generated table so the
// normalisation data below cannot drift out of step with it. It is curated
// rather than every integer, so each knob position is musically distinct; the
// odd lengths (3, 5, 7) drift against a 4/4 clock and give polyrhythms rather
// than a locked loop.

// Chance that a step repeats the previous value instead of taking a new one,
// which gives the pattern ties and rests rather than a fresh value every step.
#define SR_HOLD_PROBABILITY 0.30f

// Repeats are faded out on short patterns, where a single repeat would flatten
// too large a fraction of the cycle.
#define SR_HOLD_FADE_IN_STEPS 6.0f

// fractf() for values already known to be non-negative - avoids floorf(), which
// is a call on this target and would otherwise dominate the cost here.
static inline float sr_fract_pos(float x) { return x - (float) (int) x; }

// Triangle, period 1, range [-1, 1]. Chosen over a sine because it gives a
// uniform spread of values: the random levels cover the output range evenly
// rather than bunching at the extremes.
static inline float sr_tri(float x)
{
  float f = sr_fract_pos(x);
  return 4.0f * fabsf(f - 0.5f) - 1.0f;
}

// Each slot's value travels its own orbit as `morph` turns, at its own integer
// rate. Neighbouring morph positions therefore stay similar (every value has
// only moved a little), while different rates mean the pattern genuinely
// reshuffles instead of just sliding. Integer rates keep it periodic in morph,
// so the knob wrapping past its end lands back where the morph started.
//
// Crucially the spread of values is the same at every morph position, so there
// is no setting at which the output collapses toward flat - unlike crossfading
// between two independent random sequences, which dips toward their mean.
static inline float sr_slot_value(int slot, float morph) { return sr_tri(sr_slot_base[slot] + morph * (float) sr_slot_rate[slot]); }

static inline int sr_slot_jumps(int slot, float hold_probability) { return sr_slot_gate[slot] >= hold_probability; }

// The slot whose value this step actually shows: itself if it takes a new
// value, otherwise the most recent earlier slot that did. Terminates within
// SR_JUMP_GRID steps and never goes below 0, because the table pins every
// fourth gate - including slot 0 - to always jump.
static inline int sr_source_slot(int slot, float hold_probability)
{
  while (!sr_slot_jumps(slot, hold_probability))
  {
    slot--;
  }
  return slot;
}

float stepped_random(float phase, float shape, float mod, float hold)
{
  // Pattern length, snapped to the curated set.
  float length_pos = fclamp(0.5f * (mod + 1.0f), 0.0f, 1.0f);
  int length_idx   = (int) (length_pos * (float) (SR_LENGTH_COUNT - 1) + 0.5f);
  int length       = sr_lengths[length_idx];

  // Morph position. One full turn across the knob, matching the way the
  // parameter itself wraps at its extremes.
  float morph = 0.5f * (shape + 1.0f);

  float hold_probability = SR_HOLD_PROBABILITY * fclamp((float) (length - 2) / SR_HOLD_FADE_IN_STEPS, 0.0f, 1.0f);

  float x  = fclamp(phase, 0.0f, 1.0f) * (float) length;
  int step = (int) x;
  if (step >= length)
  {
    step = length - 1; // phase is [0,1), but do not let a rounding edge index off the end
  }
  float within = x - (float) step;

  int source      = sr_source_slot(step, hold_probability);
  int next        = (step + 1 == length) ? 0 : step + 1;
  int next_source = sr_slot_jumps(next, hold_probability) ? next : source;

  float from = sr_slot_value(source, morph);
  float to   = sr_slot_value(next_source, morph);

  // Hold, then ease.
  float span = 1.0f - hold;
  float ease = (span <= 0.0f) ? 1.0f : fclamp((within - hold) / span, 0.0f, 1.0f);

  float value = lerp(from, to, smoothstep(ease));

  // Lift patterns that would otherwise collapse toward flat (see the
  // normalisation notes in tools/gen_sr_table.c). A constant affine correction
  // across the cycle, so the loop still closes and the curve stays smooth.
  float bin_pos  = morph * (float) SR_NORM_BINS;
  int bin        = (int) bin_pos;
  float bin_frac = bin_pos - (float) bin;
  bin            = bin % SR_NORM_BINS;
  int bin_next   = (bin + 1) % SR_NORM_BINS;

  float gain   = lerp(sr_norm_gain[length_idx][bin], sr_norm_gain[length_idx][bin_next], bin_frac);
  float offset = lerp(sr_norm_offset[length_idx][bin], sr_norm_offset[length_idx][bin_next], bin_frac);

  return fclamp(value * gain + offset, -1.0f, 1.0f);
}
