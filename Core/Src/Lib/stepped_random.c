#include "stepped_random.h"
#include "helpers.h"
#include "stepped_random_pattern.h"
#include "stepped_random_table.h"

// Rhythmic random LFO.
//
// A cycle is divided into `length` steps, each holding a random value. The
// step lattice is read circularly, so the waveform closes seamlessly at the
// loop point and the pattern stays locked to the beat under the PLL. That
// requires `length` to be a whole number of steps per cycle - hence the
// curated table in the generated header rather than a continuously variable
// step count.
//
// Within a step the curve sits at its value for `hold` of the step, then eases
// to the next. The easing still reaches exactly 1.0 at the end of the step, so
// consecutive steps join continuously and, because the ease has zero slope at
// both ends, the derivative is zero at every boundary including the loop
// point. The result is a slewed sample-and-hold: audibly rhythmic, but a
// smooth curve with no clicks however hard the steps are set.
//
// What each step *shows* lives in stepped_random_pattern.h, shared with the
// table generator. This file is the part that only the runtime needs: finding
// the step a phase lands in, easing between two of them, and applying the
// correction the generator worked out.

// sr_lengths[] (steps per cycle) lives in the generated table so the
// normalisation data cannot drift out of step with it. It is curated rather
// than every integer, so each knob position is musically distinct; the odd
// lengths (3, 5, 7) drift against a 4/4 clock and give polyrhythms rather than
// a locked loop.

static const SrSlots sr_slots = {sr_slot_base, sr_slot_rate, sr_slot_gate, sr_slot_gate2};

// sr_step_value() keeps one tie weight per slot of a run on the stack.
_Static_assert(SR_JUMP_GRID <= SR_MAX_JUMP_GRID, "a run of ties must fit the weight buffer");

int sr_length_for_index(int length_idx) { return sr_lengths[iclamp(length_idx, 0, SR_LENGTH_COUNT - 1)]; }

// One value out of a [length][MOD][morph] table, interpolated on both bin axes.
// The length is an index, not a bin: it is one of a curated set and there is
// nothing between its entries.
static float sr_norm_lookup(const float table[SR_LENGTH_COUNT][SR_MOD_BINS][SR_NORM_BINS], int length_idx, int mod_bin, float mod_frac,
                            int bin, int bin_next, float bin_frac)
{
  float lo = lerp(table[length_idx][mod_bin][bin], table[length_idx][mod_bin][bin_next], bin_frac);
  float hi = lerp(table[length_idx][mod_bin + 1][bin], table[length_idx][mod_bin + 1][bin_next], bin_frac);
  return lerp(lo, hi, mod_frac);
}

float stepped_random(float phase, float shape, float mod, int length_idx, float hold)
{
  length_idx = iclamp(length_idx, 0, SR_LENGTH_COUNT - 1);
  int length = sr_lengths[length_idx];

  SrMorph m = sr_morph(shape, mod, length, SR_HOLD_MAX);

  // Which step the playhead is in. The steps are not all the same width - MOD
  // skews alternate ones long and short - so this is a small search rather than
  // a multiply, but still O(1).
  float within;
  int step = sr_step_at(phase, length, sr_swing_amount(mod), &within);
  int next = (step + 1 == length) ? 0 : step + 1;

  float from, to;
  sr_step_pair(step, next, &m, &sr_slots, SR_JUMP_GRID, &from, &to);

  // Hold, then ease.
  float span = 1.0f - hold;
  float ease = (span <= 0.0f) ? 1.0f : fclamp((within - hold) / span, 0.0f, 1.0f);

  float value = lerp(from, to, smoothstep(ease));

  // Lift patterns that would otherwise collapse toward flat (see the
  // normalisation notes in tools/gen_sr_table.c). A constant affine correction
  // across the cycle, so the loop still closes and the curve stays smooth.
  float bin_pos  = m.morph * (float) SR_NORM_BINS;
  int bin        = (int) bin_pos;
  float bin_frac = bin_pos - (float) bin;
  bin            = bin % SR_NORM_BINS;
  int bin_next   = (bin + 1) % SR_NORM_BINS;

  // The MOD axis of the same table. Both endpoints of the knob sit exactly on a
  // bin, so the top one is only ever reached exactly.
  float mod_pos  = (fclamp(mod, -1.0f, 1.0f) + 1.0f) * 0.5f * (float) (SR_MOD_BINS - 1);
  int mod_bin    = (int) mod_pos;
  float mod_frac = mod_pos - (float) mod_bin;
  if (mod_bin >= SR_MOD_BINS - 1)
  {
    mod_bin  = SR_MOD_BINS - 2;
    mod_frac = 1.0f;
  }

  float gain   = sr_norm_lookup(sr_norm_gain, length_idx, mod_bin, mod_frac, bin, bin_next, bin_frac);
  float offset = sr_norm_lookup(sr_norm_offset, length_idx, mod_bin, mod_frac, bin, bin_next, bin_frac);

  return fclamp(value * gain + offset, -1.0f, 1.0f);
}
