// Dumps the pattern the ear hears - the normalised value at every step, plus
// the tie weights and step widths the character features need - straight out of
// the shipping value path and the generated correction.
//
// from_c.py reads this. Measuring the C rather than srmodel.py's replica is
// what makes it safe to judge a change to the *correction*: the replica does not
// carry the generated table, and re-deriving it in Python is how the two came
// apart in the first place.
//
//   cc -O2 -I Core/Inc/Lib -o /tmp/dump_pat tools/sr_explore/dump_pattern.c -lm
//   /tmp/dump_pat > /tmp/pat.txt
#include "helpers.h"
#include "stepped_random_norm.h"
#include "stepped_random_pattern.h"
#include "stepped_random_table.h"
#include <stdio.h>

#define SWEEP 400

static const SrSlots slots = {sr_slot_base, sr_slot_rate, sr_slot_gate, sr_slot_gate2};

// stepped_random()'s own lookup, repeated here so the dumper needs nothing but
// headers - it is four lines and it cannot drift far without a test noticing.
static float norm_lookup(const float t[SR_LENGTH_COUNT][SR_MOD_BINS][SR_NORM_BINS], int li, int mb, float mf, int b, int bn, float bf)
{
  return lerp(lerp(t[li][mb][b], t[li][mb][bn], bf), lerp(t[li][mb + 1][b], t[li][mb + 1][bn], bf), mf);
}

// Build with -DSR_COMPUTED to measure the correction worked out from the
// pattern instead of read from the binned table - the two paths this is here to
// compare. See docs/plans/stepped-random-modes.md.
#ifdef SR_COMPUTED
static const SrNormCtx sr_ctx = {&slots, sr_lengths, SR_LENGTH_COUNT, SR_JUMP_GRID, SR_HOLD_MAX};
#endif

static void norm_for(float shape, float mod, int li, float* gain, float* offset)
{
  SrMorph m = sr_morph(shape, mod, sr_lengths[li], SR_HOLD_MAX);

  float bin_pos  = m.morph * (float) SR_NORM_BINS;
  int bin        = (int) bin_pos % SR_NORM_BINS;
  float bin_frac = bin_pos - (float) (int) bin_pos;
  int bin_next   = (bin + 1) % SR_NORM_BINS;

  float mod_pos  = (fclamp(mod, -1.0f, 1.0f) + 1.0f) * 0.5f * (float) (SR_MOD_BINS - 1);
  int mod_bin    = (int) mod_pos;
  float mod_frac = mod_pos - (float) mod_bin;
  if (mod_bin >= SR_MOD_BINS - 1)
  {
    mod_bin  = SR_MOD_BINS - 2;
    mod_frac = 1.0f;
  }

#ifdef SR_COMPUTED
  float lo = lerp(sr_centre_table[mod_bin][bin], sr_centre_table[mod_bin][bin_next], bin_frac);
  float hi = lerp(sr_centre_table[mod_bin + 1][bin], sr_centre_table[mod_bin + 1][bin_next], bin_frac);
  SrNorm n = sr_norm_at(&sr_ctx, sr_lengths[li], mod, m.orbit, lerp(lo, hi, mod_frac));
  *gain    = n.gain;
  *offset  = n.offset;
#else
  *gain   = norm_lookup(sr_norm_gain, li, mod_bin, mod_frac, bin, bin_next, bin_frac);
  *offset = norm_lookup(sr_norm_offset, li, mod_bin, mod_frac, bin, bin_next, bin_frac);
#endif
}

static void emit(int li, const char* axis, float fixed, float pos, float shape, float mod)
{
  int length = sr_lengths[li];
  SrMorph m  = sr_morph(shape, mod, length, SR_HOLD_MAX);
  float g, off;
  norm_for(shape, mod, li, &g, &off);

  float swing = sr_swing_amount(mod);
  float total = (float) length + ((length & 1) ? swing : 0.0f);

  printf("%d %s %.4f %.6f :", li, axis, fixed, pos);
  for (int i = 0; i < length; i++)
    printf(" %.5f", fclamp(sr_step_value(i, &m, &slots, SR_JUMP_GRID) * g + off, -1.0f, 1.0f));
  printf(" :");
  for (int i = 0; i < length; i++)
    printf(" %.5f", sr_tie_weight(i, &m, &slots, SR_JUMP_GRID));
  printf(" :");
  for (int i = 0; i < length; i++)
    printf(" %.5f", ((i & 1) ? 1.0f - swing : 1.0f + swing) / total);
  printf("\n");
}

int main(void)
{
  const float mods[]   = {-0.5f, 0.0f};
  const float shapes[] = {0.0f, 0.4f};

  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    for (int k = 0; k < 2; k++)
      for (int i = 0; i < SWEEP; i++)
      {
        float p = -1.0f + 2.0f * (float) i / (float) SWEEP;
        emit(li, "shape", mods[k], p, p, mods[k]);
      }
    for (int k = 0; k < 2; k++)
      for (int i = 0; i < SWEEP; i++)
      {
        float p = -1.0f + 2.0f * (float) i / (float) SWEEP;
        emit(li, "mod", shapes[k], p, shapes[k], p);
      }
  }
  return 0;
}
