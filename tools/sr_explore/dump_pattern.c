// Dumps the pattern the ear hears - the normalised value at every step, plus
// the tie weights and step widths the character features need - straight out of
// the shipping value path and the generated correction.
//
// from_c.py reads this. Measuring the C rather than srmodel.py's replica is
// what makes it safe to judge a change to the *correction*: the replica does not
// carry the generated table, and re-deriving it in Python is how the two came
// apart in the first place.
//
//   cc -O2 -I Core/Inc/Lib -o /tmp/dump_pat tools/sr_explore/dump_pattern.c \
//       Core/Src/Lib/stepped_random.c -lm
//   /tmp/dump_pat > /tmp/pat.txt
//
// The values are the ones the ear gets - the correction applied, then both of
// the reshapings the knobs drive - so the character features describe what the
// mode plays rather than what the pattern engine produced.
#include "helpers.h"
#include "stepped_random.h"
#include "stepped_random_pattern.h"
#include "stepped_random_table.h"
#include <stdio.h>

#define SWEEP 400

static const SrSlots slots = {sr_slot_base, sr_slot_rate, sr_slot_gate, sr_slot_gate2};

static void emit(int li, const char* axis, float fixed, float pos, float shape, float mod)
{
  int length = sr_lengths[li];
  SrDrive d  = sr_drive(shape, mod);
  SrMorph m  = sr_morph(shape, mod, length, SR_HOLD_MAX);
  SrNorm n   = sr_norm_exact(shape, mod, li);

  float swing = sr_swing_amount(mod);
  float total = (float) length + ((length & 1) ? swing : 0.0f);

  printf("%d %s %.4f %.6f :", li, axis, fixed, pos);
  for (int i = 0; i < length; i++)
    printf(" %.5f",
           sr_terrace_map(
               sr_terrace_map(sr_bias_map(fclamp(sr_step_value(i, &m, &slots, SR_JUMP_GRID) * n.gain + n.offset, -1.0f, 1.0f), d.bias),
                              d.terrace),
               d.terrace));
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
