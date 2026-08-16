// Generates the stepped_random slot and normalisation tables. Run once; output
// is pasted into Core/Inc/Lib/stepped_random_table.h. See the `sr-table` recipe
// in the Justfile.
//
// Neither the value path nor the correction is repeated here. Both come from
// the headers the runtime uses - stepped_random_pattern.h and
// stepped_random_norm.h - so this file is only the table's geometry: which
// points to sample, how to survive being interpolated between them, and how to
// print the result. A copy of a pattern generator is a correction quietly aimed
// at a pattern the module no longer plays.
#include "stepped_random_norm.h"
#include "stepped_random_pattern.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define SR_MAX_LENGTH 64
#define SR_JUMP_GRID 4
#define SR_MAX_ORBIT_RATE 4

// ---------------------------------------------------------------------------
// Normalisation table.
//
// With only a handful of steps in a cycle, the slot orbits can all cross at
// once, and the pattern collapses toward flat for a stretch of the shape
// sweep. Measured: at length 2 that is 18% of the sweep, in runs 15% wide -
// wide enough to turn a chunk of the knob into dead travel. The value shaper
// adds its own reasons to collapse, which is why the gain ceiling below is
// higher than the 10 that served the plain orbits.
//
// The correction that stops that is worked out in stepped_random_norm.h, and
// mostly at runtime: a channel measures its own pattern a slot at a time. What
// it cannot measure is the centring constant, which is an average across every
// pattern length - so that is what this bakes, and the only thing it bakes.
// 8 KB, against the 180 KB the length-indexed gain and offset tables cost.
//
// Span does not depend on the hold/smoothness setting: the eased curve passes
// exactly through each slot value, so one table serves all the stepped modes.
//
// The second axis is MOD itself, not the hold probability it used to be. MOD
// now rotates the gates as well as raising the threshold, so the probability no
// longer determines which steps tie and a table indexed by it would be
// correcting the wrong pattern. Indexing MOD directly costs nothing and cannot
// go stale.
//
// That axis is a smoothing, not an exact fit: the true span steps discretely as
// each slot's gate is crossed, so the bins are interpolated between - which is
// fine, because what the correction asks for changes smoothly with the knob
// rather than jumping.
// ---------------------------------------------------------------------------

#define SR_NORM_BINS 128

// The MOD axis: bins spanning the whole knob, endpoints included.
#define SR_MOD_BINS 16
#define SR_HOLD_MAX 0.85f

static const uint8_t sr_lengths[] = {3, 4, 5, 6, 8, 12, 16, 24, 32, 48, 64};
#define SR_LENGTH_COUNT ((int) (sizeof(sr_lengths) / sizeof(sr_lengths[0])))

static float g_base[SR_MAX_LENGTH];
static uint8_t g_rate[SR_MAX_LENGTH];
static float g_gate[SR_MAX_LENGTH];
static float g_gate2[SR_MAX_LENGTH];

static uint32_t seed_of(int i) { return hash_u32((uint32_t) i * 0x9E3779B9u + 0x6D2B79F5u); }

static void build_slots(void)
{
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    uint32_t seed = seed_of(i);

    g_base[i] = hash01(seed);

    int r     = 1 + (int) (hash01(seed ^ 0x85EBCA6Bu) * (float) SR_MAX_ORBIT_RATE);
    g_rate[i] = (uint8_t) (r > SR_MAX_ORBIT_RATE ? SR_MAX_ORBIT_RATE : r);

    g_gate[i]  = (i % SR_JUMP_GRID == 0) ? 1.0f : hash01(seed ^ 0xC2B2AE35u);
    g_gate2[i] = hash01(seed ^ 0x165667B1u);
  }
}

static const SrSlots g_slots = {g_base, g_rate, g_gate, g_gate2};

static const SrNormCtx g_ctx = {&g_slots, sr_lengths, SR_LENGTH_COUNT, SR_JUMP_GRID, SR_HOLD_MAX};

static void gen_normalisation(void)
{
  printf("// The correction's own axes. The MOD one spans the whole knob with\n");
  printf("// both endpoints on a bin, so the runtime cannot ask for a setting\n");
  printf("// the table was not generated for.\n");
  printf("#define SR_NORM_BINS %d\n", SR_NORM_BINS);
  printf("#define SR_MOD_BINS %d\n", SR_MOD_BINS);
  printf("#define SR_HOLD_MAX %.4ff\n", SR_HOLD_MAX);
  printf("#define SR_LENGTH_COUNT %d\n\n", SR_LENGTH_COUNT);

  printf("static const uint8_t sr_lengths[SR_LENGTH_COUNT] = {");
  for (int i = 0; i < SR_LENGTH_COUNT; i++)
    printf("%s%d", i ? ", " : "", sr_lengths[i]);
  printf("};\n\n");

  static float centre[SR_MOD_BINS][SR_NORM_BINS];

  for (int mi = 0; mi < SR_MOD_BINS; mi++)
  {
    float mod = -1.0f + 2.0f * (float) mi / (float) (SR_MOD_BINS - 1);
    for (int b = 0; b < SR_NORM_BINS; b++)
    {
      centre[mi][b] = sr_norm_centre(&g_ctx, mod, (float) b / (float) SR_NORM_BINS);
    }
  }

  // The centring constant on its own, which is the half a channel cannot work
  // out for itself: it is an average across every pattern length, and a channel
  // knows only its own. 8 KB, against 90 KB for one length-indexed table - see
  // docs/plans/stepped-random-modes.md.
  printf("// The centring constant, indexed [MOD bin][orbit bin]. Length-\n");
  printf("// independent, which is what makes the cycle boundary seamless: every\n");
  printf("// length lands on this same value at phase 0.\n");
  printf("static const float sr_centre_table[SR_MOD_BINS][SR_NORM_BINS] = {\n");
  for (int mi = 0; mi < SR_MOD_BINS; mi++)
  {
    printf("    {");
    for (int b = 0; b < SR_NORM_BINS; b++)
      printf("%s%.6ff", b ? "," : "", centre[mi][b]);
    printf("},\n");
  }
  printf("};\n\n");
}

int main(void)
{
  build_slots();

  printf("// Generated by tools/gen_sr_table.c - do not edit by hand.\n");
  printf("// Per-slot data for stepped_random(): every source of randomness in the\n");
  printf("// pattern, resolved at build time so the hot path is pure table lookups.\n");
  printf("#ifndef INC_LIB_STEPPED_RANDOM_TABLE_H_\n#define INC_LIB_STEPPED_RANDOM_TABLE_H_\n\n");
  printf("#include <stdint.h>\n\n");
  printf("#define SR_MAX_LENGTH %d\n", SR_MAX_LENGTH);
  printf("#define SR_MAX_ORBIT_RATE %d\n", SR_MAX_ORBIT_RATE);
  printf("#define SR_JUMP_GRID %d\n\n", SR_JUMP_GRID);

  printf("// Where each slot's value starts on its orbit.\n");
  printf("static const float sr_slot_base[SR_MAX_LENGTH] = {\n");
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    if (i % 6 == 0)
      printf("   ");
    printf(" %.8ff,", g_base[i]);
    if (i % 6 == 5 || i == SR_MAX_LENGTH - 1)
      printf("\n");
  }
  printf("};\n\n");

  printf("// Turns each slot's value makes across one full sweep of the shape knob.\n");
  printf("// Integer, so the morph is periodic and the knob wraps seamlessly.\n");
  printf("static const uint8_t sr_slot_rate[SR_MAX_LENGTH] = {\n");
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    if (i % 16 == 0)
      printf("   ");
    printf(" %d,", g_rate[i]);
    if (i % 16 == 15 || i == SR_MAX_LENGTH - 1)
      printf("\n");
  }
  printf("};\n\n");

  printf("// A slot takes a new value when its gate is at or above the density\n");
  printf("// threshold, and crossfades toward the previous step's value as the\n");
  printf("// threshold passes it. Every %dth slot is pinned to 1.0f so it always\n", SR_JUMP_GRID);
  printf("// takes its own value: that bounds the look-back to O(1), keeps slot 0\n");
  printf("// a jump so the loop point always closes on the same value, and stops\n");
  printf("// ties from flattening a pattern.\n");
  printf("static const float sr_slot_gate[SR_MAX_LENGTH] = {\n");
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    if (i % 6 == 0)
      printf("   ");
    printf(" %.8ff,", g_gate[i]);
    if (i % 6 == 5 || i == SR_MAX_LENGTH - 1)
      printf("\n");
  }
  printf("};\n\n");

  printf("// How fast MOD rotates each slot's gate. Without this the tied set only\n");
  printf("// ever grew - once a step tied it stayed tied - so MOD could thin one\n");
  printf("// pattern but never reach a different one.\n");
  printf("static const float sr_slot_gate2[SR_MAX_LENGTH] = {\n");
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    if (i % 6 == 0)
      printf("   ");
    printf(" %.8ff,", g_gate2[i]);
    if (i % 6 == 5 || i == SR_MAX_LENGTH - 1)
      printf("\n");
  }
  printf("};\n\n");

  gen_normalisation();

  printf("#endif /* INC_LIB_STEPPED_RANDOM_TABLE_H_ */\n");
  return 0;
}
