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
// So each (length, MOD, morph) gets a precomputed affine correction, applied as
// out = value * gain + offset. What it aims for and why lives in
// stepped_random_norm.h, which is where it is computed; this file only decides
// where to sample it and how to survive the runtime interpolating between those
// points.
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

// Sub-samples either side of a MOD bin, so its gain serves the whole bin.
#define SR_GAIN_SUBS 3

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
  printf("// Affine correction keeping patterns from collapsing to flat.\n");
  printf("// Indexed [length][MOD bin][morph bin]; apply as\n");
  printf("// out = value * gain + offset, interpolating both bin axes.\n");
  printf("//\n");
  printf("// The MOD axis spans the whole knob with both endpoints on a bin, so\n");
  printf("// the runtime cannot ask for a setting the table was not generated for.\n");
  printf("#define SR_NORM_BINS %d\n", SR_NORM_BINS);
  printf("#define SR_MOD_BINS %d\n", SR_MOD_BINS);
  printf("#define SR_HOLD_MAX %.4ff\n", SR_HOLD_MAX);
  printf("#define SR_LENGTH_COUNT %d\n\n", SR_LENGTH_COUNT);

  printf("static const uint8_t sr_lengths[SR_LENGTH_COUNT] = {");
  for (int i = 0; i < SR_LENGTH_COUNT; i++)
    printf("%s%d", i ? ", " : "", sr_lengths[i]);
  printf("};\n\n");

  static float gain[SR_LENGTH_COUNT][SR_MOD_BINS][SR_NORM_BINS], offset[SR_LENGTH_COUNT][SR_MOD_BINS][SR_NORM_BINS];
  static float centre[SR_MOD_BINS][SR_NORM_BINS];

  // First pass: the constant that centres the pattern at each (MOD, morph). It
  // is shared by every length, so it has to be worked out before any of them.
  for (int mi = 0; mi < SR_MOD_BINS; mi++)
  {
    float mod = -1.0f + 2.0f * (float) mi / (float) (SR_MOD_BINS - 1);
    for (int b = 0; b < SR_NORM_BINS; b++)
    {
      centre[mi][b] = sr_norm_centre(&g_ctx, mod, (float) b / (float) SR_NORM_BINS);
    }
  }

  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    int length = sr_lengths[li];

    for (int mi = 0; mi < SR_MOD_BINS; mi++)
    {
      float mod = -1.0f + 2.0f * (float) mi / (float) (SR_MOD_BINS - 1);

      for (int b = 0; b < SR_NORM_BINS; b++)
      {
        float morph = (float) b / (float) SR_NORM_BINS;
        float c     = centre[mi][b];
        SrExtent e  = sr_extent_of(&g_ctx, length, mod, morph);
        float g     = sr_gain_for(&e, c);

        // The gain has to serve the whole neighbourhood of its bin, not just
        // the point it was sampled at: the runtime interpolates between MOD
        // bins and the span dips *between* them as the ties rearrange, so a
        // gain fitted to the bin centres leaves those dips uncorrected -
        // measured at 1.3% of the parameter space under the flat floor.
        //
        // Only the floor is carried across the neighbourhood. Carrying the full
        // target there gave every bin the largest gain any of its neighbours
        // wanted, which is a bias upward everywhere and undoes the point of
        // normalising the level at all.
        //
        // The neighbour is measured with the bin's own anchor, not its own:
        // the anchor is what the offset below is built from, and a gain worked
        // out against a different one would not stay inside the rails when the
        // two are used together.
        float half_bin = 1.0f / (float) (SR_MOD_BINS - 1);
        for (int sub = -SR_GAIN_SUBS; sub <= SR_GAIN_SUBS; sub++)
        {
          if (sub == 0)
            continue;
          float near_mod = fclamp(mod + half_bin * (float) sub / (float) SR_GAIN_SUBS, -1.0f, 1.0f);
          SrExtent ne    = sr_extent_of(&g_ctx, length, near_mod, morph);
          ne.anchor      = e.anchor;
          float ng       = sr_gain_floor(&ne, c);
          if (ng > g)
            g = ng;
        }

        // Both c and the anchor are the same at every length, so the value at
        // the cycle boundary is too - which is what lets the engine switch
        // pattern length on the wrap without a step in the signal.
        SrNorm n          = sr_norm_affine(c, e.anchor, g);
        gain[li][mi][b]   = n.gain;
        offset[li][mi][b] = n.offset;
      }
    }
  }

  const char* names[2]                               = {"sr_norm_gain", "sr_norm_offset"};
  const float(*tables[2])[SR_MOD_BINS][SR_NORM_BINS] = {gain, offset};

  for (int t = 0; t < 2; t++)
  {
    printf("static const float %s[SR_LENGTH_COUNT][SR_MOD_BINS][SR_NORM_BINS] = {\n", names[t]);
    for (int li = 0; li < SR_LENGTH_COUNT; li++)
    {
      printf("    {\n");
      for (int mi = 0; mi < SR_MOD_BINS; mi++)
      {
        printf("        {");
        for (int b = 0; b < SR_NORM_BINS; b++)
          printf("%s%.6ff", b ? "," : "", tables[t][li][mi][b]);
        printf("},\n");
      }
      printf("    },\n");
    }
    printf("};\n\n");
  }
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
