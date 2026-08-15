// Generates the stepped_random slot and normalisation tables. Run once; output
// is pasted into Core/Inc/Lib/stepped_random_table.h. See the `sr-table` recipe
// in the Justfile.
//
// The value path itself is not repeated here - it comes from
// stepped_random_pattern.h, the same header the runtime uses. It used to be
// copied, and a copy of a pattern generator is a correction quietly aimed at a
// pattern the module no longer plays.
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
// out = value * gain + offset. It is deliberately gentle: patterns that already
// span SR_NORM_TARGET are left completely alone (gain 1.0), so the natural
// variation between calm and busy settings survives - only genuinely collapsed
// patterns are lifted, and never by more than SR_NORM_MAX_GAIN.
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
// fine, because the correction is gentle and changes smoothly with the knob
// rather than jumping.
// ---------------------------------------------------------------------------

#define SR_NORM_BINS 128
#define SR_NORM_TARGET 1.3f
#define SR_NORM_MAX_GAIN 40.0f

// How far the correction may pull its anchor toward centre, and how finely that
// is searched. See gain_for().
#define SR_PULL_MAX 1.0f

// Sub-samples either side of a MOD bin, so its gain serves the whole bin.
#define SR_GAIN_SUBS 3

// The MOD axis: bins spanning the whole knob, endpoints included.
#define SR_MOD_BINS 16
#define SR_HOLD_MAX 0.85f

static const int sr_lengths[] = {3, 4, 5, 6, 8, 12, 16, 24, 32, 48, 64};
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

// The gain that lifts a pattern toward SR_NORM_TARGET without letting the
// corrected result leave [-1,1].
//
// `pull` moves the anchor toward zero. Without it a pattern bunched hard
// against a rail cannot be expanded at all: the anchor is the rail, so the
// headroom above it is nil and the cap pins the gain at 1. That is a real dead
// spot - measured at three and four steps, a cycle of four values sitting
// within 0.07 of each other up at +0.96 - and it is why this exists. Pulling
// the anchor down makes room to expand into.
//
// The pull cannot be chosen per length, because the anchor is what every length
// has to agree on at the cycle boundary. So it is chosen once per (MOD, morph)
// for whichever length needs it most, and every length uses it.
static float gain_for(float span, float lo, float hi, float anchor, float pull)
{
  float c = anchor * (1.0f - pull);
  float g = (span < 1e-6f) ? SR_NORM_MAX_GAIN : fclamp(SR_NORM_TARGET / span, 1.0f, SR_NORM_MAX_GAIN);

  // Both limits are >= 1 whenever lo/hi are within [-1,1] and the pull has not
  // moved the anchor past them, so this never shrinks the signal - it only
  // declines to expand it as far as we wanted.
  if (hi > anchor)
  {
    float limit = (1.0f - c) / (hi - anchor);
    if (limit < g)
      g = limit;
  }
  if (lo < anchor)
  {
    float limit = (1.0f + c) / (anchor - lo);
    if (limit < g)
      g = limit;
  }
  return (g < 1.0f) ? 1.0f : g;
}

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
  static float pull[SR_MOD_BINS][SR_NORM_BINS];

  // First pass: how far the anchor has to be pulled toward centre at each
  // (MOD, morph), judged by the length that comes off worst there. Anything
  // more than the minimum is avoided - the pull moves the whole cycle's DC, so
  // it is a correction, not a feature.
  for (int mi = 0; mi < SR_MOD_BINS; mi++)
  {
    float mod = -1.0f + 2.0f * (float) mi / (float) (SR_MOD_BINS - 1);

    for (int b = 0; b < SR_NORM_BINS; b++)
    {
      float morph  = (float) b / (float) SR_NORM_BINS;
      float needed = 0.0f;
      float half   = 1.0f / (float) (SR_MOD_BINS - 1);

      for (int li = 0; li < SR_LENGTH_COUNT; li++)
      {
        int length = sr_lengths[li];
        for (int sub = -SR_GAIN_SUBS; sub <= SR_GAIN_SUBS; sub++)
        {
          float near = fclamp(mod + half * (float) sub / (float) SR_GAIN_SUBS, -1.0f, 1.0f);
          SrMorph m  = sr_morph_at(morph, near, length, SR_HOLD_MAX);
          float lo = 1e9f, hi = -1e9f;
          for (int i = 0; i < length; i++)
          {
            float v = sr_step_value(i, &m, &g_slots, SR_JUMP_GRID);
            if (v < lo)
              lo = v;
            if (v > hi)
              hi = v;
          }
          float anchor = sr_step_value(0, &m, &g_slots, SR_JUMP_GRID);
          float span   = hi - lo;
          if (span < 1e-6f || fabsf(anchor) < 1e-6f)
          {
            continue;
          }

          // The gain this length wants, and the window of anchors that lets it
          // have it without the result leaving [-1,1]. Solved rather than
          // searched: a discrete search makes the pull jump between bins, and the
          // runtime interpolates between bins, so two neighbouring bins carrying
          // corrections built on different anchors blend into one that is right
          // for neither.
          float want = fclamp(SR_NORM_TARGET / span, 1.0f, SR_NORM_MAX_GAIN);
          float c    = anchor;
          if (hi > anchor)
          {
            float upper = 1.0f - want * (hi - anchor);
            if (c > upper)
              c = upper;
          }
          if (lo < anchor)
          {
            float lower = -1.0f + want * (anchor - lo);
            if (c < lower)
              c = lower;
          }

          // Only ever toward zero, never past it or away from the anchor.
          if (anchor > 0.0f)
            c = fclamp(c, 0.0f, anchor);
          else
            c = fclamp(c, anchor, 0.0f);

          float p = 1.0f - c / anchor;
          if (p > needed)
            needed = p;
        }
      }

      // A max of continuous functions is continuous, so the field is smooth
      // enough for the runtime to interpolate across.
      pull[mi][b] = fclamp(needed, 0.0f, SR_PULL_MAX);
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
        SrMorph m   = sr_morph_at(morph, mod, length, SR_HOLD_MAX);
        float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < length; i++)
        {
          float v = sr_step_value(i, &m, &g_slots, SR_JUMP_GRID);
          if (v < lo)
            lo = v;
          if (v > hi)
            hi = v;
        }
        float span = hi - lo;

        // Anchor the correction on slot 0 rather than on the pattern's
        // midpoint. Slot 0's raw value does not depend on the length, so
        // anchoring there makes the corrected output at the cycle boundary
        // identical at every length - which is what lets the engine switch
        // pattern length on the wrap without a step in the signal. Anchoring
        // on the midpoint instead left a 0.34 jump, because the midpoint moves
        // with the length.
        float anchor = sr_step_value(0, &m, &g_slots, SR_JUMP_GRID);

        // The gain has to serve the whole neighbourhood of its bin, not just
        // the point it was sampled at. The runtime interpolates between bins,
        // and the pattern's span dips *between* MOD bins - the ties rearrange
        // as MOD turns - so a gain fitted to the bin centres leaves the dips
        // uncorrected. Measured: 1.3% of the parameter space under the flat
        // floor, which no amount of extra bins removed cheaply.
        //
        // So sample across the bin and take the gain the worst point needs.
        // Neighbouring bins over-correct slightly as a result, which the anchor
        // caps keep in range and the ear reads as the correction being gentle.
        float g        = gain_for(span, lo, hi, anchor, pull[mi][b]);
        float half_bin = 1.0f / (float) (SR_MOD_BINS - 1);
        for (int sub = -SR_GAIN_SUBS; sub <= SR_GAIN_SUBS; sub++)
        {
          if (sub == 0)
            continue;
          float near_mod = fclamp(mod + half_bin * (float) sub / (float) SR_GAIN_SUBS, -1.0f, 1.0f);
          SrMorph nm     = sr_morph_at(morph, near_mod, length, SR_HOLD_MAX);
          float nlo = 1e9f, nhi = -1e9f;
          for (int i = 0; i < length; i++)
          {
            float v = sr_step_value(i, &nm, &g_slots, SR_JUMP_GRID);
            if (v < nlo)
              nlo = v;
            if (v > nhi)
              nhi = v;
          }
          float ng = gain_for(nhi - nlo, nlo, nhi, anchor, pull[mi][b]);
          if (ng > g)
            g = ng;
        }

        // The span lever rides on the same affine: it only ever shrinks, so it
        // cannot push the result out of range, and it is free at runtime
        // because the table already carries a gain. Held off the floor by what
        // the correction actually achieved here rather than by a globally
        // conservative depth - where there is headroom the lever gets all of
        // it, where there is not it gets what there is.
        float s     = sr_span_drive(morph);
        float reach = span * g;
        if (reach > 1e-6f && reach * s < SR_SPAN_FLOOR)
        {
          s = fclamp(SR_SPAN_FLOOR / reach, 0.0f, 1.0f);
        }
        g *= s;

        // out = c + (v - anchor) * g, where c is the anchor pulled toward
        // centre. c depends only on MOD and morph, never on the length, so the
        // value at the cycle boundary is still the same at every length.
        float c           = anchor * (1.0f - pull[mi][b]);
        gain[li][mi][b]   = g;
        offset[li][mi][b] = c - anchor * g;
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
