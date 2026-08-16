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
// out = value * gain + offset. It does two jobs: it holds the peak-to-peak near
// SR_NORM_TARGET, and it puts the pattern's own centre near zero.
//
// Both used to be one-sided - lift a collapsed pattern, never shrink a wide
// one, and leave the centre wherever the pattern happened to sit. Measured on
// the shipping build, the result was a peak-to-peak that varied 2.3x between
// settings of the same knob, and a centre that walked 0.76 of the 2.0 range
// along one sweep of SHP. Both read as the shape flattening and shifting as the
// knob turns, which is what AMP and OFFSET are for; SHP and MOD are supposed to
// be steering character. Neither correction touches character: every character
// measurement in tools/sr_explore/ is invariant under an affine, and all of
// them come out unchanged to two decimals.
//
// Neither is absolute either. SR_NORM_EXP leaves the natural loud/quiet
// ordering audible while compressing its range, and the centring is one
// constant shared by all the lengths rather than a fit to each.
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
#define SR_NORM_TARGET 1.5f
#define SR_NORM_MAX_GAIN 40.0f
// A clamp rather than a setting: at SR_NORM_EXP 0.7 the smallest gain the
// normalisation ever asks for is (1.5/2.0)^0.7 = 0.82, so this only catches a
// re-tuning that went somewhere absurd.
#define SR_NORM_MIN_GAIN 0.4f

// How completely the peak-to-peak is normalised. 1 pins every setting to
// SR_NORM_TARGET; 0 leaves them all alone. At 0.7 a pattern whose natural span
// is 0.7 comes out at 1.20 and one at 2.0 comes out at 1.64 - the ordering
// survives, the 2.9x range does not.
#define SR_NORM_EXP 0.7f

// The span the correction guarantees even where it is otherwise shrinking, so
// the two-sided normalisation cannot make a dead setting.
#define SR_NORM_FLOOR 0.9f

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

// The gain that carries a pattern's peak-to-peak toward `target`, and what the
// rails leave of it once the correction's constant `c` is added.
//
// The pattern is scaled about slot 0, so the caps are asymmetric: how far the
// top of the pattern is above slot 0 decides how much expansion the top rail
// allows, and the bottom likewise. Where slot 0 sits relative to the pattern is
// therefore what the constant is really buying - it used to be pulled toward
// zero purely to open that headroom up, and centring the pattern does the same
// job as a side effect of doing something musically useful.
static float gain_toward(float target, float expo, float span, float lo, float hi, float anchor, float c)
{
  float g = (span < 1e-6f) ? SR_NORM_MAX_GAIN : fclamp(powf(target / span, expo), SR_NORM_MIN_GAIN, SR_NORM_MAX_GAIN);

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
  return (g < 0.05f) ? 0.05f : g;
}

static float gain_for(float span, float lo, float hi, float anchor, float c)
{
  return gain_toward(SR_NORM_TARGET, SR_NORM_EXP, span, lo, hi, anchor, c);
}

// What a point needs to stay clear of the flat floor, whatever the
// normalisation would otherwise ask for.
static float gain_floor(float span, float lo, float hi, float anchor, float c)
{
  return gain_toward(SR_NORM_FLOOR, 1.0f, span, lo, hi, anchor, c);
}

typedef struct
{
  float lo, hi, centre, anchor;
} Extent;

// One pattern, measured: its extremes, its DC, and the value it starts on.
//
// The DC is weighted by how wide each step is, since MOD skews alternate steps
// long and short and a plain mean of the step values would not be the level the
// ear settles on.
static Extent extent_of(float morph, float mod, int length)
{
  SrMorph m   = sr_morph_at(morph, mod, length, SR_HOLD_MAX);
  Extent e    = {1e9f, -1e9f, 0.0f, 0.0f};
  float swing = sr_swing_amount(mod);
  float dc = 0.0f, wsum = 0.0f;

  for (int i = 0; i < length; i++)
  {
    float v = sr_step_value(i, &m, &g_slots, SR_JUMP_GRID);
    float w = (i & 1) ? 1.0f - swing : 1.0f + swing;
    if (v < e.lo)
      e.lo = v;
    if (v > e.hi)
      e.hi = v;
    dc += v * w;
    wsum += w;
  }

  e.centre = dc / wsum;
  e.anchor = sr_step_value(0, &m, &g_slots, SR_JUMP_GRID);
  return e;
}

// The constant that puts the pattern's centre as near zero as one number can.
//
// One number is all there is. The corrected value at the cycle boundary is
// slot 0's, and that is what every length has to agree on for the engine to be
// able to switch pattern length on the wrap - so a constant fitted per length
// would put a step in the signal exactly where there must not be one. Hence an
// average across the lengths rather than a fit to each: the minimax was tried
// and measured worse (0.54 of residual DC swing against 0.41), because two
// short patterns at the extremes drag it away from where the other nine sit.
//
// Iterated, because the gain the rails allow depends on the constant and the
// constant depends on the gain.
static float centre_for(float morph, float mod)
{
  float c = 0.0f;

  for (int it = 0; it < 3; it++)
  {
    float sum = 0.0f;
    for (int li = 0; li < SR_LENGTH_COUNT; li++)
    {
      Extent e = extent_of(morph, mod, sr_lengths[li]);
      sum += (e.centre - e.anchor) * gain_for(e.hi - e.lo, e.lo, e.hi, e.anchor, c);
    }
    c = -sum / (float) SR_LENGTH_COUNT;
  }
  return fclamp(c, -1.0f, 1.0f);
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
  static float centre[SR_MOD_BINS][SR_NORM_BINS];

  // First pass: the constant that centres the pattern at each (MOD, morph). It
  // is shared by every length, so it has to be worked out before any of them.
  for (int mi = 0; mi < SR_MOD_BINS; mi++)
  {
    float mod = -1.0f + 2.0f * (float) mi / (float) (SR_MOD_BINS - 1);
    for (int b = 0; b < SR_NORM_BINS; b++)
    {
      centre[mi][b] = centre_for((float) b / (float) SR_NORM_BINS, mod);
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
        Extent e    = extent_of(morph, mod, length);

        // Scaled about slot 0, whose raw value does not depend on the length -
        // so with the constant shared too, the corrected value at the cycle
        // boundary is identical at every length, which is what lets the engine
        // switch pattern length on the wrap without a step in the signal.
        // Scaling about the pattern's own midpoint instead - which is what
        // centring each length exactly would need - left a 0.34 jump there,
        // because the midpoint moves with the length.
        //
        // The gain also has to serve the whole neighbourhood of its bin, not
        // just the point it was sampled at: the runtime interpolates between
        // MOD bins and the span dips *between* them as the ties rearrange, and
        // a gain fitted to the bin centres leaves those dips uncorrected -
        // measured at 1.3% of the parameter space under the flat floor. Only
        // the floor is carried across the neighbourhood, though. Carrying the
        // full target there gave every bin the largest gain any of its
        // neighbours wanted, which is a bias upward everywhere and undoes the
        // point of normalising the level at all.
        float g        = gain_for(e.hi - e.lo, e.lo, e.hi, e.anchor, c);
        float half_bin = 1.0f / (float) (SR_MOD_BINS - 1);
        for (int sub = -SR_GAIN_SUBS; sub <= SR_GAIN_SUBS; sub++)
        {
          if (sub == 0)
            continue;
          float near_mod = fclamp(mod + half_bin * (float) sub / (float) SR_GAIN_SUBS, -1.0f, 1.0f);
          Extent ne      = extent_of(morph, near_mod, length);
          float ng       = gain_floor(ne.hi - ne.lo, ne.lo, ne.hi, e.anchor, c);
          if (ng > g)
            g = ng;
        }

        // out = c + (v - anchor) * g. Both c and anchor are the same at every
        // length, so the value at the cycle boundary is too.
        gain[li][mi][b]   = g;
        offset[li][mi][b] = c - e.anchor * g;
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
