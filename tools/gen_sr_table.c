// Generates the stepped_random slot table. Run once; output is pasted into
// Core/Inc/Lib/stepped_random_table.h.
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define SR_MAX_LENGTH 64
#define SR_JUMP_GRID 4
#define SR_MAX_ORBIT_RATE 4

static uint32_t hash_u32(uint32_t x)
{
  x ^= x >> 16;
  x *= 0x7feb352d;
  x ^= x >> 15;
  x *= 0x846ca68b;
  x ^= x >> 16;
  return x;
}
static float hash01(uint32_t x) { return (hash_u32(x) & 0x00FFFFFF) * (1.0f / 16777216.0f); }

// ---------------------------------------------------------------------------
// Normalisation table.
//
// With only a handful of steps in a cycle, the slot orbits can all cross at
// once, and the pattern collapses toward flat for a stretch of the shape
// sweep. Measured: at length 2 that is 18% of the sweep, in runs 15% wide -
// wide enough to turn a chunk of the knob into dead travel.
//
// So each (length, hold probability, morph) gets a precomputed affine
// correction, applied as out = value * gain + offset. It is deliberately
// gentle: patterns that already span SR_NORM_TARGET are left completely alone
// (gain 1.0), so the natural variation between calm and busy settings survives
// - only genuinely collapsed patterns are lifted, and never by more than
// SR_NORM_MAX_GAIN.
//
// Span does not depend on the hold/smoothness setting: the eased curve passes
// exactly through each slot value, so one table serves all the stepped modes.
// It does depend on the hold *probability*, which is why that is an axis:
// MOD drives it, and a table generated at one probability drifts out of
// correction as soon as the knob moves off it.
//
// The probability axis is a smoothing, not an exact fit. The true span steps
// discretely as the probability crosses each slot's gate, so the bins are
// interpolated between - which is fine, because the correction is gentle and
// changes smoothly with the knob rather than jumping.
// ---------------------------------------------------------------------------

#define SR_NORM_BINS 128
#define SR_NORM_TARGET 1.3f
#define SR_NORM_MAX_GAIN 10.0f

// The probability axis: bins spanning [0, SR_HOLD_MAX]. The short-pattern fade
// is not applied here - the runtime owns it, so the table covers the whole
// range at every length and the fade simply picks a lower point on it.
#define SR_PROB_BINS 8
#define SR_HOLD_MAX 0.85f

static const int sr_lengths[] = {3, 4, 5, 6, 8, 12, 16, 24, 32, 48, 64};
#define SR_LENGTH_COUNT ((int) (sizeof(sr_lengths) / sizeof(sr_lengths[0])))

static float fclampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static float slot_base(int i) { return hash01(hash_u32(((uint32_t) i * 0x9E3779B9u + 0x6D2B79F5u))); }

static int slot_rate(int i)
{
  int r = 1 + (int) (hash01(hash_u32(((uint32_t) i * 0x9E3779B9u + 0x6D2B79F5u)) ^ 0x85EBCA6Bu) * (float) SR_MAX_ORBIT_RATE);
  return r > SR_MAX_ORBIT_RATE ? SR_MAX_ORBIT_RATE : r;
}

static float slot_gate(int i)
{
  return (i % SR_JUMP_GRID == 0) ? 1.0f : hash01(hash_u32(((uint32_t) i * 0x9E3779B9u + 0x6D2B79F5u)) ^ 0xC2B2AE35u);
}

static float tri(float x)
{
  float f = x - (float) (int) x;
  if (f < 0.0f)
    f += 1.0f;
  return 4.0f * fabsf(f - 0.5f) - 1.0f;
}

static float slot_value(int slot, float morph) { return tri(slot_base(slot) + morph * (float) slot_rate(slot)); }

static int source_slot(int slot, float hold_probability)
{
  while (slot_gate(slot) < hold_probability)
  {
    slot--;
  }
  return slot;
}

static void gen_normalisation(void)
{
  printf("// Affine correction keeping short patterns from collapsing to flat.\n");
  printf("// Indexed [length][hold probability bin][morph bin]; apply as\n");
  printf("// out = value * gain + offset, interpolating both bin axes.\n");
  printf("//\n");
  printf("// SR_HOLD_MAX is the top of the probability axis, so the runtime cannot\n");
  printf("// ask for a probability the table was never generated for.\n");
  printf("#define SR_NORM_BINS %d\n", SR_NORM_BINS);
  printf("#define SR_PROB_BINS %d\n", SR_PROB_BINS);
  printf("#define SR_HOLD_MAX %.4ff\n", SR_HOLD_MAX);
  printf("#define SR_LENGTH_COUNT %d\n\n", SR_LENGTH_COUNT);

  printf("static const uint8_t sr_lengths[SR_LENGTH_COUNT] = {");
  for (int i = 0; i < SR_LENGTH_COUNT; i++)
    printf("%s%d", i ? ", " : "", sr_lengths[i]);
  printf("};\n\n");

  static float gain[SR_LENGTH_COUNT][SR_PROB_BINS][SR_NORM_BINS], offset[SR_LENGTH_COUNT][SR_PROB_BINS][SR_NORM_BINS];

  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    int length = sr_lengths[li];

    for (int pi = 0; pi < SR_PROB_BINS; pi++)
    {
      float hold_probability = SR_HOLD_MAX * (float) pi / (float) (SR_PROB_BINS - 1);

      for (int b = 0; b < SR_NORM_BINS; b++)
      {
        float morph = (float) b / (float) SR_NORM_BINS;
        float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < length; i++)
        {
          float v = slot_value(source_slot(i, hold_probability), morph);
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
        float anchor = slot_value(source_slot(0, hold_probability), morph);

        float g = (span < 1e-6f) ? SR_NORM_MAX_GAIN : fclampf(SR_NORM_TARGET / span, 1.0f, SR_NORM_MAX_GAIN);

        // Cap the gain so the anchored result cannot leave [-1,1]. Both limits
        // are >= 1 whenever lo/hi are within [-1,1], so this never shrinks the
        // signal, it only declines to expand it as far as we wanted.
        if (hi > anchor)
        {
          float limit = (1.0f - anchor) / (hi - anchor);
          if (limit < g)
            g = limit;
        }
        if (lo < anchor)
        {
          float limit = (1.0f + anchor) / (anchor - lo);
          if (limit < g)
            g = limit;
        }
        if (g < 1.0f)
          g = 1.0f;

        // out = anchor + (v - anchor) * g
        gain[li][pi][b]   = g;
        offset[li][pi][b] = anchor * (1.0f - g);
      }
    }
  }

  const char* names[2]                                = {"sr_norm_gain", "sr_norm_offset"};
  const float(*tables[2])[SR_PROB_BINS][SR_NORM_BINS] = {gain, offset};

  for (int t = 0; t < 2; t++)
  {
    printf("static const float %s[SR_LENGTH_COUNT][SR_PROB_BINS][SR_NORM_BINS] = {\n", names[t]);
    for (int li = 0; li < SR_LENGTH_COUNT; li++)
    {
      printf("    {\n");
      for (int pi = 0; pi < SR_PROB_BINS; pi++)
      {
        printf("        {");
        for (int b = 0; b < SR_NORM_BINS; b++)
          printf("%s%.6ff", b ? "," : "", tables[t][li][pi][b]);
        printf("},\n");
      }
      printf("    },\n");
    }
    printf("};\n\n");
  }
}

int main(void)
{
  printf("// Generated by tools/gen_sr_table.c - do not edit by hand.\n");
  printf("// Per-slot data for stepped_random(): every source of randomness in the\n");
  printf("// pattern, resolved at build time so the hot path is pure table lookups.\n");
  printf("#ifndef INC_LIB_STEPPED_RANDOM_TABLE_H_\n#define INC_LIB_STEPPED_RANDOM_TABLE_H_\n\n");
  printf("#include <stdint.h>\n\n");
  printf("#define SR_MAX_LENGTH %d\n", SR_MAX_LENGTH);
  printf("#define SR_MAX_ORBIT_RATE %d\n\n", SR_MAX_ORBIT_RATE);

  // Orbit start phase.
  printf("// Where each slot's value starts on its orbit.\n");
  printf("static const float sr_slot_base[SR_MAX_LENGTH] = {\n");
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    if (i % 6 == 0)
      printf("   ");
    printf(" %.8ff,", hash01(hash_u32(((uint32_t) i * 0x9E3779B9u + 0x6D2B79F5u))));
    if (i % 6 == 5 || i == SR_MAX_LENGTH - 1)
      printf("\n");
  }
  printf("};\n\n");

  // Orbit rate: how many turns this slot's value makes across one full sweep
  // of the shape knob. Integer so the morph is periodic in shape.
  printf("// Turns each slot's value makes across one full sweep of the shape knob.\n");
  printf("// Integer, so the morph is periodic and the knob wraps seamlessly.\n");
  printf("static const uint8_t sr_slot_rate[SR_MAX_LENGTH] = {\n");
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    if (i % 16 == 0)
      printf("   ");
    int rate = 1 + (int) (hash01(hash_u32(((uint32_t) i * 0x9E3779B9u + 0x6D2B79F5u)) ^ 0x85EBCA6Bu) * (float) SR_MAX_ORBIT_RATE);
    if (rate > SR_MAX_ORBIT_RATE)
      rate = SR_MAX_ORBIT_RATE;
    printf(" %d,", rate);
    if (i % 16 == 15 || i == SR_MAX_LENGTH - 1)
      printf("\n");
  }
  printf("};\n\n");

  // Jump gate. A slot takes a new value when its gate is >= the hold
  // probability. Every SR_JUMP_GRID-th slot is pinned to 1.0 so it always
  // jumps, which bounds how far a run of holds can chain (keeping the
  // look-back O(1)), guarantees slot 0 is a jump so the loop point always
  // closes on the same value, and stops holds from flattening a pattern.
  printf("// A slot takes a new value when its gate is >= the hold probability.\n");
  printf("// Every %dth slot is pinned to 1.0f so it always jumps: that bounds the\n", SR_JUMP_GRID);
  printf("// hold look-back to O(1), keeps slot 0 a jump so the loop point always\n");
  printf("// closes on the same value, and stops holds from flattening a pattern.\n");
  printf("static const float sr_slot_gate[SR_MAX_LENGTH] = {\n");
  for (int i = 0; i < SR_MAX_LENGTH; i++)
  {
    if (i % 6 == 0)
      printf("   ");
    float gate = (i % SR_JUMP_GRID == 0) ? 1.0f : hash01(hash_u32(((uint32_t) i * 0x9E3779B9u + 0x6D2B79F5u)) ^ 0xC2B2AE35u);
    printf(" %.8ff,", gate);
    if (i % 6 == 5 || i == SR_MAX_LENGTH - 1)
      printf("\n");
  }
  printf("};\n\n");

  gen_normalisation();

  printf("#endif /* INC_LIB_STEPPED_RANDOM_TABLE_H_ */\n");
  return 0;
}
