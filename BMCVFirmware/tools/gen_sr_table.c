// Generates the stepped_random slot table. Run once; output is pasted into
// Core/Inc/Lib/stepped_random_table.h.
#include <stdint.h>
#include <math.h>
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
// So each (length, morph) gets a precomputed affine correction, applied as
// out = value * gain + offset. It is deliberately gentle: patterns that
// already span SR_NORM_TARGET are left completely alone (gain 1.0), so the
// natural variation between calm and busy settings survives - only genuinely
// collapsed patterns are lifted, and never by more than SR_NORM_MAX_GAIN.
//
// Span does not depend on the hold/smoothness setting: the eased curve passes
// exactly through each slot value, so one table serves all the stepped modes.
// ---------------------------------------------------------------------------

#define SR_NORM_BINS 128
#define SR_NORM_TARGET 1.3f
#define SR_NORM_MAX_GAIN 10.0f
#define SR_HOLD_PROBABILITY 0.30f
#define SR_HOLD_FADE_IN_STEPS 6.0f

static const int sr_lengths[] = {3, 4, 5, 6, 7, 8, 12, 16, 24, 32, 48, 64};
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
  printf("// Indexed [length][morph bin]; apply as out = value * gain + offset.\n");
  printf("#define SR_NORM_BINS %d\n", SR_NORM_BINS);
  printf("#define SR_LENGTH_COUNT %d\n\n", SR_LENGTH_COUNT);

  printf("static const uint8_t sr_lengths[SR_LENGTH_COUNT] = {");
  for (int i = 0; i < SR_LENGTH_COUNT; i++)
    printf("%s%d", i ? ", " : "", sr_lengths[i]);
  printf("};\n\n");

  // Canonical CH_PARAM_MOD value for each length, so the encoder can step
  // straight from one division to the next via val_neighbour() instead of
  // grinding through ~22 detents of dead travel. Ascending, as val_neighbour
  // requires. Feeding one of these back through sr_length_index_from_mod()
  // returns exactly its own index.
  printf("// Canonical CH_PARAM_MOD value per length, for val_neighbour() stepping.\n");
  printf("static const int16_t sr_length_param[SR_LENGTH_COUNT] = {");
  for (int i = 0; i < SR_LENGTH_COUNT; i++)
  {
    float pos = (float) i / (float) (SR_LENGTH_COUNT - 1);
    long p    = lroundf((2.0f * pos - 1.0f) * 32767.0f);
    if (p < -32768) p = -32768;
    if (p > 32767) p = 32767;
    printf("%s%ld", i ? ", " : "", p);
  }
  printf("};\n\n");

  float gain[SR_LENGTH_COUNT][SR_NORM_BINS], offset[SR_LENGTH_COUNT][SR_NORM_BINS];

  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    int length             = sr_lengths[li];
    float hold_probability = SR_HOLD_PROBABILITY * fclampf((float) (length - 2) / SR_HOLD_FADE_IN_STEPS, 0.0f, 1.0f);

    for (int b = 0; b < SR_NORM_BINS; b++)
    {
      float morph = (float) b / (float) SR_NORM_BINS;
      float lo = 1e9f, hi = -1e9f;
      for (int i = 0; i < length; i++)
      {
        float v = slot_value(source_slot(i, hold_probability), morph);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
      float span = hi - lo;

      // Anchor the correction on slot 0 rather than on the pattern's midpoint.
      // Slot 0's raw value does not depend on the length, so anchoring there
      // makes the corrected output at the cycle boundary identical at every
      // length - which is what lets the engine switch pattern length on the
      // wrap without a step in the signal. Anchoring on the midpoint instead
      // left a 0.34 jump, because the midpoint moves with the length.
      float anchor = slot_value(source_slot(0, hold_probability), morph);

      float g = (span < 1e-6f) ? SR_NORM_MAX_GAIN : fclampf(SR_NORM_TARGET / span, 1.0f, SR_NORM_MAX_GAIN);

      // Cap the gain so the anchored result cannot leave [-1,1]. Both limits
      // are >= 1 whenever lo/hi are within [-1,1], so this never shrinks the
      // signal, it only declines to expand it as far as we wanted.
      if (hi > anchor)
      {
        float limit = (1.0f - anchor) / (hi - anchor);
        if (limit < g) g = limit;
      }
      if (lo < anchor)
      {
        float limit = (1.0f + anchor) / (anchor - lo);
        if (limit < g) g = limit;
      }
      if (g < 1.0f)
        g = 1.0f;

      // out = anchor + (v - anchor) * g
      gain[li][b]   = g;
      offset[li][b] = anchor * (1.0f - g);
    }
  }

  printf("static const float sr_norm_gain[SR_LENGTH_COUNT][SR_NORM_BINS] = {\n");
  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    printf("    {");
    for (int b = 0; b < SR_NORM_BINS; b++)
      printf("%s%.6ff", b ? "," : "", gain[li][b]);
    printf("},\n");
  }
  printf("};\n\n");

  printf("static const float sr_norm_offset[SR_LENGTH_COUNT][SR_NORM_BINS] = {\n");
  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    printf("    {");
    for (int b = 0; b < SR_NORM_BINS; b++)
      printf("%s%.6ff", b ? "," : "", offset[li][b]);
    printf("},\n");
  }
  printf("};\n\n");
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
