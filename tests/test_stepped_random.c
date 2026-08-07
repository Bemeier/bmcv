#include "stepped_random.h"
#include "stepped_random_table.h" // SR_LENGTH_COUNT
#include "testkit.h"
#include <math.h>

static const float HOLDS[] = {SR_HOLD_SMOOTH, SR_HOLD_SEMI, SR_HOLD_HARD};

// Every property below is about the pattern itself, which is what MOD 0 gives:
// even steps, no skew. The MOD cases have their own tests at the end.
static float sr_flat(float phase, float shape, int length_idx, float hold) { return stepped_random(phase, shape, 0.0f, length_idx, hold); }
#define HOLD_COUNT ((int) (sizeof(HOLDS) / sizeof(HOLDS[0])))

// Peak-to-peak of one full cycle at a given setting.
static float cycle_span(float shape, int length_idx, float hold)
{
  float lo = 1e9f, hi = -1e9f;
  for (int i = 0; i < 2048; i++)
  {
    float v = sr_flat((float) i / 2048.0f, shape, length_idx, hold);
    if (v < lo)
      lo = v;
    if (v > hi)
      hi = v;
  }
  return hi - lo;
}

TEST_CASE(output_stays_in_bipolar_range)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.2f)
    {
      for (int li = 0; li < SR_LENGTH_COUNT; li++)
      {
        for (float phase = 0.0f; phase < 1.0f; phase += 0.02f)
        {
          float v = sr_flat(phase, shape, li, HOLDS[h]);
          CHECK(v >= -1.0001f && v <= 1.0001f);
        }
      }
    }
  }
}

TEST_CASE(is_deterministic_for_same_inputs) { CHECK(sr_flat(0.37f, 0.1f, 4, SR_HOLD_SEMI) == sr_flat(0.37f, 0.1f, 4, SR_HOLD_SEMI)); }

// The point of an integer, circularly-read lattice: the waveform closes on
// itself, so the loop point is inaudible and stays put under the PLL.
TEST_CASE(phase_wraps_seamlessly_across_the_loop_point)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.25f)
    {
      for (int li = 0; li < SR_LENGTH_COUNT; li++)
      {
        CHECK_NEAR(sr_flat(0.0f, shape, li, HOLDS[h]), sr_flat(1.0f, shape, li, HOLDS[h]), 1e-4);
      }
    }
  }
}

// Every length starts and ends its cycle on slot 0, whose raw value does not
// depend on the length, and the normalisation is anchored there too. So the
// engine can switch pattern length on the cycle wrap and the signal barely
// moves - which is what makes a scene crossfade of MOD glitch-free. Switching
// mid-cycle instead jumps by up to 1.8 of a 2.0 range.
//
// Not exactly equal: the normalisation gain is interpolated between morph bins,
// leaving a measured worst case of ~0.026 out of 2.0.
TEST_CASE(changing_length_at_the_cycle_boundary_is_seamless)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
    {
      for (int from = 0; from < SR_LENGTH_COUNT; from++)
      {
        for (int to = 0; to < SR_LENGTH_COUNT; to++)
        {
          float leaving  = sr_flat(1.0f, shape, from, HOLDS[h]);
          float entering = sr_flat(0.0f, shape, to, HOLDS[h]);
          CHECK_NEAR(leaving, entering, 0.05);
        }
      }
    }
  }
}

// Continuity everywhere, not only at the loop point: no step boundary may
// produce a jump, however hard the hold setting.
TEST_CASE(curve_is_continuous_at_every_step_boundary)
{
  const int steps = 20000;
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (int li = 0; li < SR_LENGTH_COUNT; li++)
    {
      float prev     = sr_flat(0.0f, 0.3f, li, HOLDS[h]);
      float max_jump = 0.0f;
      for (int i = 1; i <= steps; i++)
      {
        float v    = sr_flat((float) i / (float) steps, 0.3f, li, HOLDS[h]);
        float jump = fabsf(v - prev);
        if (jump > max_jump)
          max_jump = jump;
        prev = v;
      }
      CHECK(max_jump < 0.15f);
    }
  }
}

// "Neighbouring areas of the morph parameter should sound similar" - a small
// turn of the shape knob must deform the pattern, not re-randomise it.
TEST_CASE(morph_changes_gradually_with_the_shape_parameter)
{
  const float ds = 0.01f;
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (float shape = -1.0f; shape < 1.0f; shape += 0.05f)
    {
      float worst = 0.0f;
      for (float phase = 0.0f; phase < 1.0f; phase += 0.01f)
      {
        float d = fabsf(sr_flat(phase, shape, 6, HOLDS[h]) - sr_flat(phase, shape + ds, 6, HOLDS[h]));
        if (d > worst)
          worst = d;
      }
      CHECK(worst < 0.25f);
    }
  }
}

// "No total flat areas": every combination of shape and length must still swing.
TEST_CASE(no_setting_collapses_to_a_flat_output)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    float worst = 2.0f;
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.02f)
    {
      for (int li = 0; li < SR_LENGTH_COUNT; li++)
      {
        float span = cycle_span(shape, li, HOLDS[h]);
        if (span < worst)
          worst = span;
      }
    }
    // Full range is 2.0. Measured worst case across the whole grid is ~0.53,
    // at the shortest pattern where only three values are in play; everything
    // else is >= 0.99. Calm settings are wanted, dead ones are not.
    CHECK(worst > 0.5f);
  }
}

static int direction_changes(int length_idx)
{
  int turns        = 0;
  float prev       = sr_flat(0.0f, 0.25f, length_idx, SR_HOLD_SEMI);
  float prev_slope = 0.0f;
  for (int i = 1; i <= 4096; i++)
  {
    float v     = sr_flat((float) i / 4096.0f, 0.25f, length_idx, SR_HOLD_SEMI);
    float slope = v - prev;
    if (slope * prev_slope < 0.0f)
      turns++;
    if (slope != 0.0f)
      prev_slope = slope;
    prev = v;
  }
  return turns;
}

TEST_CASE(longer_patterns_contain_more_events) { CHECK(direction_changes(SR_LENGTH_COUNT - 1) > direction_changes(0) * 4); }

TEST_CASE(hold_setting_controls_how_step_like_the_curve_is)
{
  int still_smooth = 0, still_hard = 0;
  const int n = 4000;
  for (int i = 1; i < n; i++)
  {
    float p  = (float) i / (float) n;
    float pp = (float) (i - 1) / (float) n;
    if (fabsf(sr_flat(p, 0.4f, 6, SR_HOLD_SMOOTH) - sr_flat(pp, 0.4f, 6, SR_HOLD_SMOOTH)) < 1e-5f)
      still_smooth++;
    if (fabsf(sr_flat(p, 0.4f, 6, SR_HOLD_HARD) - sr_flat(pp, 0.4f, 6, SR_HOLD_HARD)) < 1e-5f)
      still_hard++;
  }
  CHECK(still_hard > still_smooth * 2);
}

TEST_CASE(length_index_maps_to_the_curated_step_counts)
{
  CHECK(sr_length_for_index(0) == 3);
  CHECK(sr_length_for_index(SR_LENGTH_COUNT - 1) == 64);
  // out-of-range indices must not read off the end of the tables
  CHECK(sr_length_for_index(-5) == 3);
  CHECK(sr_length_for_index(999) == 64);
}

// How many of the pattern's steps take a new value rather than tying to the
// previous one. Sampled inside each step's hold region, where the curve is
// sitting exactly on the slot value.
static int distinct_steps(float mod, int length_idx)
{
  int length  = sr_length_for_index(length_idx);
  int changes = 0;
  float prev  = stepped_random(0.4f / (float) length, 0.3f, mod, length_idx, SR_HOLD_HARD);

  for (int i = 1; i < length; i++)
  {
    float v = stepped_random(((float) i + 0.4f) / (float) length, 0.3f, mod, length_idx, SR_HOLD_HARD);
    if (fabsf(v - prev) > 1e-4f)
      changes++;
    prev = v;
  }
  return changes;
}

// MOD is the pattern's density: how often a step ties to the previous value
// instead of taking a new one.
TEST_CASE(mod_thins_the_pattern_out_as_it_turns_up)
{
  const int li = 7; // 16 steps - long enough that the short-pattern fade is fully in

  int busy    = distinct_steps(-1.0f, li);
  int neutral = distinct_steps(0.0f, li);
  int sparse  = distinct_steps(1.0f, li);

  CHECK(busy > neutral && neutral > sparse);
  CHECK(busy == sr_length_for_index(li) - 1); // every step takes a new value
  CHECK(sparse >= 3);                         // and it never collapses to one held value
}

// Whatever the density, the two properties the whole design rests on have to
// hold: the cycle closes on itself, and the output stays in range.
TEST_CASE(every_density_still_closes_the_loop_and_stays_in_range)
{
  for (float mod = -1.0f; mod <= 1.0f; mod += 0.25f)
  {
    for (int li = 0; li < SR_LENGTH_COUNT; li++)
    {
      for (int h = 0; h < HOLD_COUNT; h++)
      {
        // Same value either side of the loop point, so the cycle still closes.
        float at_end   = stepped_random(0.99999f, 0.3f, mod, li, HOLDS[h]);
        float at_start = stepped_random(0.0f, 0.3f, mod, li, HOLDS[h]);
        CHECK(fabsf(at_end - at_start) < 0.02f);

        for (float p = 0.0f; p < 1.0f; p += 0.02f)
        {
          float v = stepped_random(p, 0.3f, mod, li, HOLDS[h]);
          CHECK(v >= -1.0001f && v <= 1.0001f);
        }
      }
    }
  }
}

// The normalisation table gained a probability axis for exactly this reason: a
// correction generated at one density drifts as MOD moves off it, and the
// symptom is the pattern going quiet at some settings and not others.
TEST_CASE(no_density_setting_collapses_to_a_flat_output)
{
  for (float mod = -1.0f; mod <= 1.0f; mod += 0.2f)
  {
    for (int li = 0; li < SR_LENGTH_COUNT; li++)
    {
      float worst = 1e9f;
      for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
      {
        float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < 512; i++)
        {
          float v = stepped_random((float) i / 512.0f, shape, mod, li, SR_HOLD_SEMI);
          if (v < lo)
            lo = v;
          if (v > hi)
            hi = v;
        }
        if (hi - lo < worst)
          worst = hi - lo;
      }
      CHECK(worst > 0.5f);
    }
  }
}

int main(void)
{
  RUN_TEST(output_stays_in_bipolar_range);
  RUN_TEST(is_deterministic_for_same_inputs);
  RUN_TEST(phase_wraps_seamlessly_across_the_loop_point);
  RUN_TEST(changing_length_at_the_cycle_boundary_is_seamless);
  RUN_TEST(curve_is_continuous_at_every_step_boundary);
  RUN_TEST(morph_changes_gradually_with_the_shape_parameter);
  RUN_TEST(no_setting_collapses_to_a_flat_output);
  RUN_TEST(longer_patterns_contain_more_events);
  RUN_TEST(hold_setting_controls_how_step_like_the_curve_is);
  RUN_TEST(length_index_maps_to_the_curated_step_counts);
  RUN_TEST(mod_thins_the_pattern_out_as_it_turns_up);
  RUN_TEST(every_density_still_closes_the_loop_and_stays_in_range);
  RUN_TEST(no_density_setting_collapses_to_a_flat_output);
  return TESTKIT_SUMMARY();
}
