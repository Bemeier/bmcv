#include "stepped_random.h"
#include "testkit.h"
#include <math.h>

static const float HOLDS[] = {SR_HOLD_SMOOTH, SR_HOLD_SEMI, SR_HOLD_HARD};
#define HOLD_COUNT ((int) (sizeof(HOLDS) / sizeof(HOLDS[0])))

// Peak-to-peak of one full cycle at a given setting.
static float cycle_span(float shape, float mod, float hold)
{
  float lo = 1e9f, hi = -1e9f;
  for (int i = 0; i < 2048; i++)
  {
    float v = stepped_random((float) i / 2048.0f, shape, mod, hold);
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
      for (float mod = -1.0f; mod <= 1.0f; mod += 0.2f)
      {
        for (float phase = 0.0f; phase < 1.0f; phase += 0.02f)
        {
          float v = stepped_random(phase, shape, mod, HOLDS[h]);
          CHECK(v >= -1.0001f && v <= 1.0001f);
        }
      }
    }
  }
}

TEST_CASE(is_deterministic_for_same_inputs)
{
  CHECK(stepped_random(0.37f, 0.1f, -0.4f, SR_HOLD_SEMI) == stepped_random(0.37f, 0.1f, -0.4f, SR_HOLD_SEMI));
}

// The point of an integer, circularly-read lattice: the waveform closes on
// itself, so the loop point is inaudible and stays put under the PLL.
TEST_CASE(phase_wraps_seamlessly_across_the_loop_point)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.25f)
    {
      for (float mod = -1.0f; mod <= 1.0f; mod += 0.25f)
      {
        CHECK_NEAR(stepped_random(0.0f, shape, mod, HOLDS[h]), stepped_random(1.0f, shape, mod, HOLDS[h]), 1e-4);
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
    for (float mod = -1.0f; mod <= 1.0f; mod += 0.25f)
    {
      float prev     = stepped_random(0.0f, 0.3f, mod, HOLDS[h]);
      float max_jump = 0.0f;
      for (int i = 1; i <= steps; i++)
      {
        float v    = stepped_random((float) i / (float) steps, 0.3f, mod, HOLDS[h]);
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
        float d = fabsf(stepped_random(phase, shape, 0.2f, HOLDS[h]) - stepped_random(phase, shape + ds, 0.2f, HOLDS[h]));
        if (d > worst)
          worst = d;
      }
      CHECK(worst < 0.25f);
    }
  }
}

// "No total flat areas": every combination of shape and mod must still swing.
TEST_CASE(no_setting_collapses_to_a_flat_output)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    float worst = 2.0f;
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.02f)
    {
      for (float mod = -1.0f; mod <= 1.0f; mod += 0.05f)
      {
        float span = cycle_span(shape, mod, HOLDS[h]);
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

static int direction_changes(float mod)
{
  int turns         = 0;
  float prev        = stepped_random(0.0f, 0.25f, mod, SR_HOLD_SEMI);
  float prev_slope  = 0.0f;
  for (int i = 1; i <= 4096; i++)
  {
    float v     = stepped_random((float) i / 4096.0f, 0.25f, mod, SR_HOLD_SEMI);
    float slope = v - prev;
    if (slope * prev_slope < 0.0f)
      turns++;
    if (slope != 0.0f)
      prev_slope = slope;
    prev = v;
  }
  return turns;
}

TEST_CASE(mod_parameter_controls_pattern_length)
{
  CHECK(direction_changes(1.0f) > direction_changes(-1.0f) * 4);
}

TEST_CASE(hold_setting_controls_how_step_like_the_curve_is)
{
  int still_smooth = 0, still_hard = 0;
  const int n = 4000;
  for (int i = 1; i < n; i++)
  {
    float p  = (float) i / (float) n;
    float pp = (float) (i - 1) / (float) n;
    if (fabsf(stepped_random(p, 0.4f, 0.0f, SR_HOLD_SMOOTH) - stepped_random(pp, 0.4f, 0.0f, SR_HOLD_SMOOTH)) < 1e-5f)
      still_smooth++;
    if (fabsf(stepped_random(p, 0.4f, 0.0f, SR_HOLD_HARD) - stepped_random(pp, 0.4f, 0.0f, SR_HOLD_HARD)) < 1e-5f)
      still_hard++;
  }
  CHECK(still_hard > still_smooth * 2);
}

int main(void)
{
  RUN_TEST(output_stays_in_bipolar_range);
  RUN_TEST(is_deterministic_for_same_inputs);
  RUN_TEST(phase_wraps_seamlessly_across_the_loop_point);
  RUN_TEST(curve_is_continuous_at_every_step_boundary);
  RUN_TEST(morph_changes_gradually_with_the_shape_parameter);
  RUN_TEST(no_setting_collapses_to_a_flat_output);
  RUN_TEST(mod_parameter_controls_pattern_length);
  RUN_TEST(hold_setting_controls_how_step_like_the_curve_is);
  return TESTKIT_SUMMARY();
}
