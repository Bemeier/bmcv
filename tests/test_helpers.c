// The pure helpers in helpers.h that carry real design constraints - so far,
// the phase warp, whose properties every beat-locked shape depends on.
#include "helpers.h"
#include "testkit.h"
#include <math.h>

// The warp has to fix both ends, or the cycle it warps stops closing on itself
// and every loop point becomes a step in the output.
TEST_CASE(the_phase_warp_leaves_the_cycle_endpoints_alone)
{
  for (float mod = -1.0f; mod <= 1.0f; mod += 0.1f)
  {
    CHECK_NEAR(phase_mod(0.0f, mod), 0.0f, 1e-6f);
    CHECK_NEAR(phase_mod(1.0f, mod), 1.0f, 1e-6f);
  }
}

// Monotone, or the waveform would run backwards somewhere in the cycle.
TEST_CASE(the_phase_warp_is_monotone_at_every_setting)
{
  for (float mod = -1.0f; mod <= 1.0f; mod += 0.1f)
  {
    float prev = -1.0f;
    for (int i = 0; i <= 1000; i++)
    {
      float p = (float) i / 1000.0f;
      float w = phase_mod(p, mod);
      CHECK(w > prev);
      CHECK(w >= -1e-6f && w <= 1.0f + 1e-6f);
      prev = w;
    }
  }
}

TEST_CASE(the_phase_warp_is_the_identity_at_the_centre)
{
  for (int i = 0; i <= 100; i++)
  {
    float p = (float) i / 100.0f;
    CHECK(phase_mod(p, 0.0f) == p);
  }
}

// The sign convention the whole module shares: negative leans early, positive
// leans late. Early means the phase has got further by any given point, so
// whatever the shape does, it does it sooner.
TEST_CASE(negative_leans_the_shape_early_and_positive_late)
{
  CHECK(phase_mod(0.5f, -0.5f) > 0.5f);
  CHECK(phase_mod(0.5f, 0.5f) < 0.5f);

  // ...and further out is further leaned, in both directions.
  CHECK(phase_mod(0.5f, -0.9f) > phase_mod(0.5f, -0.5f));
  CHECK(phase_mod(0.5f, 0.9f) < phase_mod(0.5f, 0.5f));
}

// No kink: the old two-segment warp had a slope discontinuity where its halves
// met, which showed up as a corner halfway through the waveform.
TEST_CASE(the_phase_warp_has_no_corner_in_it)
{
  const float step = 1.0f / 2000.0f;

  for (float mod = -0.9f; mod <= 0.9f; mod += 0.3f)
  {
    float worst      = 0.0f;
    float prev_slope = (phase_mod(step, mod) - phase_mod(0.0f, mod)) / step;

    for (int i = 1; i < 2000; i++)
    {
      float p     = (float) i / 2000.0f;
      float slope = (phase_mod(p + step, mod) - phase_mod(p, mod)) / step;
      float jump  = fabsf(slope - prev_slope);
      if (jump > worst)
        worst = jump;
      prev_slope = slope;
    }

    // A smooth curve's slope creeps between adjacent samples; a corner jumps.
    CHECK(worst < 0.5f);
  }
}

int main(void)
{
  RUN_TEST(the_phase_warp_leaves_the_cycle_endpoints_alone);
  RUN_TEST(the_phase_warp_is_monotone_at_every_setting);
  RUN_TEST(the_phase_warp_is_the_identity_at_the_centre);
  RUN_TEST(negative_leans_the_shape_early_and_positive_late);
  RUN_TEST(the_phase_warp_has_no_corner_in_it);
  return TESTKIT_SUMMARY();
}
