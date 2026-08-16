#include "drift.h"
#include "testkit.h"
#include <math.h>

// One cycle, as the engine plays it.
static void cycle(float shape, float mod, int n, float* out)
{
  for (int i = 0; i < n; i++)
  {
    out[i] = drift_value((float) i / (float) n, shape, mod);
  }
}

static int direction_changes(const float* v, int n)
{
  int turns        = 0;
  float prev_slope = 0.0f;
  for (int i = 1; i < n; i++)
  {
    float slope = v[i] - v[i - 1];
    if (slope * prev_slope < 0.0f)
      turns++;
    if (slope != 0.0f)
      prev_slope = slope;
  }
  return turns;
}

TEST_CASE(output_stays_in_bipolar_range)
{
  for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
  {
    for (float mod = -1.0f; mod <= 1.0f; mod += 0.1f)
    {
      for (float phase = 0.0f; phase < 1.0f; phase += 0.01f)
      {
        float v = drift_value(phase, shape, mod);
        CHECK(v >= -1.0001f && v <= 1.0001f);
      }
    }
  }
}

TEST_CASE(is_deterministic_for_same_inputs) { CHECK(drift_value(0.31f, 0.2f, -0.4f) == drift_value(0.31f, 0.2f, -0.4f)); }

// Every octave runs a whole number of times per cycle, which is the whole
// reason for choosing them that way: the waveform meets itself at the loop
// point rather than being made to.
TEST_CASE(the_cycle_closes_on_itself)
{
  for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
  {
    for (float mod = -1.0f; mod <= 1.0f; mod += 0.25f)
    {
      CHECK_NEAR(drift_value(0.0f, shape, mod), drift_value(1.0f, shape, mod), 1e-5);
    }
  }
}

// SHP is a ramp of detail, so unlike SHP in the stepped modes it does not wrap:
// its two ends are opposite ends, the way their MOD's density is. What the two
// ends share is the wander underneath, since each octave still advances a whole
// number of turns across the sweep.
TEST_CASE(detail_rises_across_shp)
{
  const int n = 4096;
  static float v[4096];
  int turns[9];

  for (int i = 0; i < 9; i++)
  {
    cycle(-1.0f + 2.0f * (float) i / 8.0f, 0.0f, n, v);
    turns[i] = direction_changes(v, n);
  }

  // Not point by point: the octaves move along SHP as well as growing, so a
  // step of the knob can turn a peak into a shoulder. Measured 2 turns per
  // cycle at the left and 16 at the right, rising throughout.
  int calm = turns[0] + turns[1] + turns[2];
  int busy = turns[6] + turns[7] + turns[8];
  CHECK(busy > calm * 3);
  CHECK(turns[4] > turns[0]);
  CHECK(turns[8] > turns[4]);
}

// The property that makes this the cheap shape: the mean over a cycle is zero
// by construction - every octave is an odd wave over a whole number of periods
// - so there is nothing to centre and no table saying how to. The stepped modes
// need a per-channel measurement for the same guarantee and still only get
// within 0.38 of it.
TEST_CASE(no_setting_leaves_the_wander_off_centre)
{
  const int n = 2048;
  static float v[2048];

  for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
  {
    cycle(shape, 0.0f, n, v);

    float dc = 0.0f;
    for (int i = 0; i < n; i++)
      dc += v[i];
    CHECK(fabsf(dc / (float) n) < 0.01f);
  }
}

// No dead settings, and no setting much louder than its neighbours - the same
// two things the stepped modes spend a correction on. Here it is two constants:
// one octave swings the whole range on its own, and the rest are scaled to what
// they add rather than to what they could add.
TEST_CASE(every_setting_swings_about_as_far_as_the_others)
{
  const int n = 1024;
  static float v[1024];
  float worst = 9.0f, loudest = 0.0f;

  for (float shape = -1.0f; shape <= 1.0f; shape += 0.01f)
  {
    cycle(shape, 0.0f, n, v);

    float lo = 9.0f, hi = -9.0f;
    for (int i = 0; i < n; i++)
    {
      if (v[i] < lo)
        lo = v[i];
      if (v[i] > hi)
        hi = v[i];
    }
    if (hi - lo < worst)
      worst = hi - lo;
    if (hi - lo > loudest)
      loudest = hi - lo;
  }

  // Measured 1.32..1.83 of the 2.0 range, against the stepped modes' 1.10..1.58
  // and their ratio of 1.44.
  CHECK(worst > 1.1f);
  CHECK(loudest < 1.95f);
  CHECK(loudest / worst < 1.5f);
}

// A wander with a corner in it is not a wander. sr_scurve over sr_tri rounds
// the turn at every peak; without it the curve has a kink wherever an octave
// changes direction.
TEST_CASE(the_curve_has_no_corners)
{
  const int n = 8000;
  static float v[8000];

  for (float shape = -1.0f; shape <= 1.0f; shape += 0.1f)
  {
    cycle(shape, 0.0f, n, v);

    // A corner is a step in the slope, so this watches the second difference
    // rather than the first.
    float worst = 0.0f;
    for (int i = 2; i < n; i++)
    {
      float bend = fabsf((v[i] - v[i - 1]) - (v[i - 1] - v[i - 2]));
      if (bend > worst)
        worst = bend;
    }
    CHECK(worst < 0.002f);
  }
}

// The budget every one of these shapes is set against: a small turn deforms
// what is playing rather than replacing it. This shape spends 0.26 of the 0.35
// limit, and it spends it deliberately - at one excursion of the wander per
// sweep instead of three it measured 0.09, and a detent that quiet is a detent
// that does nothing.
TEST_CASE(a_small_turn_moves_the_wander_without_replacing_it)
{
  for (int i = 0; i <= 200; i++)
  {
    float x = -1.0f + 2.0f * (float) i / 200.0f;
    if (x > 0.99f)
      break;

    for (float phase = 0.0f; phase < 1.0f; phase += 0.01f)
    {
      CHECK(fabsf(drift_value(phase, x, 0.0f) - drift_value(phase, x + 0.01f, 0.0f)) < 0.35f);
      CHECK(fabsf(drift_value(phase, 0.3f, x) - drift_value(phase, 0.3f, x + 0.01f)) < 0.35f);
    }
  }
}

// What SHP is: one slow swell at the left, detail on top of it at the right.
TEST_CASE(shp_steers_how_much_detail_the_wander_has)
{
  const int n = 4096;
  static float smooth[4096], busy[4096];

  cycle(-1.0f, 0.0f, n, smooth);
  cycle(1.0f, 0.0f, n, busy);

  CHECK(direction_changes(busy, n) > direction_changes(smooth, n) * 3);
}

// And what MOD is: where the wander sits. The same reshaping the control mode
// puts on SHP, which is free here because it is a function of the value and
// this shape has no correction for it to interfere with.
TEST_CASE(mod_leans_where_the_wander_sits)
{
  const int n = 2048;
  static float low[2048], plain[2048], gate[2048];

  cycle(0.2f, -1.0f, n, low);
  cycle(0.2f, 0.0f, n, plain);
  cycle(0.2f, 1.0f, n, gate);

  float low_mean = 0.0f, plain_mean = 0.0f, low_peak = -9.0f;
  int gate_middling = 0, plain_middling = 0;
  for (int i = 0; i < n; i++)
  {
    low_mean += low[i];
    plain_mean += plain[i];
    if (low[i] > low_peak)
      low_peak = low[i];
    if (fabsf(gate[i]) < 0.3f)
      gate_middling++;
    if (fabsf(plain[i]) < 0.3f)
      plain_middling++;
  }

  CHECK(low_mean < plain_mean - 0.2f * (float) n);
  CHECK(low_peak - low_mean / (float) n > 0.2f); // sits low, still reaches
  CHECK(gate_middling < plain_middling);
}

int main(void)
{
  RUN_TEST(output_stays_in_bipolar_range);
  RUN_TEST(is_deterministic_for_same_inputs);
  RUN_TEST(the_cycle_closes_on_itself);
  RUN_TEST(detail_rises_across_shp);
  RUN_TEST(no_setting_leaves_the_wander_off_centre);
  RUN_TEST(every_setting_swings_about_as_far_as_the_others);
  RUN_TEST(the_curve_has_no_corners);
  RUN_TEST(a_small_turn_moves_the_wander_without_replacing_it);
  RUN_TEST(shp_steers_how_much_detail_the_wander_has);
  RUN_TEST(mod_leans_where_the_wander_sits);
  return TESTKIT_SUMMARY();
}
