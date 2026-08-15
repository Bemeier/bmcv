#include "stepped_random.h"
#include "stepped_random_pattern.h" // sr_step_at, sr_swing_amount
#include "stepped_random_table.h"   // SR_LENGTH_COUNT
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
// turn of either knob must deform the pattern, not re-randomise it.
//
// This is the budget every lever depth and rate is set against, so it is swept
// finely: at 0.05 per step the old version of this test sampled too coarsely to
// see its own limit being exceeded, and it checked one pattern length out of
// eleven.
//
// Two limits, because a short cycle cannot change gently: with only a handful
// of values in it, one of them moving is a large fraction of the whole, and the
// shortest lengths sat at 0.88-0.90 in the original algorithm too. Everything
// from eight steps up - which is the range this shape is really for - is held
// to the tight figure, and measures 0.29.
#define SR_SMALL_TURN_LIMIT 0.35f
#define SR_SMALL_TURN_LIMIT_SHORT 1.10f

static float worst_small_turn(int length_idx, int on_mod)
{
  const float d = 0.01f;
  float worst   = 0.0f;

  for (int i = 0; i <= 200; i++)
  {
    float x = -1.0f + 2.0f * (float) i / 200.0f;
    if (x > 1.0f - d)
      break;

    for (float phase = 0.0f; phase < 1.0f; phase += 0.01f)
    {
      float a =
          on_mod ? stepped_random(phase, 0.3f, x, length_idx, SR_HOLD_SMOOTH) : stepped_random(phase, x, 0.0f, length_idx, SR_HOLD_SMOOTH);
      float b    = on_mod ? stepped_random(phase, 0.3f, x + d, length_idx, SR_HOLD_SMOOTH)
                          : stepped_random(phase, x + d, 0.0f, length_idx, SR_HOLD_SMOOTH);
      float diff = fabsf(a - b);
      if (diff > worst)
        worst = diff;
    }
  }
  return worst;
}

TEST_CASE(a_small_turn_deforms_the_pattern_rather_than_replacing_it)
{
  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    float limit = (sr_length_for_index(li) < 8) ? SR_SMALL_TURN_LIMIT_SHORT : SR_SMALL_TURN_LIMIT;
    CHECK(worst_small_turn(li, 0) < limit);
    CHECK(worst_small_turn(li, 1) < limit);
  }
}

// The flat-spot tests below step SHP coarsely, and a coarse step can walk
// straight over a narrow notch: one lived at shape 0.475 for a long time,
// unseen, because 0.475 is not on any of their grids. Four values sitting
// within 0.07 of each other up against the top rail, at three and four steps.
//
// So this one sweeps finely, at the shortest lengths where the notches were,
// and reads the span off the step values rather than a rendered cycle - the
// curve passes exactly through them, so it is the same number for a fraction
// of the work, which is what makes the fine sweep affordable.
TEST_CASE(no_narrow_notch_collapses_the_output)
{
  for (int li = 0; li < SR_LENGTH_COUNT; li++)
  {
    int length = sr_length_for_index(li);

    for (int mi = 0; mi <= 20; mi++)
    {
      float mod = -1.0f + 2.0f * (float) mi / 20.0f;

      for (int si = 0; si <= 1000; si++)
      {
        float shape = -1.0f + 2.0f * (float) si / 1000.0f;
        float lo = 9.0f, hi = -9.0f;
        for (int k = 0; k < length; k++)
        {
          float v = stepped_random(((float) k + 0.5f) / (float) length, shape, mod, li, SR_HOLD_HARD);
          if (v < lo)
            lo = v;
          if (v > hi)
            hi = v;
        }
        CHECK(hi - lo > 0.5f);
      }
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

// SHP -1 is the character-neutral end of the sweep: the contour blend and the
// value shaper are both at zero there, so the steps are the plain orbit values.
// The MOD tests below read the density off that, the way the SHP tests above
// read the pattern off MOD 0 - each parameter measured with the other one out
// of the way.
#define SR_SHAPE_NEUTRAL (-1.0f)

// How many of the pattern's steps take a new value rather than tying to the
// previous one. Sampled inside each step's hold region, where the curve is
// sitting exactly on the slot value.
//
// "Distinct" is a threshold rather than an equality because a tie is now a
// crossfade: a step part-way through tying has moved a little way toward its
// own value rather than sitting exactly on the previous one. A step that has
// travelled less than a tenth of the range is a tie by ear.
static int distinct_steps(float mod, int length_idx)
{
  int length  = sr_length_for_index(length_idx);
  int changes = 0;
  float prev  = stepped_random(0.4f / (float) length, SR_SHAPE_NEUTRAL, mod, length_idx, SR_HOLD_HARD);

  for (int i = 1; i < length; i++)
  {
    float v = stepped_random(((float) i + 0.4f) / (float) length, SR_SHAPE_NEUTRAL, mod, length_idx, SR_HOLD_HARD);
    if (fabsf(v - prev) > 0.1f)
      changes++;
    prev = v;
  }
  return changes;
}

// Peak-to-peak of the pattern's steps, and how far apart consecutive steps sit.
static void pattern_steps(float shape, float mod, int length_idx, float* out)
{
  int length = sr_length_for_index(length_idx);
  for (int i = 0; i < length; i++)
  {
    out[i] = stepped_random(((float) i + 0.4f) / (float) length, shape, mod, length_idx, SR_HOLD_HARD);
  }
}

// Mean distance between consecutive steps: low is a melodic walk, high is leaps.
static float mean_interval(float shape, int length_idx)
{
  float v[64];
  int length = sr_length_for_index(length_idx);
  pattern_steps(shape, 0.0f, length_idx, v);

  float sum = 0.0f;
  for (int i = 0; i < length; i++)
  {
    sum += fabsf(v[(i + 1) % length] - v[i]);
  }
  return sum / (float) length;
}

// Fraction of steps sitting out near the rails rather than in the middle.
static float edge_fraction(float shape, int length_idx)
{
  float v[64];
  int length = sr_length_for_index(length_idx);
  pattern_steps(shape, 0.0f, length_idx, v);

  int n = 0;
  for (int i = 0; i < length; i++)
  {
    if (fabsf(v[i]) > 0.6f)
      n++;
  }
  return (float) n / (float) length;
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
  CHECK(sparse >= 3); // and it never collapses to one held value

  // Fully left, no step is held: every one moves off the previous value. Not
  // that every one moves *far* - two independent random values land close
  // together often enough, and counting those as ties is what a threshold on
  // the interval would do.
  int length = sr_length_for_index(li);
  float prev = stepped_random(0.4f / (float) length, SR_SHAPE_NEUTRAL, -1.0f, li, SR_HOLD_HARD);
  for (int i = 1; i < length; i++)
  {
    float v = stepped_random(((float) i + 0.4f) / (float) length, SR_SHAPE_NEUTRAL, -1.0f, li, SR_HOLD_HARD);
    CHECK(fabsf(v - prev) > 1e-3f);
    prev = v;
  }
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

// MOD used to be a dead staircase: 80-90% of its travel did nothing at all,
// because the tied set only changed where the density threshold crossed a
// slot's gate. Every point of the knob has to move the pattern.
TEST_CASE(mod_moves_the_pattern_everywhere_on_its_travel)
{
  for (int li = 6; li < SR_LENGTH_COUNT; li++) // 16 steps and up
  {
    float quietest = 1e9f;
    for (float mod = -1.0f; mod < 0.99f; mod += 0.01f)
    {
      float sum = 0.0f;
      for (int i = 0; i < 256; i++)
      {
        float p = (float) i / 256.0f;
        float d = stepped_random(p, 0.3f, mod, li, SR_HOLD_SMOOTH) - stepped_random(p, 0.3f, mod + 0.01f, li, SR_HOLD_SMOOTH);
        sum += d * d;
      }
      float rms = sqrtf(sum / 256.0f);
      if (rms < quietest)
        quietest = rms;
    }
    // Measured worst case across these lengths is 0.0033 of a 2.0 range. The
    // old algorithm sat at exactly 0.0 for most of the knob.
    CHECK(quietest > 0.001f);
  }
}

// SHP does not only pick different values, it changes their character. Part of
// the way through the sweep the steps are blended toward a running average of
// the ones before them, which turns a sequence of leaps into a walk.
//
// Scanned rather than probed at two fixed shapes. Each lever runs at its own
// rate across the knob now, so where "contour off" and "contour full" actually
// fall moves whenever a rate is retuned - and a test that hard-codes them fails
// for a reason that has nothing to do with the lever having stopped working.
TEST_CASE(shape_steers_the_contour_from_leaps_to_a_walk)
{
  for (int li = 6; li < SR_LENGTH_COUNT; li++)
  {
    float leaps = 0.0f, walk = 9.0f;

    for (int si = 0; si <= 400; si++)
    {
      float iv = mean_interval(-1.0f + 2.0f * (float) si / 400.0f, li);
      if (iv > leaps)
        leaps = iv;
      if (iv < walk)
        walk = iv;
    }
    // Measured 5.0x at 16 steps rising to 8.4x at 48.
    CHECK(leaps > walk * 3.0f);
  }
}

// The shaper reshapes the spread of the values themselves: bunched around the
// middle at some points of the sweep, pushed out to the rails at others.
TEST_CASE(shape_steers_the_spread_of_values)
{
  for (int li = 6; li < SR_LENGTH_COUNT; li++)
  {
    float middle = 9.0f, rails = 0.0f;

    for (int si = 0; si <= 400; si++)
    {
      float ed = edge_fraction(-1.0f + 2.0f * (float) si / 400.0f, li);
      if (ed > rails)
        rails = ed;
      if (ed < middle)
        middle = ed;
    }
    // Measured 0.00 to 0.60-0.75: somewhere on the knob nothing is near a rail,
    // and somewhere else most of the pattern is.
    CHECK(middle < 0.10f);
    CHECK(rails > 0.35f);
  }
}

// The step grid itself: MOD leans alternate steps long and short. The widths
// have to keep summing to one whatever the skew, or the cycle stops closing -
// and phase 1.0 has to land at the *end* of the last step, not the start of it.
// Getting that wrong put a jump at the loop point, the one place there must
// never be one.
TEST_CASE(swing_skews_the_step_grid_without_breaking_the_cycle)
{
  const int lengths[] = {3, 4, 5, 8, 15, 16};

  for (int k = 0; k < (int) (sizeof(lengths) / sizeof(lengths[0])); k++)
  {
    int length = lengths[k];

    for (float mod = -1.0f; mod <= 1.0f; mod += 0.1f)
    {
      float swing = sr_swing_amount(mod);
      float within;

      // phase 1.0 is the end of the last step
      CHECK(sr_step_at(1.0f, length, swing, &within) == length - 1);
      CHECK_NEAR(within, 1.0f, 1e-4);

      // phase 0 is the start of the first
      CHECK(sr_step_at(0.0f, length, swing, &within) == 0);
      CHECK_NEAR(within, 0.0f, 1e-4);

      // every step is entered exactly once, in order, and `within` runs 0..1
      int seen = 0, last = -1;
      float prev_within = 0.0f;
      for (int i = 0; i <= 4096; i++)
      {
        int step = sr_step_at((float) i / 4096.0f, length, swing, &within);
        CHECK(within >= -1e-4f && within <= 1.0f + 1e-4f);
        if (step != last)
        {
          CHECK(step == last + 1); // never skips or goes back
          if (last >= 0)
            CHECK(prev_within > 0.9f); // and left the previous step at its end
          seen++;
          last = step;
        }
        prev_within = within;
      }
      CHECK(seen == length);
    }
  }
}

// Straight time at the centre of the knob and at both ends, leaning one way
// then the other in between.
TEST_CASE(swing_is_neutral_at_the_centre_and_ends_of_mod)
{
  CHECK_NEAR(sr_swing_amount(0.0f), 0.0f, 1e-4);
  CHECK_NEAR(sr_swing_amount(-1.0f), 0.0f, 1e-4);
  CHECK_NEAR(sr_swing_amount(1.0f), 0.0f, 1e-4);
  CHECK(sr_swing_amount(0.5f) * sr_swing_amount(-0.5f) < 0.0f); // opposite ways
  CHECK(fabsf(sr_swing_amount(0.5f)) > 0.2f);                   // and not subtle
}

int main(void)
{
  RUN_TEST(output_stays_in_bipolar_range);
  RUN_TEST(is_deterministic_for_same_inputs);
  RUN_TEST(phase_wraps_seamlessly_across_the_loop_point);
  RUN_TEST(changing_length_at_the_cycle_boundary_is_seamless);
  RUN_TEST(curve_is_continuous_at_every_step_boundary);
  RUN_TEST(a_small_turn_deforms_the_pattern_rather_than_replacing_it);
  RUN_TEST(no_narrow_notch_collapses_the_output);
  RUN_TEST(no_setting_collapses_to_a_flat_output);
  RUN_TEST(longer_patterns_contain_more_events);
  RUN_TEST(hold_setting_controls_how_step_like_the_curve_is);
  RUN_TEST(length_index_maps_to_the_curated_step_counts);
  RUN_TEST(mod_thins_the_pattern_out_as_it_turns_up);
  RUN_TEST(every_density_still_closes_the_loop_and_stays_in_range);
  RUN_TEST(no_density_setting_collapses_to_a_flat_output);
  RUN_TEST(mod_moves_the_pattern_everywhere_on_its_travel);
  RUN_TEST(shape_steers_the_contour_from_leaps_to_a_walk);
  RUN_TEST(shape_steers_the_spread_of_values);
  RUN_TEST(swing_skews_the_step_grid_without_breaking_the_cycle);
  RUN_TEST(swing_is_neutral_at_the_centre_and_ends_of_mod);
  return TESTKIT_SUMMARY();
}
