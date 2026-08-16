#include "stepped.h"
#include "stepped_pattern.h" // st_step_at, st_swing_amount
#include "stepped_table.h"   // ST_LENGTH_COUNT
#include "testkit.h"
#include <math.h>

static const float HOLDS[] = {ST_HOLD_SMOOTH, ST_HOLD_SEMI, ST_HOLD_HARD};

// The correction depends on the setting, not on the phase, and every loop below
// walks phase at a standing setting - so remember the last one rather than
// measure the whole pattern again for every sample. The engine does exactly
// this with its rolling scan; this is the test's version of the same fact, and
// without it the suite spends a minute re-measuring patterns it already knows.
static float st_eval(float phase, float shape, float mod, int length_idx, float hold)
{
  static float prev_shape = 9.0f, prev_mod = 9.0f;
  static int prev_idx = -1;
  static StNorm norm;

  if (shape != prev_shape || mod != prev_mod || length_idx != prev_idx)
  {
    norm       = st_norm_exact(shape, mod, length_idx);
    prev_shape = shape;
    prev_mod   = mod;
    prev_idx   = length_idx;
  }
  StDrive bare = {hold, 0.0f, 0.0f};
  return stepped_shape_with(phase, shape, mod, length_idx, &bare, &norm);
}

// Every property below is about the pattern itself, which is what MOD 0 gives:
// even steps, no skew. The MOD cases have their own tests at the end.
static float st_flat(float phase, float shape, int length_idx, float hold) { return st_eval(phase, shape, 0.0f, length_idx, hold); }
#define HOLD_COUNT ((int) (sizeof(HOLDS) / sizeof(HOLDS[0])))

// Peak-to-peak of one full cycle at a given setting.
static float cycle_span(float shape, int length_idx, float hold)
{
  float lo = 1e9f, hi = -1e9f;
  for (int i = 0; i < 2048; i++)
  {
    float v = st_flat((float) i / 2048.0f, shape, length_idx, hold);
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
      for (int li = 0; li < ST_LENGTH_COUNT; li++)
      {
        for (float phase = 0.0f; phase < 1.0f; phase += 0.02f)
        {
          float v = st_flat(phase, shape, li, HOLDS[h]);
          CHECK(v >= -1.0001f && v <= 1.0001f);
        }
      }
    }
  }
}

TEST_CASE(is_deterministic_for_same_inputs) { CHECK(st_flat(0.37f, 0.1f, 4, ST_HOLD_SEMI) == st_flat(0.37f, 0.1f, 4, ST_HOLD_SEMI)); }

// The plain entry point measures the pattern for itself; the engine's hands it
// the measurement. Same shape either way, or every test below is describing a
// function the module does not call.
TEST_CASE(measuring_the_correction_here_or_passing_it_in_are_the_same_shape)
{
  for (float shape = -1.0f; shape <= 1.0f; shape += 0.13f)
  {
    for (float mod = -1.0f; mod <= 1.0f; mod += 0.29f)
    {
      for (int li = 0; li < ST_LENGTH_COUNT; li++)
      {
        StNorm n = st_norm_exact(shape, mod, li);
        for (float phase = 0.0f; phase < 1.0f; phase += 0.077f)
        {
          CHECK(stepped_shape(phase, shape, mod, li, ST_HOLD_SMOOTH) ==
                stepped_shape_with(phase, shape, mod, li, &(StDrive){ST_HOLD_SMOOTH, 0.0f, 0.0f}, &n));
        }
      }
    }
  }
}

// The point of an integer, circularly-read lattice: the waveform closes on
// itself, so the loop point is inaudible and stays put under the PLL.
TEST_CASE(phase_wraps_seamlessly_across_the_loop_point)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.25f)
    {
      for (int li = 0; li < ST_LENGTH_COUNT; li++)
      {
        CHECK_NEAR(st_flat(0.0f, shape, li, HOLDS[h]), st_flat(1.0f, shape, li, HOLDS[h]), 1e-4);
      }
    }
  }
}

// Every length starts and ends its cycle on slot 0, whose raw value does not
// depend on the length; the correction is scaled about slot 0 and its constant
// is shared by every length. So the engine can switch pattern length on the
// cycle wrap and the signal barely moves - which is what makes a scene
// crossfade of MOD glitch-free. Switching mid-cycle instead jumps by up to 1.8
// of a 2.0 range.
//
// Not exactly equal: the gain is interpolated between bins while slot 0's value
// is computed exactly, so between two bins the two no longer cancel, by an
// amount proportional to how much the lengths' gains differ. Measured worst
// case 0.044 of 2.0 here, and 0.122 once MOD is off centre as well.
TEST_CASE(changing_length_at_the_cycle_boundary_is_seamless)
{
  for (int h = 0; h < HOLD_COUNT; h++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
    {
      for (int from = 0; from < ST_LENGTH_COUNT; from++)
      {
        for (int to = 0; to < ST_LENGTH_COUNT; to++)
        {
          float leaving  = st_flat(1.0f, shape, from, HOLDS[h]);
          float entering = st_flat(0.0f, shape, to, HOLDS[h]);
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
    for (int li = 0; li < ST_LENGTH_COUNT; li++)
    {
      float prev     = st_flat(0.0f, 0.3f, li, HOLDS[h]);
      float max_jump = 0.0f;
      for (int i = 1; i <= steps; i++)
      {
        float v    = st_flat((float) i / (float) steps, 0.3f, li, HOLDS[h]);
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
// Measured as Spearman's rank correlation between the two patterns, which is
// what that sentence means: the steps keep their order, whatever happens to
// their values. That distinction is the whole point of measuring it this way.
// The bound used to be on how far any sample moved, which was the same thing
// back when character came only from redrawing the pattern - but it is not the
// same thing for a reshaping of the finished values, which cannot move a step
// out of order however far it moves the samples. That is true by construction,
// not by measurement, so the old bound was charging full price for the one
// mechanism that is musically free.
//
// Ties take average ranks, since a tied step shares its value with the one
// before it.
//
// Short patterns are exempt, as they were under the old bound: three values
// cannot change gently, and at three steps one swap inverts the order outright
// (measured, rho reaches -0.50).
#define ST_SMALL_TURN_RHO 0.55f
#define ST_SMALL_TURN_RHO_MIN_LENGTH 8

static void step_ranks(const float* v, int n, float* r)
{
  for (int i = 0; i < n; i++)
  {
    float below = 0.0f, equal = 0.0f;
    for (int j = 0; j < n; j++)
    {
      if (v[j] < v[i] - 1e-6f)
        below += 1.0f;
      else if (fabsf(v[j] - v[i]) <= 1e-6f)
        equal += 1.0f;
    }
    r[i] = below + 0.5f * (equal - 1.0f);
  }
}

static float spearman(const float* a, const float* b, int n)
{
  float ra[64], rb[64];
  step_ranks(a, n, ra);
  step_ranks(b, n, rb);

  float ma = 0.0f, mb = 0.0f;
  for (int i = 0; i < n; i++)
  {
    ma += ra[i];
    mb += rb[i];
  }
  ma /= (float) n;
  mb /= (float) n;

  float num = 0.0f, da = 0.0f, db = 0.0f;
  for (int i = 0; i < n; i++)
  {
    num += (ra[i] - ma) * (rb[i] - mb);
    da += (ra[i] - ma) * (ra[i] - ma);
    db += (rb[i] - mb) * (rb[i] - mb);
  }
  if (da < 1e-9f || db < 1e-9f)
  {
    return 1.0f; // a pattern with no order to keep cannot lose it
  }
  return num / sqrtf(da * db);
}

// The pattern as a channel plays it, one value per step.
static void driven_pattern(float shape, float mod, int length_idx, float* out)
{
  int n       = st_length_for_index(length_idx);
  StDrive d   = st_drive(shape, mod);
  StNorm norm = st_norm_exact(shape, mod, length_idx);
  for (int k = 0; k < n; k++)
  {
    out[k] = stepped_shape_with(((float) k + 0.5f) / (float) n, shape, mod, length_idx, &d, &norm);
  }
}

TEST_CASE(a_small_turn_deforms_the_pattern_rather_than_replacing_it)
{
  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    int n = st_length_for_index(li);
    if (n < ST_SMALL_TURN_RHO_MIN_LENGTH)
    {
      continue;
    }

    float a[64], b[64];
    for (int i = 0; i <= 200; i++)
    {
      float x = -1.0f + 2.0f * (float) i / 200.0f;
      if (x > 0.99f)
        break;

      driven_pattern(x, 0.3f, li, a);
      driven_pattern(x + 0.01f, 0.3f, li, b);
      CHECK(spearman(a, b, n) > ST_SMALL_TURN_RHO);

      driven_pattern(0.3f, x, li, a);
      driven_pattern(0.3f, x + 0.01f, li, b);
      CHECK(spearman(a, b, n) > ST_SMALL_TURN_RHO);
    }
  }
}

// And a coarse guard on the other thing a small turn must not do: leap. Keeping
// the order is not enough on its own - a reshaping steep enough to throw one
// step from the bottom of the range to the top would pass the test above and
// still be heard as a jump.
//
// Loose on purpose. This is not the "sounds related" rule any more, which is
// why it is no longer 0.35: that number was calibrated when this measure was
// standing in for the one above.
#define ST_SMALL_TURN_LIMIT 0.60f
#define ST_SMALL_TURN_LIMIT_SHORT 1.10f

static float worst_small_turn(int length_idx, int on_mod)
{
  const float d = 0.01f;
  float worst   = 0.0f;

  for (int i = 0; i <= 200; i++)
  {
    float x = -1.0f + 2.0f * (float) i / 200.0f;
    if (x > 1.0f - d)
      break;

    float a_shape = on_mod ? 0.3f : x, a_mod = on_mod ? x : 0.0f;
    float b_shape = on_mod ? 0.3f : x + d, b_mod = on_mod ? x + d : 0.0f;
    StDrive da = st_drive(a_shape, a_mod), db = st_drive(b_shape, b_mod);
    StNorm na = st_norm_exact(a_shape, a_mod, length_idx), nb = st_norm_exact(b_shape, b_mod, length_idx);

    for (float phase = 0.0f; phase < 1.0f; phase += 0.01f)
    {
      float diff = fabsf(stepped_shape_with(phase, a_shape, a_mod, length_idx, &da, &na) -
                         stepped_shape_with(phase, b_shape, b_mod, length_idx, &db, &nb));
      if (diff > worst)
        worst = diff;
    }
  }
  return worst;
}

TEST_CASE(a_small_turn_never_leaps_the_output)
{
  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    float limit = (st_length_for_index(li) < 8) ? ST_SMALL_TURN_LIMIT_SHORT : ST_SMALL_TURN_LIMIT;
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
  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    int length = st_length_for_index(li);

    for (int mi = 0; mi <= 20; mi++)
    {
      float mod = -1.0f + 2.0f * (float) mi / 20.0f;

      for (int si = 0; si <= 1000; si++)
      {
        float shape = -1.0f + 2.0f * (float) si / 1000.0f;
        float lo = 9.0f, hi = -9.0f;
        for (int k = 0; k < length; k++)
        {
          float v = st_eval(((float) k + 0.5f) / (float) length, shape, mod, li, ST_HOLD_HARD);
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
      for (int li = 0; li < ST_LENGTH_COUNT; li++)
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

// Where one cycle sits, and how far it swings, at one setting of both knobs.
static void cycle_level(float shape, float mod, int length_idx, float* dc, float* span)
{
  const int n = 64;
  float lo = 1e9f, hi = -1e9f, sum = 0.0f;
  for (int i = 0; i < n; i++)
  {
    float v = st_eval((float) i / (float) n, shape, mod, length_idx, ST_HOLD_SMOOTH);
    if (v < lo)
      lo = v;
    if (v > hi)
      hi = v;
    sum += v;
  }
  *dc   = sum / (float) n;
  *span = hi - lo;
}

// The pattern the bias starts from has to be centred, whatever the knobs are
// doing. SHP does walk the finished value off centre - that is what its bias
// axis is for - but a lean applied to a pattern that was already leaning would
// lean differently at every setting, which is the inconsistency this whole
// correction exists to remove.
//
// It used to: the correction was anchored on slot 0 and expanded about it, so
// the whole cycle's DC followed slot 0's own value as the orbit turned, and one
// sweep of SHP moved it by up to 1.5 of the 2.0 range. Centring the correction
// instead brought the worst |DC| anywhere on this grid from 0.78 to 0.64 and
// the RMS from 0.25 to 0.15.
//
// The worst case left is a twelve-step pattern at high MOD, where most steps
// tie to one value and there is genuinely no centre to find. That is why this
// bounds the RMS as well as the worst point: one lopsided corner is a property
// of the pattern, a whole knob that leans is not.
TEST_CASE(the_pattern_under_the_bias_stays_centred)
{
  float worst = 0.0f, sq = 0.0f;
  int n = 0;

  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    for (int si = 0; si <= 20; si++)
    {
      for (int mi = 0; mi <= 12; mi++)
      {
        float dc, span;
        cycle_level(-1.0f + 2.0f * (float) si / 20.0f, -1.0f + 2.0f * (float) mi / 12.0f, li, &dc, &span);
        if (fabsf(dc) > worst)
          worst = fabsf(dc);
        sq += dc * dc;
        n++;
      }
    }
  }

  CHECK(worst < 0.80f);
  CHECK(sqrtf(sq / (float) n) < 0.20f);
}

// The same idea for the other half of level: the pattern the bias starts from
// may not be much louder or quieter at one setting than at its neighbours. The normalisation used to
// lift collapsed patterns and never shrink wide ones, and SHP carried a lever
// that ducked the peak-to-peak outright, so the range across the grid was
// 0.58..1.99 of 2.0 - the shape audibly flattening and swelling as the knob
// turned. It is 0.78..1.73 now from five steps up.
//
// Deliberately not pinned: ST_NORM_EXP leaves the natural ordering between calm
// and emphatic settings audible, and three or four values cannot be spread as
// evenly as thirty-two.
TEST_CASE(the_pattern_under_the_bias_is_about_as_loud_everywhere)
{
  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    float floor_span = (st_length_for_index(li) < 5) ? 0.5f : 0.7f;

    for (int si = 0; si <= 20; si++)
    {
      for (int mi = 0; mi <= 12; mi++)
      {
        float dc, span;
        cycle_level(-1.0f + 2.0f * (float) si / 20.0f, -1.0f + 2.0f * (float) mi / 12.0f, li, &dc, &span);
        CHECK(span > floor_span);
        CHECK(span < 1.85f);
      }
    }
  }
}

// The engine cannot measure a whole pattern every sample, so it measures one
// slot per tick and corrects with what the last full pass found. Whatever that
// costs, it has to arrive at the same answer a full measurement would - or
// every property in this file describes a shape the module does not play.
//
// Ticks at 4 kHz, which is the engine's own rate; a pass is `length` of them.
#define TICK_S (1.0f / 4000.0f)

// What engine.c gives out: one measurement per tick, shared between channels.
#define N_SCAN_TURNS 8

TEST_CASE(the_rolling_measurement_lands_where_a_full_one_does)
{
  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.31f)
    {
      for (float mod = -1.0f; mod <= 1.0f; mod += 0.47f)
      {
        StScan s    = {0};
        StNorm want = st_norm_exact(shape, mod, li);

        // A channel's first tick in a stepped mode has nothing measured yet, so
        // it takes the full route rather than playing a cycle uncorrected.
        st_norm_scan(&s, shape, mod, li, TICK_S, 1);
        CHECK_NEAR(s.norm.gain, want.gain, 1e-4);
        CHECK_NEAR(s.norm.offset, want.offset, 1e-4);

        // and it stays there: every later pass measures the same standing
        // pattern and asks for the same correction.
        for (int i = 0; i < 4000; i++)
        {
          st_norm_scan(&s, shape, mod, li, TICK_S, 1);
        }
        CHECK_NEAR(s.norm.gain, want.gain, 1e-3);
        CHECK_NEAR(s.norm.offset, want.offset, 1e-3);

        // Once settled it stops measuring, since re-measuring a standing
        // pattern can only give the answer it already has. So the knobs moving
        // afterwards has to wake it up again - a channel left corrected for a
        // pattern it is no longer playing is what that optimisation risks, and
        // it would be silent until someone turned a knob and heard the level
        // sit wrong.
        float moved_shape = shape + 0.37f, moved_mod = mod - 0.21f;
        StNorm want_moved = st_norm_exact(moved_shape, moved_mod, li);
        for (int i = 0; i < 4000; i++)
        {
          st_norm_scan(&s, moved_shape, moved_mod, li, TICK_S, 1);
        }
        CHECK_NEAR(s.norm.gain, want_moved.gain, 1e-3);
        CHECK_NEAR(s.norm.offset, want_moved.offset, 1e-3);
      }
    }
  }
}

// The engine hands out one measurement per tick across all eight channels, so a
// channel gets every eighth tick at worst. It still has to arrive - a rota that
// converges only when a channel has the module to itself would leave a patch
// with eight stepped channels permanently mis-corrected.
TEST_CASE(a_channel_that_only_gets_every_eighth_tick_still_arrives)
{
  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    StScan s    = {0};
    StNorm want = st_norm_exact(-0.4f, 0.6f, li);

    // past the first tick, which measures in full, and onto a different setting
    // so that what follows has something to measure
    st_norm_scan(&s, 0.0f, 0.0f, li, TICK_S, 0);

    for (int i = 0; i < 40000; i++)
    {
      st_norm_scan(&s, -0.4f, 0.6f, li, TICK_S, (i % N_SCAN_TURNS) == 0);
    }
    CHECK_NEAR(s.norm.gain, want.gain, 1e-3);
    CHECK_NEAR(s.norm.offset, want.offset, 1e-3);
  }
}

// The budget is the caller's to give: a channel told it may not measure must
// not, or the rota above is decoration.
TEST_CASE(a_channel_that_never_gets_a_turn_never_measures)
{
  StScan s = {0};
  st_norm_scan(&s, 0.2f, 0.1f, 8, TICK_S, 0); // the first tick, measured in full
  StNorm first = s.norm;

  for (int i = 0; i < 1000; i++)
  {
    st_norm_scan(&s, -0.7f, 0.9f, 8, TICK_S, 0);
  }
  CHECK(s.norm.gain == first.gain);
  CHECK(s.norm.offset == first.offset);
}

// The measurement is slewed into place rather than swapped in, so that a pass
// whose pattern changed under it cannot step the output. Pattern length is the
// hard case: it swaps the whole pattern at once, and the engine does it on the
// cycle wrap precisely because the signal must not jump there.
TEST_CASE(a_length_change_moves_the_correction_without_stepping_the_output)
{
  for (int from = 0; from < ST_LENGTH_COUNT; from++)
  {
    for (int to = 0; to < ST_LENGTH_COUNT; to++)
    {
      StScan s = {0};
      for (int i = 0; i < 500; i++)
      {
        st_norm_scan(&s, 0.3f, -0.2f, from, TICK_S, 1);
      }

      // The correction itself, tick by tick, and the value at the wrap with it.
      // The gain is what would step if a completed pass were swapped in rather
      // than slewed - the wrap value would not, since it lands on the shared
      // constant whatever the gain is.
      float prev      = stepped_shape_with(0.0f, 0.3f, -0.2f, from, &(StDrive){ST_HOLD_SMOOTH, 0.0f, 0.0f}, &s.norm);
      float prev_gain = s.norm.gain;
      for (int i = 0; i < 2000; i++)
      {
        st_norm_scan(&s, 0.3f, -0.2f, to, TICK_S, 1);
        float v = stepped_shape_with(0.0f, 0.3f, -0.2f, to, &(StDrive){ST_HOLD_SMOOTH, 0.0f, 0.0f}, &s.norm);
        CHECK(fabsf(v - prev) < 0.05f);
        CHECK(fabsf(s.norm.gain - prev_gain) < 0.1f);
        prev      = v;
        prev_gain = s.norm.gain;
      }

      // and it ends up where a full measurement of the new length would put it.
      // It glides there rather than arriving: the change restarts the pass
      // instead of paying for a full measurement in one tick, so the level
      // takes a pass plus the slew to settle - tens of milliseconds.
      StNorm want = st_norm_exact(0.3f, -0.2f, to);
      CHECK_NEAR(s.norm.gain, want.gain, 1e-3);
    }
  }
}

// ---- as the module drives it ----------------------------------------------------
//
// Everything above measures the pattern and its correction, at a fixed ease and
// no reshaping. These measure the shape as a channel actually plays it: SHP
// reshaping the distribution, MOD carrying the ease as well as the density.

// One cycle of a styled channel, the way channel.c plays it.
static void driven_cycle(float shape, float mod, int length_idx, int n, float* out)
{
  StDrive d   = st_drive(shape, mod);
  StNorm norm = st_norm_exact(shape, mod, length_idx);
  for (int i = 0; i < n; i++)
  {
    out[i] = stepped_shape_with((float) i / (float) n, shape, mod, length_idx, &d, &norm);
  }
}

// What the control style is for. Measured on one pattern at a time, with only
// the bias changed: SHP moves the pattern as well, and pattern-to-pattern
// variation in these numbers is larger than the reshaping, so comparing two
// settings of SHP measures mostly which patterns they landed on.
//
// Read at a setting where MOD holds the steps rather than slewing between them,
// since a curve on its way from one value to the next passes through the middle
// whatever the values are.
TEST_CASE(the_bias_leans_the_distribution_the_value_path_cannot)
{
  const int n = 2048;
  static float plain[2048], low[2048], gate[2048];

  for (int li = 4; li < ST_LENGTH_COUNT; li++) // from eight steps up
  {
    for (float shape = -0.6f; shape <= 0.61f; shape += 0.4f)
    {
      StDrive d   = st_drive(shape, 0.5f);
      StNorm norm = st_norm_exact(shape, 0.5f, li);
      // the depth the mode itself reaches, rather than a copy of it here
      float depth = st_drive(1.0f, 0.0f).bias;

      for (int i = 0; i < n; i++)
      {
        float p  = (float) i / (float) n;
        plain[i] = stepped_shape_with(p, shape, 0.5f, li, &(StDrive){d.hold, 0.0f, 0.0f}, &norm);
        low[i]   = stepped_shape_with(p, shape, 0.5f, li, &(StDrive){d.hold, -depth, 0.0f}, &norm);
        gate[i]  = stepped_shape_with(p, shape, 0.5f, li, &(StDrive){d.hold, depth, 0.0f}, &norm);
      }

      float plain_mean = 0.0f, low_mean = 0.0f, low_peak = -9.0f;
      int plain_middling = 0, gate_middling = 0;
      for (int i = 0; i < n; i++)
      {
        plain_mean += plain[i];
        low_mean += low[i];
        if (low[i] > low_peak)
          low_peak = low[i];
        if (fabsf(plain[i]) < 0.3f)
          plain_middling++;
        if (fabsf(gate[i]) < 0.3f)
          gate_middling++;
      }

      // sits low - and still reaches above where it sits, which is the
      // difference between a bias and an offset. Measured worst case across
      // this grid is 0.264 of the 2.0 range; the flat-floor test above is what
      // keeps the whole swing honest.
      CHECK(low_mean < plain_mean - 0.2f * (float) n);
      CHECK(low_peak - low_mean / (float) n > 0.2f);

      // and at the other end, less of the time spent anywhere near the middle
      CHECK(gate_middling < plain_middling);
    }
  }
}

// SHP is what drives it, one traversal, ends first.
TEST_CASE(shp_is_the_bias_axis)
{
  CHECK(st_drive(-1.0f, 0.0f).bias < -0.5f);
  CHECK(st_drive(0.0f, 0.0f).bias == 0.0f);
  CHECK(st_drive(1.0f, 0.0f).bias > 0.5f);

  float prev = -9.0f;
  for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
  {
    float bias = st_drive(shape, 0.0f).bias;
    CHECK(bias > prev); // monotone, so neighbouring positions stay neighbours
    prev = bias;
  }
}

// MOD carries the ease as well as the density, so one knob goes from a curve
// that never stops moving to one that sits still between steps.
TEST_CASE(mod_carries_the_ease)
{
  const int n = 2000;
  static float smooth[2000], hard[2000];
  driven_cycle(-1.0f, -1.0f, 6, n, smooth);
  driven_cycle(-1.0f, 1.0f, 6, n, hard);

  int still_smooth = 0, still_hard = 0;
  for (int i = 1; i < n; i++)
  {
    if (fabsf(smooth[i] - smooth[i - 1]) < 1e-5f)
      still_smooth++;
    if (fabsf(hard[i] - hard[i - 1]) < 1e-5f)
      still_hard++;
  }
  CHECK(still_hard > still_smooth * 2);
}

// The invariants the bare shape holds, the driven one holds too: in range, and
// never collapsing to a flat output. A monotone reshaping of the finished value
// cannot break either, which is why the bias is applied where it is - but
// "cannot" is worth a test.
TEST_CASE(the_driven_shape_keeps_every_invariant_the_bare_one_has)
{
  const int n = 256;
  static float cycle[256];

  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    for (float shape = -1.0f; shape <= 1.0f; shape += 0.13f)
    {
      for (float mod = -1.0f; mod <= 1.0f; mod += 0.29f)
      {
        driven_cycle(shape, mod, li, n, cycle);

        float lo = 9.0f, hi = -9.0f;
        for (int i = 0; i < n; i++)
        {
          if (cycle[i] < lo)
            lo = cycle[i];
          if (cycle[i] > hi)
            hi = cycle[i];
        }

        CHECK(lo >= -1.0001f && hi <= 1.0001f);
        CHECK(hi - lo > 0.5f);
      }
    }
  }
}

// Continuity, including across the loop point, swept finely enough to see it.
// A hold-heavy setting crosses most of a step in a fraction of it, so a coarse
// sweep reads its own sampling as a jump - which is why the melodic case walks
// 20000 points too.
TEST_CASE(the_driven_curve_is_continuous_everywhere)
{
  const int n = 20000;

  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    for (float mod = -1.0f; mod <= 1.0f; mod += 0.5f)
    {
      StDrive d   = st_drive(0.3f, mod);
      StNorm norm = st_norm_exact(0.3f, mod, li);

      float prev = stepped_shape_with(0.0f, 0.3f, mod, li, &d, &norm);
      for (int i = 1; i <= n; i++)
      {
        float v = stepped_shape_with((float) i / (float) n, 0.3f, mod, li, &d, &norm);
        CHECK(fabsf(v - prev) < 0.15f);
        prev = v;
      }
    }
  }
}

static int direction_changes(int length_idx)
{
  int turns        = 0;
  float prev       = st_flat(0.0f, 0.25f, length_idx, ST_HOLD_SEMI);
  float prev_slope = 0.0f;
  for (int i = 1; i <= 4096; i++)
  {
    float v     = st_flat((float) i / 4096.0f, 0.25f, length_idx, ST_HOLD_SEMI);
    float slope = v - prev;
    if (slope * prev_slope < 0.0f)
      turns++;
    if (slope != 0.0f)
      prev_slope = slope;
    prev = v;
  }
  return turns;
}

TEST_CASE(longer_patterns_contain_more_events) { CHECK(direction_changes(ST_LENGTH_COUNT - 1) > direction_changes(0) * 4); }

TEST_CASE(hold_setting_controls_how_step_like_the_curve_is)
{
  int still_smooth = 0, still_hard = 0;
  const int n = 4000;
  for (int i = 1; i < n; i++)
  {
    float p  = (float) i / (float) n;
    float pp = (float) (i - 1) / (float) n;
    if (fabsf(st_flat(p, 0.4f, 6, ST_HOLD_SMOOTH) - st_flat(pp, 0.4f, 6, ST_HOLD_SMOOTH)) < 1e-5f)
      still_smooth++;
    if (fabsf(st_flat(p, 0.4f, 6, ST_HOLD_HARD) - st_flat(pp, 0.4f, 6, ST_HOLD_HARD)) < 1e-5f)
      still_hard++;
  }
  CHECK(still_hard > still_smooth * 2);
}

TEST_CASE(length_index_maps_to_the_curated_step_counts)
{
  CHECK(st_length_for_index(0) == 3);
  CHECK(st_length_for_index(ST_LENGTH_COUNT - 1) == 64);
  // out-of-range indices must not read off the end of the tables
  CHECK(st_length_for_index(-5) == 3);
  CHECK(st_length_for_index(999) == 64);
}

// SHP -1 is the character-neutral end of the sweep: the contour blend and the
// value shaper are both at zero there, so the steps are the plain orbit values.
// The MOD tests below read the density off that, the way the SHP tests above
// read the pattern off MOD 0 - each parameter measured with the other one out
// of the way.
#define ST_SHAPE_NEUTRAL (-1.0f)

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
  int length  = st_length_for_index(length_idx);
  int changes = 0;
  float prev  = st_eval(0.4f / (float) length, ST_SHAPE_NEUTRAL, mod, length_idx, ST_HOLD_HARD);

  for (int i = 1; i < length; i++)
  {
    float v = st_eval(((float) i + 0.4f) / (float) length, ST_SHAPE_NEUTRAL, mod, length_idx, ST_HOLD_HARD);
    if (fabsf(v - prev) > 0.1f)
      changes++;
    prev = v;
  }
  return changes;
}

// Peak-to-peak of the pattern's steps, and how far apart consecutive steps sit.
static void pattern_steps(float shape, float mod, int length_idx, float* out)
{
  int length = st_length_for_index(length_idx);
  for (int i = 0; i < length; i++)
  {
    out[i] = st_eval(((float) i + 0.4f) / (float) length, shape, mod, length_idx, ST_HOLD_HARD);
  }
}

// Mean distance between consecutive steps: low is a melodic walk, high is leaps.
static float mean_interval(float shape, int length_idx)
{
  float v[64];
  int length = st_length_for_index(length_idx);
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
  int length = st_length_for_index(length_idx);
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
  int length = st_length_for_index(li);
  float prev = st_eval(0.4f / (float) length, ST_SHAPE_NEUTRAL, -1.0f, li, ST_HOLD_HARD);
  for (int i = 1; i < length; i++)
  {
    float v = st_eval(((float) i + 0.4f) / (float) length, ST_SHAPE_NEUTRAL, -1.0f, li, ST_HOLD_HARD);
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
    for (int li = 0; li < ST_LENGTH_COUNT; li++)
    {
      for (int h = 0; h < HOLD_COUNT; h++)
      {
        // Same value either side of the loop point, so the cycle still closes.
        float at_end   = st_eval(0.99999f, 0.3f, mod, li, HOLDS[h]);
        float at_start = st_eval(0.0f, 0.3f, mod, li, HOLDS[h]);
        CHECK(fabsf(at_end - at_start) < 0.02f);

        for (float p = 0.0f; p < 1.0f; p += 0.02f)
        {
          float v = st_eval(p, 0.3f, mod, li, HOLDS[h]);
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
    for (int li = 0; li < ST_LENGTH_COUNT; li++)
    {
      float worst = 1e9f;
      for (float shape = -1.0f; shape <= 1.0f; shape += 0.05f)
      {
        float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < 512; i++)
        {
          float v = st_eval((float) i / 512.0f, shape, mod, li, ST_HOLD_SEMI);
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
  for (int li = 6; li < ST_LENGTH_COUNT; li++) // 16 steps and up
  {
    float quietest = 1e9f;
    for (float mod = -1.0f; mod < 0.99f; mod += 0.01f)
    {
      float sum = 0.0f;
      for (int i = 0; i < 256; i++)
      {
        float p = (float) i / 256.0f;
        float d = st_eval(p, 0.3f, mod, li, ST_HOLD_SMOOTH) - st_eval(p, 0.3f, mod + 0.01f, li, ST_HOLD_SMOOTH);
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
  for (int li = 6; li < ST_LENGTH_COUNT; li++)
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
  for (int li = 6; li < ST_LENGTH_COUNT; li++)
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
      float swing = st_swing_amount(mod);
      float within;

      // phase 1.0 is the end of the last step
      CHECK(st_step_at(1.0f, length, swing, &within) == length - 1);
      CHECK_NEAR(within, 1.0f, 1e-4);

      // phase 0 is the start of the first
      CHECK(st_step_at(0.0f, length, swing, &within) == 0);
      CHECK_NEAR(within, 0.0f, 1e-4);

      // every step is entered exactly once, in order, and `within` runs 0..1
      int seen = 0, last = -1;
      float prev_within = 0.0f;
      for (int i = 0; i <= 4096; i++)
      {
        int step = st_step_at((float) i / 4096.0f, length, swing, &within);
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
  CHECK_NEAR(st_swing_amount(0.0f), 0.0f, 1e-4);
  CHECK_NEAR(st_swing_amount(-1.0f), 0.0f, 1e-4);
  CHECK_NEAR(st_swing_amount(1.0f), 0.0f, 1e-4);
  CHECK(st_swing_amount(0.5f) * st_swing_amount(-0.5f) < 0.0f); // opposite ways
  CHECK(fabsf(st_swing_amount(0.5f)) > 0.2f);                   // and not subtle
}

int main(void)
{
  RUN_TEST(output_stays_in_bipolar_range);
  RUN_TEST(is_deterministic_for_same_inputs);
  RUN_TEST(measuring_the_correction_here_or_passing_it_in_are_the_same_shape);
  RUN_TEST(phase_wraps_seamlessly_across_the_loop_point);
  RUN_TEST(changing_length_at_the_cycle_boundary_is_seamless);
  RUN_TEST(curve_is_continuous_at_every_step_boundary);
  RUN_TEST(a_small_turn_deforms_the_pattern_rather_than_replacing_it);
  RUN_TEST(a_small_turn_never_leaps_the_output);
  RUN_TEST(no_narrow_notch_collapses_the_output);
  RUN_TEST(no_setting_collapses_to_a_flat_output);
  RUN_TEST(the_pattern_under_the_bias_stays_centred);
  RUN_TEST(the_pattern_under_the_bias_is_about_as_loud_everywhere);
  RUN_TEST(the_rolling_measurement_lands_where_a_full_one_does);
  RUN_TEST(a_channel_that_only_gets_every_eighth_tick_still_arrives);
  RUN_TEST(a_channel_that_never_gets_a_turn_never_measures);
  RUN_TEST(a_length_change_moves_the_correction_without_stepping_the_output);
  RUN_TEST(the_bias_leans_the_distribution_the_value_path_cannot);
  RUN_TEST(shp_is_the_bias_axis);
  RUN_TEST(mod_carries_the_ease);
  RUN_TEST(the_driven_shape_keeps_every_invariant_the_bare_one_has);
  RUN_TEST(the_driven_curve_is_continuous_everywhere);
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
