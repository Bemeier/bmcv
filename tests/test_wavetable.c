// The SHAPE_LFO wavetable, and the properties it is generated to have.
//
// These are asserted against the *lookup*, not against the table, because the
// lookup is what a channel hears: it interpolates across both axes, so a knob
// sitting between two slices is the normal case rather than an edge one. The
// table the hand-drawn version replaced had every slice at full scale and still
// went silent at one setting, because nothing had ever checked what happened
// between two of them.

#include "config.h"
#include "pll_metrics.h" // the clock driver, for the divider case
#include "testkit.h"
#include "wavetable.h"
#include "wavetables.h"
#include <math.h>

#define STEPS 512 // phase resolution the properties are measured at

static float shape_of_slice(int i) { return 2.0f * (float) i / (float) WT_SLICES - 1.0f; }

// Peak-to-peak of one cycle at a given shape, in full-scale units where 2.0 is
// the whole converter range.
static float swing_at(float shape, float* out_dc)
{
  float lo = 1e9f, hi = -1e9f, sum = 0.0f;
  for (int n = 0; n < STEPS; n++)
  {
    float v = wavetable_lookup((float) n / STEPS, shape) / 32767.0f;
    if (v < lo)
      lo = v;
    if (v > hi)
      hi = v;
    sum += v;
  }
  if (out_dc)
    *out_dc = sum / STEPS;
  return hi - lo;
}

// The property the whole generator exists to guarantee. Swept finely enough to
// land between slices many times over, and across the whole parameter range
// including both ends.
TEST_CASE(every_shape_setting_reaches_the_full_swing)
{
  const int n    = WT_SLICES * 16;
  float worst    = 2.0f;
  float worst_at = 0.0f;

  for (int i = 0; i <= n; i++)
  {
    float shape = -1.0f + 2.0f * (float) i / (float) n;
    float pp    = swing_at(shape, NULL);
    if (pp < worst)
    {
      worst    = pp;
      worst_at = shape;
    }
  }

  // 1.98 of 2.0: the only thing between a blend and the full swing is int16
  // rounding, so this has a lot of room in it and still catches any cancellation.
  CHECK(worst > 1.98f);
  if (worst <= 1.98f)
    fprintf(stderr, "  worst swing %.4f at shape %+.4f\n", (double) worst, (double) worst_at);
}

// The specific failure that prompted the rework: two neighbouring slices in
// opposite phase, whose midpoint cancels to nothing. It measured 0.0003 of a
// possible 2.0 - the channel was silent at that setting.
TEST_CASE(no_pair_of_neighbouring_slices_cancels)
{
  for (int i = 0; i < WT_SLICES; i++)
  {
    // Halfway between slice i and slice i+1, the wrap included.
    float shape = shape_of_slice(i) + 1.0f / (float) WT_SLICES;
    CHECK(swing_at(shape, NULL) > 1.98f);
  }
}

// Half-wave antisymmetry gives this for free, and it is what stops a sweep of
// SHP walking the channel's average level. The drawn table moved it by up to
// 0.7 of full scale.
TEST_CASE(no_shape_setting_has_a_dc_offset)
{
  const int n = WT_SLICES * 8;
  for (int i = 0; i <= n; i++)
  {
    float shape = -1.0f + 2.0f * (float) i / (float) n;
    float dc    = 0.0f;
    swing_at(shape, &dc);
    CHECK(fabsf(dc) < 0.01f);
  }
}

// The canonical shapes, in their canonical form and to the resolution the table
// is stored at - phase 0 is the rising edge, so the sine is sin(2*pi*p) rather
// than -cos. One LSB is
// 1/32767 = 0.00003 in these units, so this is asserting "exact" with a margin
// for the rounding and for the lookup's own interpolation.
TEST_CASE(the_sine_slice_is_a_sine)
{
  float shape = shape_of_slice(WT_SLICE_SINE);
  CHECK_NEAR(shape, 0.0, 1e-6); // and it is what a channel resets to

  float err = 0.0f;
  for (int n = 0; n < STEPS; n++)
  {
    float p    = (float) n / STEPS;
    float want = sinf(2.0f * (float) M_PI * p);
    float got  = wavetable_lookup(p, shape) / 32767.0f;
    err += (got - want) * (got - want);
  }
  CHECK(sqrtf(err / STEPS) < 0.001f);
}

TEST_CASE(the_triangle_slice_is_a_triangle)
{
  float shape = shape_of_slice(WT_SLICE_TRIANGLE);

  float err = 0.0f;
  for (int n = 0; n < STEPS; n++)
  {
    float p = (float) n / STEPS;
    float want;
    if (p < 0.25f)
      want = 4.0f * p; // up from the rising zero crossing
    else if (p < 0.75f)
      want = 2.0f - 4.0f * p; // down through zero to the trough
    else
      want = 4.0f * p - 4.0f; // back up to meet the start
    float got = wavetable_lookup(p, shape) / 32767.0f;
    err += (got - want) * (got - want);
  }
  CHECK(sqrtf(err / STEPS) < 0.001f);
}

// Every wave crosses zero rising at phase 0, peaks a quarter cycle later and
// troughs at three quarters. Two things rest on it:
//
//   * it is the anchoring the amplitude guarantee is built on - all slices
//     agreeing on *where* the extremes are is what makes a blend of any two of
//     them reach the same extremes;
//   * it is what makes PHS mean one thing across all three shape modes. The
//     event is on the beat in SHAPE_PWM (the gate opens at phase 0) and in
//     SHAPE_STEPPED (step 0 begins at phase 0), and now here too.
TEST_CASE(every_shape_has_its_rising_edge_at_phase_zero)
{
  const int n = WT_SLICES * 4;
  for (int i = 0; i <= n; i++)
  {
    float shape = -1.0f + 2.0f * (float) i / (float) n;

    CHECK(fabsf(wavetable_lookup(0.00f, shape) / 32767.0f) < 0.01f); // rising through zero
    CHECK(wavetable_lookup(0.25f, shape) / 32767.0f > 0.99f);        // peak
    CHECK(fabsf(wavetable_lookup(0.50f, shape) / 32767.0f) < 0.01f); // falling through zero
    CHECK(wavetable_lookup(0.75f, shape) / 32767.0f < -0.99f);       // trough

    // ...and it is genuinely rising through zero, not falling through it.
    // Sampled a tenth of a cycle out rather than right against the crossing:
    // the pointiest shapes are so flat there that both sides quantise to zero,
    // which says nothing about the direction either way.
    CHECK(wavetable_lookup(0.10f, shape) > wavetable_lookup(0.90f, shape));
  }
}

// The square, specifically, because it has a job: a channel in this shape at a
// whole-number ratio is a clock divider, and a divider whose edge is not on the
// beat is not one. The rising edge has to be at phase 0 and the gate has to be
// half the cycle wide.
TEST_CASE(the_square_is_a_gate_that_opens_on_the_beat)
{
  float shape = shape_of_slice(WT_SLICE_SQUARE);

  // Hard against the edge from both sides: low just before phase 0, high just
  // after. One sample of the table is 1/WT_LEN, so this is as close as the
  // stored data can be asked about.
  CHECK(wavetable_lookup(1.0f - 1.5f / WT_LEN, shape) / 32767.0f < -0.9f);
  CHECK(wavetable_lookup(1.5f / WT_LEN, shape) / 32767.0f > 0.9f);

  // Fifty percent duty, and flat in between - a gate, not a ramp.
  for (int n = 0; n < STEPS; n++)
  {
    float p = ((float) n + 0.5f) / STEPS;
    float v = wavetable_lookup(p, shape) / 32767.0f;

    if (p > 0.02f && p < 0.48f)
      CHECK(v > 0.9f);
    else if (p > 0.52f && p < 0.98f)
      CHECK(v < -0.9f);
  }
}

// The shape axis is a loop: shape +1 lands on the same slice as shape -1, and
// the slices either side of the seam have to be neighbours like any other.
TEST_CASE(the_shape_axis_closes_on_itself)
{
  for (int n = 0; n < STEPS; n++)
  {
    float p = (float) n / STEPS;
    CHECK_NEAR(wavetable_lookup(p, 1.0f), wavetable_lookup(p, -1.0f), 1.0);
  }

  // ...and the seam is not a cliff. Measured against the rest of the table
  // rather than against an absolute number: the slices are spaced evenly in
  // RMS within each leg but the legs have different densities, so what matters
  // is that the step across the wrap is an ordinary step and not a special one.
  float worst_step = 0.0f;
  float wrap_step  = 0.0f;

  for (int i = 0; i < WT_SLICES; i++)
  {
    float acc = 0.0f;
    for (int n = 0; n < STEPS; n++)
    {
      float p = (float) n / STEPS;
      float a = wavetable_lookup(p, shape_of_slice(i)) / 32767.0f;
      float b = wavetable_lookup(p, shape_of_slice((i + 1) % WT_SLICES)) / 32767.0f;
      acc += (a - b) * (a - b);
    }
    float step = sqrtf(acc / STEPS);

    if (i == WT_SLICES - 1)
      wrap_step = step;
    else if (step > worst_step)
      worst_step = step;
  }

  CHECK(wrap_step <= 1.5f * worst_step);
  if (wrap_step > 1.5f * worst_step)
    fprintf(stderr, "  wrap step %.4f vs worst elsewhere %.4f\n", (double) wrap_step, (double) worst_step);
}

// A cycle has to join itself. Stated as "the wrap is not special" rather than
// "the wrap is small", because for the square it is not small: phase 0 is the
// rising edge, so the steepest slope in the whole cycle sits exactly on the
// join. That is the convention working, not a discontinuity - and the way to
// tell the difference is that the falling edge, in the middle of the cycle, is
// just as steep.
TEST_CASE(each_cycle_closes_without_a_step)
{
  const int n = WT_SLICES * 4;
  for (int i = 0; i <= n; i++)
  {
    float shape = -1.0f + 2.0f * (float) i / (float) n;

    float biggest_inside = 0.0f;
    float prev           = wavetable_lookup(0.0f, shape) / 32767.0f;
    for (int k = 1; k < WT_LEN; k++)
    {
      float v = wavetable_lookup((float) k / WT_LEN, shape) / 32767.0f;
      float d = fabsf(v - prev);
      if (d > biggest_inside)
        biggest_inside = d;
      prev = v;
    }

    float across = fabsf(wavetable_lookup(0.0f, shape) / 32767.0f - prev);
    CHECK(across <= biggest_inside + 0.01f);
  }
}

// The lookup is bounded whatever it is handed. A phase outside [0,1) used to
// index the table from an arbitrary address.
TEST_CASE(the_lookup_is_bounded_for_any_input)
{
  const float wild[]   = {-100.0f, -1.5f, -0.001f, 0.0f, 0.999f, 1.0f, 1.5f, 100.0f};
  const float shapes[] = {-100.0f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f, 100.0f};

  for (unsigned a = 0; a < sizeof(wild) / sizeof(wild[0]); a++)
  {
    for (unsigned b = 0; b < sizeof(shapes) / sizeof(shapes[0]); b++)
    {
      float v = wavetable_lookup(wild[a], shapes[b]);
      CHECK(v >= -32768.0f && v <= 32768.0f);
    }
  }
}

// The use case, end to end: a channel in the wavetable shape mode, set to the
// square and to a whole-number division of the beat, is a clock divider.
//
// Everything above is about the table. This is about what comes out of a jack:
// the engine's phase lock puts phase 0 on the beat, and the square puts its
// rising edge on phase 0, so the gate opens on the beat and stays open for half
// the division. With the edge a quarter cycle late - which is where it was -
// a divide-by-four gate opened a whole beat after the one it was dividing.
static Fixture fx;

// One division. Returns the number of rising edges, how many of them landed on
// a beat boundary, and how many beats went by.
static void run_divider(int div, uint32_t* edges, uint32_t* on_beat, uint64_t* beats)
{
  PllClock clk;
  fixture_init(&fx);

  fixture_set_param(&fx, 0, 0, CH_PARAM_AMP, INT16_MAX);
  fixture_set_param(&fx, 0, 0, CH_PARAM_FRQ, (int16_t) lrintf((1.0f - (float) div) * 255.0f));
  fixture_set_param(&fx, 0, 0, CH_PARAM_SHP, -INT16_MAX); // shape -1: the square

  pll_clock_init(&clk, 120.0f);
  pll_run(&fx, &clk, 0, 20.0f, NULL); // acquire the lock

  *edges   = 0;
  *on_beat = 0;
  *beats   = 0;

  int16_t prev = fx.engine_state.channels_output_level[0];
  for (uint32_t i = 0; i < 64000; i++) // 16 seconds, 32 beats at 120bpm
  {
    uint64_t before = fx.engine_state.clock.beat_counter;
    pll_step(&fx, &clk, 0, ENGINE_TICK_US, NULL);
    if (fx.engine_state.clock.beat_counter != before)
      (*beats)++;

    int16_t now = fx.engine_state.channels_output_level[0];
    if (prev < 0 && now >= 0)
    {
      (*edges)++;
      float bp = fx.engine_state.clock.beat_phase;
      if (bp < 0.02f || bp > 0.98f)
        (*on_beat)++;
    }
    prev = now;
  }
}

TEST_CASE(a_square_channel_divides_the_clock)
{
  // Two divisions, and the odd one matters. At divide-by-four a quarter of a
  // cycle is a whole beat, so even the old quarter-late edge landed on *a* beat
  // - the wrong one, but a beat - and a test that only checked ÷4 would have
  // passed either way. At divide-by-two a quarter cycle is half a beat, which
  // puts the old alignment squarely between two beats.
  const int divisions[] = {2, 4};

  for (unsigned k = 0; k < sizeof(divisions) / sizeof(divisions[0]); k++)
  {
    uint32_t edges, on_beat;
    uint64_t beats;
    run_divider(divisions[k], &edges, &on_beat, &beats);

    CHECK(beats > 28);
    CHECK(edges > 4);

    // Every gate opens on a beat, not near one.
    CHECK(edges == on_beat);

    // ...and one gate per division, not per beat.
    CHECK_NEAR((double) beats / (double) edges, (double) divisions[k], 0.35);

    if (edges != on_beat)
      fprintf(stderr, "  div %d: %u edges, %u on a beat\n", divisions[k], edges, on_beat);
  }
}

int main(void)
{
  RUN_TEST(every_shape_setting_reaches_the_full_swing);
  RUN_TEST(no_pair_of_neighbouring_slices_cancels);
  RUN_TEST(no_shape_setting_has_a_dc_offset);
  RUN_TEST(the_sine_slice_is_a_sine);
  RUN_TEST(the_triangle_slice_is_a_triangle);
  RUN_TEST(every_shape_has_its_rising_edge_at_phase_zero);
  RUN_TEST(the_square_is_a_gate_that_opens_on_the_beat);
  RUN_TEST(the_shape_axis_closes_on_itself);
  RUN_TEST(each_cycle_closes_without_a_step);
  RUN_TEST(the_lookup_is_bounded_for_any_input);
  RUN_TEST(a_square_channel_divides_the_clock);
  return TESTKIT_SUMMARY();
}
