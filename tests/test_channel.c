#include "clock_sync.h"
#include "fixture.h"
#include "helpers.h"
#include "stepped_table.h"
#include "testkit.h"

TEST_CASE(zero_amplitude_channel_outputs_the_offset)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 1234);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);

  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_output_level[0] == 1234);
}

TEST_CASE(amplitude_bounds_the_output_around_the_offset)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 0);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0); // multiplier 1x
  Clock_Init(&f.engine_state.clock);

  // AMP is the peak: the swing is the parameter, not half of it.
  for (int i = 0; i < 50; i++)
  {
    fixture_tick(&f, 20000);
    int16_t v = f.engine_state.channels_output_level[0];
    CHECK(v >= -20001 && v <= 20001);
  }
}

// Full scale has to reach the ends of the converter's range on its own, with no
// offset and nothing modulating it - otherwise the wider output clamps are
// unreachable from the oscillator.
TEST_CASE(full_amplitude_alone_covers_the_whole_bipolar_range)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 0);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, INT16_MAX);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0);
  Clock_Init(&f.engine_state.clock);

  int16_t hi = INT16_MIN, lo = INT16_MAX;
  for (int i = 0; i < 200; i++)
  {
    fixture_tick(&f, 5000);
    int16_t v = f.engine_state.channels_output_level[0];
    if (v > hi)
      hi = v;
    if (v < lo)
      lo = v;
  }

  // Within a percent of +/-10V at both ends.
  CHECK(hi > DAC_10V - DAC_10V / 100);
  CHECK(lo < -(DAC_10V - DAC_10V / 100));
}

TEST_CASE(input_add_mode_adds_the_raw_input_value)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 1000);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);
  f.engine_config.channel_state[0].src_input      = 0;
  f.engine_config.channel_state[0].input_amp_mode = INPUT_AMP_ADD;
  f.hw_state.input_state[0]                       = 500;

  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_output_level[0] == 1500);
}

TEST_CASE(input_mult_mode_zero_input_zeroes_the_output)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 1000);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);
  f.engine_config.channel_state[0].src_input      = 0;
  f.engine_config.channel_state[0].input_amp_mode = INPUT_AMP_MULT;
  f.hw_state.input_state[0]                       = 0;

  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_output_level[0] == 0);
}

TEST_CASE(continuous_quantize_matches_quantize_value)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 5000);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);
  f.engine_config.channel_state[0].quantize_mode = QUANTIZE_CONTINUOUS;
  f.engine_config.quantize_mask                  = 0b111111111111;

  fixture_tick(&f, 1000);

  int16_t expected = quantize_value(5000, f.engine_config.quantize_mask);
  CHECK(f.engine_state.channels_output_level[0] == expected);
}

TEST_CASE(phase_advances_by_frequency_times_dt_without_pll_lock)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0); // multiplier 1x
  Clock_Init(&f.engine_state.clock);            // have_beat=false, beat_freq_smooth=1.0Hz, no PLL correction applied

  fixture_tick(&f, 500000); // 0.5s at 1Hz -> phase advances by 0.5

  CHECK_NEAR(f.engine_state.channels_shared_phase[0], 0.5, 1e-4);
}

// ---- pattern-length latching (stepped modes) -------------------------------
//
// Switching pattern length mid-cycle moves the step grid under the playhead
// and jumps the output. So it is held until the cycle wraps - except while the
// encoder is being turned, where instant feedback matters more.

static void set_stepped_channel(Fixture* f, int8_t length_idx)
{
  f->engine_config.channel_state[0].shape_mode    = SHAPE_STEPPED;
  f->engine_config.channel_state[0].st_length_idx = length_idx;
  fixture_set_param(f, 0, 0, CH_PARAM_FRQ, 0); // 1x the beat
  fixture_set_param(f, 0, 0, CH_PARAM_AMP, 20000);
}

TEST_CASE(pattern_length_is_latched_until_the_cycle_wraps)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init(&f.engine_state.clock);
  set_stepped_channel(&f, 0); // shortest pattern
  fixture_tick(&f, 1000);
  int8_t latched = f.engine_state.channels_length_idx[0];
  CHECK(latched == 0);

  // move well past the edit window so this counts as a scene-style change
  for (int i = 0; i < 40; i++)
    fixture_tick(&f, 20000); // 0.8s, but phase only reaches ~0.8 at 1Hz

  f.engine_config.channel_state[0].st_length_idx = ST_LENGTH_COUNT - 1;
  fixture_tick(&f, 1000);

  // still mid-cycle, so the length must not have moved yet
  CHECK(f.engine_state.channels_length_idx[0] == 0);
  CHECK(f.engine_state.channels_shared_phase[0] < 1.0f);
}

TEST_CASE(pattern_length_updates_once_the_cycle_wraps)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init(&f.engine_state.clock);
  set_stepped_channel(&f, 0);
  fixture_tick(&f, 1000);
  for (int i = 0; i < 40; i++)
    fixture_tick(&f, 20000);

  f.engine_config.channel_state[0].st_length_idx = ST_LENGTH_COUNT - 1;
  // run past the wrap
  for (int i = 0; i < 80; i++)
    fixture_tick(&f, 20000);

  CHECK(f.engine_state.channels_length_idx[0] == ST_LENGTH_COUNT - 1);
}

TEST_CASE(pattern_length_applies_immediately_while_the_encoder_is_turning)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init(&f.engine_state.clock);
  set_stepped_channel(&f, 0);
  fixture_tick(&f, 1000);
  for (int i = 0; i < 40; i++)
    fixture_tick(&f, 20000);
  CHECK(f.engine_state.channels_length_idx[0] == 0);

  // mark the channel as just-edited, the way the STA-page encoder does
  f.engine_state.channels_last_delta[0]          = f.hw_state.time;
  f.engine_config.channel_state[0].st_length_idx = ST_LENGTH_COUNT - 1;
  fixture_tick(&f, 1000);

  CHECK(f.engine_state.channels_length_idx[0] == ST_LENGTH_COUNT - 1);
  CHECK(f.engine_state.channels_shared_phase[0] < 1.0f); // proved it did not wait for a wrap
}

// ---- output clamp ----------------------------------------------------------

static int16_t peak_over_a_cycle(Fixture* f, int8_t clamp_mode, int16_t* lowest)
{
  f->engine_config.channel_state[0].clamp_mode = clamp_mode;
  int16_t hi = INT16_MIN, lo = INT16_MAX;
  for (int i = 0; i < 200; i++)
  {
    fixture_tick(f, 5000);
    int16_t v = f->engine_state.channels_output_level[0];
    if (v > hi)
      hi = v;
    if (v < lo)
      lo = v;
  }
  if (lowest)
    *lowest = lo;
  return hi;
}

// A clamp, not a scaling: what was dialled in still means what it said, and the
// swing is cut off at the ends.
TEST_CASE(the_output_clamp_bounds_the_swing_without_rescaling_it)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init(&f.engine_state.clock);
  // Offset as well as amplitude, so the unclamped swing runs past the top of
  // the range at one end and below zero at the other.
  fixture_set_param(&f, 0, 0, CH_PARAM_OFS, 8000);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, INT16_MAX);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0);

  int16_t lo = 0;
  int16_t hi = peak_over_a_cycle(&f, CLAMP_BI_10, &lo);
  CHECK(hi > DAC_5V && lo < 0);

  hi = peak_over_a_cycle(&f, CLAMP_BI_5, &lo);
  CHECK(hi <= DAC_5V && lo >= -DAC_5V);
  CHECK(hi > DAC_5V / 2); // clipped at 5V, not squeezed into it

  hi = peak_over_a_cycle(&f, CLAMP_UNI_10, &lo);
  CHECK(lo == 0 && hi > DAC_5V);

  hi = peak_over_a_cycle(&f, CLAMP_UNI_5, &lo);
  CHECK(lo == 0 && hi <= DAC_5V);
}

// ---- pwm -------------------------------------------------------------------

TEST_CASE(pwm_is_a_square_whose_duty_follows_the_shape_parameter)
{
  Fixture f;
  fixture_init(&f);
  Clock_Init(&f.engine_state.clock);
  f.engine_config.channel_state[0].shape_mode = SHAPE_PWM;
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 0); // 1x the beat, so one cycle is 1s

  int high_at[3]          = {0};
  const int16_t shapes[3] = {-30000, 0, 30000};

  for (int s = 0; s < 3; s++)
  {
    Fixture g;
    fixture_init(&g);
    Clock_Init(&g.engine_state.clock);
    g.engine_config.channel_state[0].shape_mode = SHAPE_PWM;
    fixture_set_param(&g, 0, 0, CH_PARAM_AMP, 20000);
    fixture_set_param(&g, 0, 0, CH_PARAM_FRQ, 0);
    fixture_set_param(&g, 0, 0, CH_PARAM_SHP, shapes[s]);

    for (int i = 0; i < 500; i++)
    {
      fixture_tick(&g, 2000); // 1s in total: exactly one cycle
      int16_t v = g.engine_state.channels_output_level[0];
      CHECK(v == 20000 || v == -20000); // two levels only - it is a square
      if (v > 0)
        high_at[s]++;
    }
  }

  CHECK(high_at[0] < high_at[1] && high_at[1] < high_at[2]);
  CHECK(high_at[1] > 200 && high_at[1] < 300); // ~50% at the centre
}

// Sample one full cycle of channel 0 at 1x the beat, which is one second.
static void sweep_cycle(Fixture* f, float* out, int n)
{
  for (int i = 0; i < n; i++)
  {
    fixture_tick(f, 1000000u / (uint32_t) n);
    out[i] = (float) f->engine_state.channels_output_level[0];
  }
}

static void setup_pwm(Fixture* f, int16_t shp, int16_t mod)
{
  fixture_init(f);
  Clock_Init(&f->engine_state.clock);
  f->engine_config.channel_state[0].shape_mode = SHAPE_PWM;
  fixture_set_param(f, 0, 0, CH_PARAM_AMP, 20000);
  fixture_set_param(f, 0, 0, CH_PARAM_FRQ, 0);
  fixture_set_param(f, 0, 0, CH_PARAM_SHP, shp);
  fixture_set_param(f, 0, 0, CH_PARAM_MOD, mod);
}

// Where in the cycle the output peaks, as a fraction. Which end of the pulse
// the level is at when it opens is the whole point of MOD's sign, and it is
// what the peak position shows.
static float peak_position(const float* v, int n)
{
  int best = 0;
  for (int i = 1; i < n; i++)
  {
    if (v[i] > v[best])
      best = i;
  }
  return (float) best / (float) n;
}

static int intermediate_samples(const float* v, int n)
{
  int count = 0;
  for (int i = 0; i < n; i++)
  {
    if (v[i] > -19000.0f && v[i] < 19000.0f)
      count++;
  }
  return count;
}

// The first and last sample at full swing, which bound the plateau. -1 for
// both when the envelope never gets there.
static void plateau_bounds(const float* v, int n, int* first, int* last)
{
  *first = -1;
  *last  = -1;
  for (int i = 0; i < n; i++)
  {
    if (v[i] > 19000.0f)
    {
      if (*first < 0)
        *first = i;
      *last = i;
    }
  }
}

// MOD 0 is still a gate, which is what keeps a PWM channel usable as a clock
// divider. The ends of the knob are one ramp filling the pulse - a pure decay
// one way, a pure attack the other.
TEST_CASE(pwm_mod_ends_are_a_pure_decay_and_a_pure_attack)
{
  const int N = 400;
  float v[400];

  Fixture f;

  setup_pwm(&f, 0, 0); // hard gate
  sweep_cycle(&f, v, N);
  CHECK(intermediate_samples(v, N) <= 2); // only the two crossings

  // Negative: instant attack, then a decay that finishes with the pulse. The
  // peak is at the very start of the cycle.
  setup_pwm(&f, 0, -INT16_MAX);
  sweep_cycle(&f, v, N);
  CHECK(intermediate_samples(v, N) > N / 8);
  CHECK(peak_position(v, N) < 0.05f);

  // Positive: a swell across the whole pulse and then a hard drop, so the peak
  // is where the gate closes - at this width, halfway through the cycle.
  setup_pwm(&f, 0, INT16_MAX);
  sweep_cycle(&f, v, N);
  CHECK(intermediate_samples(v, N) > N / 8);
  CHECK(peak_position(v, N) > 0.4f && peak_position(v, N) < 0.55f);
}

// The point of the mapping. Off centre the pulse carries an attack, a plateau
// and a decay at once, and MOD's sign says which of the two ramps gets the
// larger share of the budget.
//
// Under the mapping before it the two ramps sat on opposite sides of the pulse
// edge - the rise inside the on-time, the fall inside the off-time - so they
// could never coexist and no setting on the knob was an AD envelope.
TEST_CASE(pwm_mod_off_centre_gives_an_attack_a_plateau_and_a_decay)
{
  const int N    = 400;
  const int gate = N / 2; // SHP 0 is a half-cycle pulse
  float v[400];
  int first, last;

  Fixture f;

  // Half a turn toward the decay: a quarter of the pulse is ramp, split
  // three-to-one in the decay's favour.
  setup_pwm(&f, 0, -INT16_MAX / 2);
  sweep_cycle(&f, v, N);
  plateau_bounds(v, N, &first, &last);

  CHECK(first > 0);             // an attack ran before the plateau
  CHECK(last > first);          // the plateau is a plateau, not a corner
  CHECK(last < gate - 1);       // and a decay ran after it, inside the pulse
  CHECK((gate - last) > first); // the decay is the longer of the two

  // The same turn the other way, and the two ramps swap shares.
  setup_pwm(&f, 0, INT16_MAX / 2);
  sweep_cycle(&f, v, N);
  plateau_bounds(v, N, &first, &last);

  CHECK(first > 0);
  CHECK(last > first);
  CHECK(last < gate - 1);
  CHECK(first > (gate - last)); // the attack is now the longer one
}

// Both ramps live inside the pulse, so the width still means what it says
// however far MOD is turned: past the pulse the output is at the floor.
//
// This is the coupling the change removes. The decay used to be spent on the
// off-time, which ran its length *inverse* to the width - a narrow pulse rang
// on for the rest of the cycle whether or not that was wanted, and a wide one
// had nowhere to decay into at all.
TEST_CASE(the_pwm_envelope_stays_inside_the_pulse)
{
  const int N = 400;
  float v[400];

  Fixture f;
  setup_pwm(&f, -25000, -INT16_MAX); // narrow pulse, all decay
  sweep_cycle(&f, v, N);

  CHECK(peak_position(v, N) < 0.05f);
  CHECK(intermediate_samples(v, N) > 4); // the decay is real, not a corner

  // Width here is ~13% of the cycle, so the whole envelope is over long before
  // halfway and the rest of the cycle is flat.
  for (int i = N / 4; i < N; i++)
  {
    CHECK(v[i] == -20000.0f);
  }
}

// A host is free to hand the engine any beat_freq it likes, and one that is
// not finite used to poison the channel permanently: every comparison against
// a NaN is false, so it survived the wrap, the fmodf and every later tick, and
// the wavetable was then indexed with whatever a NaN casts to. One bad frame
// should cost one cycle, not the channel.
TEST_CASE(a_non_finite_clock_does_not_poison_the_phase)
{
  Fixture f;
  fixture_init(&f);
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 10000);

  f.engine_state.clock.beat_freq        = INFINITY;
  f.engine_state.clock.beat_freq_smooth = INFINITY;

  fixture_tick(&f, 250);
  CHECK(isfinite(f.engine_state.channels_shared_phase[0]));

  // and it runs normally again from the next tick
  f.engine_state.clock.beat_freq        = 1.0f;
  f.engine_state.clock.beat_freq_smooth = 1.0f;

  for (int i = 0; i < 100; i++)
  {
    fixture_tick(&f, 250);
    CHECK(isfinite(f.engine_state.channels_shared_phase[0]));
    CHECK(isfinite(f.engine_state.channels_effective[0].phase));
  }
}

int main(void)
{
  RUN_TEST(a_non_finite_clock_does_not_poison_the_phase);
  RUN_TEST(zero_amplitude_channel_outputs_the_offset);
  RUN_TEST(amplitude_bounds_the_output_around_the_offset);
  RUN_TEST(full_amplitude_alone_covers_the_whole_bipolar_range);
  RUN_TEST(input_add_mode_adds_the_raw_input_value);
  RUN_TEST(input_mult_mode_zero_input_zeroes_the_output);
  RUN_TEST(continuous_quantize_matches_quantize_value);
  RUN_TEST(phase_advances_by_frequency_times_dt_without_pll_lock);
  RUN_TEST(pattern_length_is_latched_until_the_cycle_wraps);
  RUN_TEST(pattern_length_updates_once_the_cycle_wraps);
  RUN_TEST(pattern_length_applies_immediately_while_the_encoder_is_turning);
  RUN_TEST(the_output_clamp_bounds_the_swing_without_rescaling_it);
  RUN_TEST(pwm_is_a_square_whose_duty_follows_the_shape_parameter);
  RUN_TEST(pwm_mod_ends_are_a_pure_decay_and_a_pure_attack);
  RUN_TEST(pwm_mod_off_centre_gives_an_attack_a_plateau_and_a_decay);
  RUN_TEST(the_pwm_envelope_stays_inside_the_pulse);
  return TESTKIT_SUMMARY();
}
