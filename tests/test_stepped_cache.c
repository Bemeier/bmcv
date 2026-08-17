// stepped_shape_cached() must return bit-for-bit what stepped_shape_with()
// returns, for every sequence of calls, for ever.
//
// That is the whole contract of the cache, and it is not a property that stays
// true by itself. StStepCache holds a key - shape, mod, length_idx, step - and
// that key has to name every input st_morph() and st_step_pair() read. A lever
// added to StMorph and driven by something the key does not mention would leave
// a channel drawing its old pattern: no crash, no drift in level, just the
// wrong shape, sounding entirely plausible. This file is what turns that into a
// red build.
//
// Bit-for-bit rather than within a tolerance, deliberately. The cached route
// takes the same operations in the same order - st_step_next() returns exactly
// st_step_value(next), which is what lets one tick's `to` become the next
// tick's `from` - so any difference at all means the two routes have come
// apart, and a tolerance would let the first one through.

#include "stepped.h"
#include "stepped_table.h" // ST_LENGTH_COUNT
#include "testkit.h"
#include <math.h>
#include <string.h>

// A channel's worth of state, driven the way channel_compute drives it, so the
// cache sees runs of hits, single-step advances, wraps and jumps in the
// proportions the engine actually produces.
typedef struct
{
  StStepCache cache;
  StSlotMemo memo;
  StDrive drive;
  StNorm norm;
  float shape, mod;
  int length_idx;
} Rig;

static void rig_init(Rig* r, float shape, float mod, int length_idx)
{
  memset(&r->cache, 0, sizeof r->cache); // a fresh EngineState is zeroed
  memset(&r->memo, 0, sizeof r->memo);
  r->shape      = shape;
  r->mod        = mod;
  r->length_idx = length_idx;
  r->drive      = st_drive(shape, mod);
  r->norm       = st_norm_exact(shape, mod, length_idx, &r->drive);
}

// Both routes, one sample. memcmp rather than ==, so that two NaNs still
// compare equal and two zeroes of different sign do not.
static int agrees(Rig* r, float phase)
{
  // The reference route takes no memo at all, so this compares the memoised
  // path against one that recomputes everything - which is the property the
  // memo has to have, not merely that two memoised runs agree.
  float want = stepped_shape_with(phase, r->shape, r->mod, r->length_idx, &r->drive, &r->norm);
  st_slot_memo_begin(&r->memo, r->shape, r->mod, st_length_for_index(r->length_idx));
  float got = stepped_shape_cached(&r->cache, &r->memo, phase, r->shape, r->mod, r->length_idx, &r->drive, &r->norm);
  return memcmp(&want, &got, sizeof want) == 0;
}

// A playhead advancing smoothly, which is what the carry exists for: long runs
// inside one step, a single-step advance, then a wrap. Every length, and knob
// settings spread across both axes.
TEST_CASE(a_full_sweep_is_identical_at_every_length)
{
  static const float shapes[] = {-1.0f, -0.61f, -0.25f, 0.0f, 0.17f, 0.5f, 0.83f, 1.0f};
  static const float mods[]   = {-1.0f, -0.4f, 0.0f, 0.33f, 0.7f, 1.0f};

  int mismatches = 0;
  for (int li = 0; li < ST_LENGTH_COUNT; li++)
  {
    for (size_t si = 0; si < sizeof shapes / sizeof *shapes; si++)
    {
      for (size_t mi = 0; mi < sizeof mods / sizeof *mods; mi++)
      {
        Rig r;
        rig_init(&r, shapes[si], mods[mi], li);

        // Deliberately not a whole number of samples per step, so boundaries
        // land at every offset within a sample rather than always on one.
        const int samples = 733;
        for (int i = 0; i < samples; i++)
        {
          if (!agrees(&r, (float) i / (float) samples))
          {
            mismatches++;
          }
        }
      }
    }
  }
  CHECK(mismatches == 0);
}

// Several cycles in a row, so the wrap from the last step back to slot 0 is
// exercised as an advance and not only as a first call. The carry has to hold
// across it: the last step's `to` is slot 0's value.
TEST_CASE(wrapping_to_slot_zero_keeps_the_routes_identical)
{
  Rig r;
  rig_init(&r, 0.31f, -0.2f, 8); // length 32

  int mismatches      = 0;
  const int per_cycle = 97;
  for (int cycle = 0; cycle < 6; cycle++)
  {
    for (int i = 0; i < per_cycle; i++)
    {
      if (!agrees(&r, (float) i / (float) per_cycle))
      {
        mismatches++;
      }
    }
  }
  CHECK(mismatches == 0);
}

// A rate high enough to skip whole steps, which is the case the carry cannot
// take and has to fall back from. This is the worst case the cache does not
// improve, and it still has to be correct.
TEST_CASE(a_rate_that_skips_steps_falls_back_and_still_matches)
{
  Rig r;
  rig_init(&r, -0.44f, 0.62f, ST_LENGTH_COUNT - 1); // length 64

  int mismatches = 0;
  float phase    = 0.0f;
  for (int i = 0; i < 400; i++)
  {
    phase += 0.0503f; // just over three steps per sample at this length
    if (phase >= 1.0f)
    {
      phase -= 1.0f;
    }
    if (!agrees(&r, phase))
    {
      mismatches++;
    }
  }
  CHECK(mismatches == 0);
}

// The playhead running backwards, which the PLL does when it pulls a channel
// into line. The cache must not mistake it for an advance.
TEST_CASE(a_phase_correction_that_runs_backwards_still_matches)
{
  Rig r;
  rig_init(&r, 0.0f, 0.0f, 6); // length 16

  int mismatches = 0;
  float phase    = 0.9f;
  for (int i = 0; i < 300; i++)
  {
    phase -= 0.011f;
    if (phase < 0.0f)
    {
      phase += 1.0f;
    }
    if (!agrees(&r, phase))
    {
      mismatches++;
    }
  }
  CHECK(mismatches == 0);
}

// Knobs moving under a standing playhead. Phase is held still on purpose: if a
// knob move failed to invalidate the pair, the cached route would keep handing
// back the old one and nothing about the playhead would expose it.
TEST_CASE(shp_and_mod_moving_under_a_still_playhead_invalidate_the_pair)
{
  Rig r;
  rig_init(&r, 0.0f, 0.0f, 6);

  const float phase = 0.4212f;
  int mismatches    = 0;

  for (int i = 0; i < 240; i++)
  {
    r.shape = sinf((float) i * 0.041f);
    r.mod   = cosf((float) i * 0.017f);
    r.drive = st_drive(r.shape, r.mod);
    r.norm  = st_norm_exact(r.shape, r.mod, r.length_idx, &r.drive);

    if (!agrees(&r, phase))
    {
      mismatches++;
    }
  }
  CHECK(mismatches == 0);
}

// A length change swaps the whole pattern at once, and the step index alone
// cannot tell: step 3 of a 16-step pattern and step 3 of a 64-step one are
// different values.
TEST_CASE(a_pattern_length_change_invalidates_the_pair)
{
  Rig r;
  rig_init(&r, 0.22f, -0.35f, 0);

  int mismatches = 0;
  for (int i = 0; i < 200; i++)
  {
    r.length_idx = i % ST_LENGTH_COUNT;
    r.norm       = st_norm_exact(r.shape, r.mod, r.length_idx, &r.drive);

    if (!agrees(&r, 0.61f + 0.001f * (float) i))
    {
      mismatches++;
    }
  }
  CHECK(mismatches == 0);
}

// A zeroed cache means nothing remembered, which is what a fresh EngineState
// hands over. Step 0 is a real step, so the validity flag has to carry that -
// a zeroed step index alone would read as "already holding step 0".
TEST_CASE(a_zeroed_cache_computes_rather_than_trusting_its_zeroed_step)
{
  Rig r;
  rig_init(&r, 0.5f, 0.5f, 4);

  CHECK(agrees(&r, 0.001f)); // a phase inside step 0
}

// The cache is per channel and holds no statics, so two of them interleaved -
// which is exactly what eight channels in one tick are - must not see each
// other's pattern.
TEST_CASE(two_interleaved_channels_do_not_share_a_pattern)
{
  Rig a, b;
  rig_init(&a, -0.7f, 0.25f, 5);
  rig_init(&b, 0.45f, -0.8f, 9);

  int mismatches = 0;
  for (int i = 0; i < 512; i++)
  {
    float pa = (float) i / 512.0f;
    float pb = (float) ((i * 7) % 512) / 512.0f;

    if (!agrees(&a, pa))
    {
      mismatches++;
    }
    if (!agrees(&b, pb))
    {
      mismatches++;
    }
  }
  CHECK(mismatches == 0);
}

int main(void)
{
  RUN_TEST(a_full_sweep_is_identical_at_every_length);
  RUN_TEST(wrapping_to_slot_zero_keeps_the_routes_identical);
  RUN_TEST(a_rate_that_skips_steps_falls_back_and_still_matches);
  RUN_TEST(a_phase_correction_that_runs_backwards_still_matches);
  RUN_TEST(shp_and_mod_moving_under_a_still_playhead_invalidate_the_pair);
  RUN_TEST(a_pattern_length_change_invalidates_the_pair);
  RUN_TEST(a_zeroed_cache_computes_rather_than_trusting_its_zeroed_step);
  RUN_TEST(two_interleaved_channels_do_not_share_a_pattern);
  return TESTKIT_SUMMARY();
}
