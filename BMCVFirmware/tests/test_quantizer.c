#include "helpers.h"
#include "testkit.h"

static const float SEMITONE_STEP = (float) SEMITONE_DAC_FP / (float) FP_SCALE; // ~273.07 DAC units/semitone

// quantize_value() subtracts DAC_OFFSET_CORRECTION so the analog output lands
// on pitch. Add it back to recover the ideal note position these tests reason
// about.
static float note_position(int16_t quantized) { return (float) (quantized + DAC_OFFSET_CORRECTION); }

static float distance_to_nearest_semitone(float pos)
{
  float nearest = roundf(pos / SEMITONE_STEP);
  return fabsf(pos - nearest * SEMITONE_STEP);
}

static int32_t scale_degree_of(int16_t quantized)
{
  int32_t semitone = (int32_t) lrintf(note_position(quantized) / SEMITONE_STEP);
  return ((semitone % 12) + 12) % 12;
}

TEST_CASE(output_lands_on_an_exact_semitone)
{
  uint16_t masks[] = {0b111111111111, 1u << 0, 1u << 7, (1u << 0) | (1u << 4) | (1u << 7)};
  for (size_t m = 0; m < sizeof(masks) / sizeof(masks[0]); m++)
  {
    for (int16_t x = -8000; x <= 8000; x += 137)
    {
      CHECK(distance_to_nearest_semitone(note_position(quantize_value(x, masks[m]))) < 1.0f);
    }
  }
}

TEST_CASE(output_lands_on_a_note_enabled_in_the_scale)
{
  for (int16_t x = -8000; x <= 8000; x += 251)
  {
    CHECK(scale_degree_of(quantize_value(x, 1u << 7)) == 7);
  }
}

TEST_CASE(chromatic_scale_never_moves_pitch_by_more_than_half_a_semitone)
{
  uint16_t full_mask = 0b111111111111;
  for (int16_t x = -16000; x <= 16000; x += 37)
  {
    float moved = fabsf(note_position(quantize_value(x, full_mask)) - (float) x);
    CHECK(moved <= SEMITONE_STEP / 2.0f + 1.0f);
  }
}

TEST_CASE(picks_the_truly_nearest_enabled_note)
{
  // 238 sits 0.87 semitones above the root, so it belongs on note 1, not 0.
  CHECK(scale_degree_of(quantize_value(238, 0b111111111111)) == 1);

  // With only root and tritone enabled, 3.2 semitones is nearer the tritone.
  uint16_t root_and_tritone = (1u << 0) | (1u << 6);
  int16_t at_3_2            = (int16_t) (3.2f * SEMITONE_STEP);
  CHECK(scale_degree_of(quantize_value(at_3_2, root_and_tritone)) == 6);
}

TEST_CASE(quantization_is_idempotent)
{
  uint16_t masks[] = {0b111111111111, 1u << 0, 1u << 7, (1u << 0) | (1u << 4) | (1u << 7), 0b101011010101};
  for (size_t m = 0; m < sizeof(masks) / sizeof(masks[0]); m++)
  {
    for (int16_t x = -16000; x <= 16000; x += 371)
    {
      int16_t once  = quantize_value(x, masks[m]);
      int16_t twice = quantize_value(once, masks[m]);
      CHECK(once == twice);
    }
  }
}

TEST_CASE(negative_voltages_behave_symmetrically_with_positive)
{
  uint16_t full_mask = 0b111111111111;
  for (int16_t x = 137; x <= 8000; x += 271)
  {
    float up   = note_position(quantize_value(x, full_mask)) - (float) x;
    float down = note_position(quantize_value((int16_t) -x, full_mask)) - (float) (-x);
    CHECK_NEAR(up, -down, 1.5);
  }
}

TEST_CASE(empty_scale_passes_the_signal_through_unquantized)
{
  for (int16_t x = -8000; x <= 8000; x += 313)
  {
    CHECK(quantize_value(x, 0) == x);
  }
}

int main(void)
{
  RUN_TEST(output_lands_on_an_exact_semitone);
  RUN_TEST(output_lands_on_a_note_enabled_in_the_scale);
  RUN_TEST(chromatic_scale_never_moves_pitch_by_more_than_half_a_semitone);
  RUN_TEST(picks_the_truly_nearest_enabled_note);
  RUN_TEST(quantization_is_idempotent);
  RUN_TEST(negative_voltages_behave_symmetrically_with_positive);
  RUN_TEST(empty_scale_passes_the_signal_through_unquantized);
  return TESTKIT_SUMMARY();
}
