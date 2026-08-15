// The boundary between untrusted persistent storage and the config that indexes
// arrays.
//
// preset_load is the only thing standing between a bit-flipped, half-written or
// foreign FRAM and an EngineConfig the rest of the firmware trusts - and every
// consumer downstream indexes tables with fields out of that struct. It checked
// four things and none of them were tested, on a driver-layer file that could
// not be host-compiled until this fake replaced the SPI part.
//
// config_validate is the second line and has its own suite; the point here is
// the first, which decides whether a record is looked at at all.

#include "config.h"
#include "config_validate.h"
#include "fram.h"
#include "helpers.h"
#include "presets.h"
#include "testkit.h"
#include <string.h>

// Where preset_load will look for slot `n`, in the fake part's address space.
static uint8_t* slot_bytes(int8_t n) { return fram_fake_bytes() + FRAM_CONFIG_BASE_ADDR + (uint16_t) n * FRAM_CONFIG_SLOT_SIZE; }

static EngineConfig a_config(int16_t marker)
{
  EngineConfig cfg;
  config_defaults(&cfg);
  cfg.channel_state[0].params[0][CH_PARAM_AMP] = marker;
  return cfg;
}

// Blank part, then one good record in slot 2 so the negative cases have
// something real to corrupt.
static EngineConfig store_a_good_one(int8_t slot, int16_t marker)
{
  fram_fake_reset(0xFF);
  EngineConfig cfg = a_config(marker);
  CHECK(preset_store(&cfg, slot) == 1);
  return cfg;
}

TEST_CASE(a_stored_record_comes_back)
{
  EngineConfig in = store_a_good_one(2, 4321);

  EngineConfig out;
  memset(&out, 0, sizeof out);
  CHECK(preset_load(&out, 2) == 1);
  CHECK(out.channel_state[0].params[0][CH_PARAM_AMP] == 4321);
  CHECK(memcmp(&in, &out, sizeof in) == 0);
}

// A part that has never been written is 0xFF everywhere, which is not a record
// and must not be read as one. The magic is what says "something wrote this".
TEST_CASE(a_blank_part_holds_no_presets)
{
  fram_fake_reset(0xFF);

  EngineConfig out;
  for (int8_t slot = 0; slot < FRAM_CONFIG_SLOTS; slot++)
    CHECK(preset_load(&out, slot) == 0);

  // And the other kind of blank: zeroed rather than erased.
  fram_fake_reset(0x00);
  for (int8_t slot = 0; slot < FRAM_CONFIG_SLOTS; slot++)
    CHECK(preset_load(&out, slot) == 0);
}

TEST_CASE(a_record_with_the_wrong_magic_is_refused)
{
  store_a_good_one(2, 100);

  slot_bytes(2)[0] ^= 0xFF; // one byte of the magic

  EngineConfig out;
  CHECK(preset_load(&out, 2) == 0);
}

// The length is the CRC's extent, so a corrupt one aims crc32 at a span that
// was never read out of the part. Bounded both ways: zero has nothing to check,
// and anything past the payload would run off the end of the record on the
// stack.
TEST_CASE(a_record_whose_length_is_impossible_is_refused)
{
  EngineConfig out;

  store_a_good_one(2, 100);
  ((FramRecordHeader*) slot_bytes(2))->length = 0;
  CHECK(preset_load(&out, 2) == 0);

  store_a_good_one(2, 100);
  ((FramRecordHeader*) slot_bytes(2))->length = sizeof(EngineConfig) + 1;
  CHECK(preset_load(&out, 2) == 0);

  store_a_good_one(2, 100);
  ((FramRecordHeader*) slot_bytes(2))->length = 0xFFFF;
  CHECK(preset_load(&out, 2) == 0);
}

// The case the magic and the length both pass: a header that looks right over a
// payload that has rotted. This is what the CRC is for, and the only check that
// catches a single flipped bit in the middle of the config.
TEST_CASE(a_record_whose_payload_has_rotted_is_refused)
{
  store_a_good_one(2, 100);

  // One bit, well inside the payload rather than in the header.
  slot_bytes(2)[sizeof(FramRecordHeader) + 40] ^= 0x01;

  EngineConfig out;
  CHECK(preset_load(&out, 2) == 0);
}

TEST_CASE(a_record_from_a_future_version_is_refused_rather_than_guessed_at)
{
  store_a_good_one(2, 100);
  ((FramRecordHeader*) slot_bytes(2))->version = CONFIG_STATE_VERSION + 1;

  // The CRC still covers the payload, so this is a record that is intact and
  // simply not readable by this build - which is a different thing from
  // corrupt, and must fail the same way rather than being migrated by guess.
  EngineConfig out;
  CHECK(preset_load(&out, 2) == 0);
}

// A refused record must not leave a half-filled config behind. The caller reads
// the return value, but a partially-overwritten destination is the sort of
// thing that only bites once something later stops checking.
TEST_CASE(a_refused_load_leaves_the_destination_alone)
{
  store_a_good_one(2, 100);
  slot_bytes(2)[0] ^= 0xFF;

  EngineConfig out;
  memset(&out, 0x5A, sizeof out);
  EngineConfig before = out;

  CHECK(preset_load(&out, 2) == 0);
  CHECK(memcmp(&before, &out, sizeof out) == 0);
}

// Negative and past-the-end slots, both ends. A negative one is the dangerous
// direction: the address is computed as an unsigned 16-bit product, so -1 would
// wrap to somewhere near the top of the part and write there.
TEST_CASE(an_out_of_range_slot_is_refused_at_both_ends)
{
  fram_fake_reset(0xFF);
  EngineConfig cfg = a_config(7);
  EngineConfig out;

  CHECK(preset_store(&cfg, -1) == 0);
  CHECK(preset_store(&cfg, FRAM_CONFIG_SLOTS) == 0);
  CHECK(preset_load(&out, -1) == 0);
  CHECK(preset_load(&out, FRAM_CONFIG_SLOTS) == 0);

  // Nothing was written anywhere while refusing.
  for (uint32_t i = 0; i < FRAM_FAKE_SIZE; i++)
    CHECK(fram_fake_bytes()[i] == 0xFF);
}

TEST_CASE(slots_do_not_overlap)
{
  fram_fake_reset(0xFF);

  for (int8_t slot = 0; slot < FRAM_CONFIG_SLOTS; slot++)
  {
    EngineConfig cfg = a_config((int16_t) (1000 + slot));
    CHECK(preset_store(&cfg, slot) == 1);
  }

  for (int8_t slot = 0; slot < FRAM_CONFIG_SLOTS; slot++)
  {
    EngineConfig out;
    CHECK(preset_load(&out, slot) == 1);
    CHECK(out.channel_state[0].params[0][CH_PARAM_AMP] == 1000 + slot);
  }
}

// Clearing invalidates every header without touching the payloads - see the
// note in presets.h about FRAM wear.
TEST_CASE(clearing_makes_every_slot_read_as_empty)
{
  fram_fake_reset(0xFF);
  for (int8_t slot = 0; slot < FRAM_CONFIG_SLOTS; slot++)
  {
    EngineConfig cfg = a_config(55);
    preset_store(&cfg, slot);
  }

  CHECK(preset_clear() == 1);

  EngineConfig out;
  for (int8_t slot = 0; slot < FRAM_CONFIG_SLOTS; slot++)
    CHECK(preset_load(&out, slot) == 0);
}

// What a record that survives every check is worth: preset_load hands it to
// config_migrate, which validates. So a header-intact record carrying nonsense
// still cannot reach the engine with an out-of-range field - which is what
// makes the ui_render tables safe to index without a second check.
TEST_CASE(a_valid_record_carrying_nonsense_is_still_brought_into_range)
{
  fram_fake_reset(0xFF);

  EngineConfig cfg                = a_config(0);
  cfg.channel_state[3].shape_mode = 99;
  cfg.channel_state[3].src_input  = 77;
  cfg.selected_param              = 200;
  cfg.scene_a                     = 99;
  CHECK(preset_store(&cfg, 1) == 1); // stored as-is; the CRC covers it

  EngineConfig out;
  CHECK(preset_load(&out, 1) == 1);
  CHECK(out.channel_state[3].shape_mode < SHAPE_MODE_COUNT);
  CHECK(out.channel_state[3].src_input == -1);
  CHECK(out.selected_param < CH_PARAM_COUNT);
  CHECK(out.scene_a < N_SCENES);
}

int main(void)
{
  RUN_TEST(a_stored_record_comes_back);
  RUN_TEST(a_blank_part_holds_no_presets);
  RUN_TEST(a_record_with_the_wrong_magic_is_refused);
  RUN_TEST(a_record_whose_length_is_impossible_is_refused);
  RUN_TEST(a_record_whose_payload_has_rotted_is_refused);
  RUN_TEST(a_record_from_a_future_version_is_refused_rather_than_guessed_at);
  RUN_TEST(a_refused_load_leaves_the_destination_alone);
  RUN_TEST(an_out_of_range_slot_is_refused_at_both_ends);
  RUN_TEST(slots_do_not_overlap);
  RUN_TEST(clearing_makes_every_slot_read_as_empty);
  RUN_TEST(a_valid_record_carrying_nonsense_is_still_brought_into_range);
  return TESTKIT_SUMMARY();
}
