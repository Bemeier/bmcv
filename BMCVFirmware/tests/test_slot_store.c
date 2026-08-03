// The in-memory preset slots every host uses in place of FRAM.
//
// Small enough that it looks not worth testing, but it is the thing standing
// between a saved patch and a lost one, and it is what an instance boots from:
// bmcv_instance_init() reads CONFIG_AUTOSAVE_SLOT, so "an unoccupied slot
// reports nothing" is the first-boot path, not an edge case.

#include "config.h"
#include "instance.h"
#include "config_validate.h"
#include "slot_store.h"
#include "testkit.h"
#include <string.h>

static EngineConfig marked(uint8_t byte)
{
  EngineConfig cfg;
  config_defaults(&cfg);
  cfg.channel_state[0].params[0][CH_PARAM_AMP] = (int8_t) byte;
  return cfg;
}

// The store is a plain member of whatever struct the host keeps, and a VCV
// Rack Module is allocated with `new`. If init trusted the caller to have
// zeroed it, `occupied` would be heap noise, every slot would report itself
// full, and the module would boot on garbage rather than on the first-boot
// defaults - which looks like a working module with nonsense in it.
TEST_CASE(a_store_that_was_never_zeroed_still_comes_up_empty)
{
  SlotStore st;
  memset(&st, 0xA5, sizeof(st));

  PresetIo io;
  slot_store_init(&st, &io);

  for (int8_t s = 0; s < FRAM_CONFIG_SLOTS; s++)
  {
    EngineConfig cfg;
    CHECK(io.load(io.user, &cfg, s) == 0);
  }

  // Compared against an instance built on a store that *was* zeroed, rather
  // than against config_defaults(): first boot is more than the defaults -
  // bmcv_instance_init() runs channel_reset() over them too - and restating
  // that here would only be a copy of the thing under test.
  BmcvInstance dirty;
  bmcv_instance_init(&dirty, &io, 0);

  SlotStore clean = {0};
  PresetIo clean_io;
  slot_store_init(&clean, &clean_io);
  BmcvInstance fresh;
  bmcv_instance_init(&fresh, &clean_io, 0);

  CHECK(memcmp(&dirty.engine_config, &fresh.engine_config, sizeof(EngineConfig)) == 0);
}

TEST_CASE(a_stored_slot_comes_back_unchanged)
{
  SlotStore st = {0};
  PresetIo io;
  slot_store_init(&st, &io);

  EngineConfig in = marked(42);
  CHECK(io.store(io.user, &in, 3) == 1);

  EngineConfig out;
  memset(&out, 0, sizeof(out));
  CHECK(io.load(io.user, &out, 3) == 1);
  CHECK(memcmp(&in, &out, sizeof(EngineConfig)) == 0);
}

TEST_CASE(an_unoccupied_slot_reports_nothing_stored)
{
  SlotStore st = {0};
  PresetIo io;
  slot_store_init(&st, &io);

  EngineConfig out = marked(7);
  CHECK(io.load(io.user, &out, 0) == 0);
  // A failed load must leave the caller's config alone rather than half-fill it.
  CHECK(out.channel_state[0].params[0][CH_PARAM_AMP] == 7);
}

TEST_CASE(an_out_of_range_slot_fails_rather_than_clamping)
{
  SlotStore st = {0};
  PresetIo io;
  slot_store_init(&st, &io);

  EngineConfig cfg = marked(1);
  CHECK(io.store(io.user, &cfg, -1) == 0);
  CHECK(io.store(io.user, &cfg, FRAM_CONFIG_SLOTS) == 0);
  CHECK(io.load(io.user, &cfg, -1) == 0);
  CHECK(io.load(io.user, &cfg, FRAM_CONFIG_SLOTS) == 0);

  // Nothing was written to slot 0 or to the last slot on the way past.
  for (int8_t s = 0; s < FRAM_CONFIG_SLOTS; s++)
  {
    CHECK(st.occupied[s] == 0);
  }
}

TEST_CASE(slots_are_independent)
{
  SlotStore st = {0};
  PresetIo io;
  slot_store_init(&st, &io);

  for (int8_t s = 0; s < FRAM_CONFIG_SLOTS; s++)
  {
    EngineConfig cfg = marked((uint8_t) (s + 10));
    CHECK(io.store(io.user, &cfg, s) == 1);
  }
  for (int8_t s = 0; s < FRAM_CONFIG_SLOTS; s++)
  {
    EngineConfig cfg;
    CHECK(io.load(io.user, &cfg, s) == 1);
    CHECK(cfg.channel_state[0].params[0][CH_PARAM_AMP] == (int8_t) (s + 10));
  }
}

TEST_CASE(clearing_puts_it_back_to_blank_fram)
{
  SlotStore st = {0};
  PresetIo io;
  slot_store_init(&st, &io);

  EngineConfig cfg = marked(99);
  CHECK(io.store(io.user, &cfg, CONFIG_AUTOSAVE_SLOT) == 1);
  slot_store_clear(&st);
  CHECK(io.load(io.user, &cfg, CONFIG_AUTOSAVE_SLOT) == 0);
}

// The reason this store exists: a host serialises it, and an instance built on
// it boots from CONFIG_AUTOSAVE_SLOT. Copying the bytes has to be enough to
// move a module's state from one process to another.
TEST_CASE(a_copied_store_boots_a_second_instance_the_same_way)
{
  SlotStore sa = {0};
  PresetIo ioa;
  slot_store_init(&sa, &ioa);

  BmcvInstance a;
  bmcv_instance_init(&a, &ioa, 0);
  a.engine_config.channel_state[2].params[0][CH_PARAM_AMP] = 77;
  CHECK(ioa.store(ioa.user, &a.engine_config, CONFIG_AUTOSAVE_SLOT) == 1);

  // Init first, then load the blob over it - which is the order a host has to
  // use, because slot_store_init() empties the store it is given.
  SlotStore sb;
  PresetIo iob;
  slot_store_init(&sb, &iob);
  memcpy(&sb, &sa, sizeof(SlotStore)); // what a host's blob round trip is

  BmcvInstance b;
  bmcv_instance_init(&b, &iob, 0);
  CHECK(b.engine_config.channel_state[2].params[0][CH_PARAM_AMP] == 77);

  // And the two stores are separate afterwards.
  b.engine_config.channel_state[2].params[0][CH_PARAM_AMP] = 5;
  CHECK(iob.store(iob.user, &b.engine_config, 0) == 1);
  CHECK(sa.occupied[0] == 0);
}

int main(void)
{
  RUN_TEST(a_store_that_was_never_zeroed_still_comes_up_empty);
  RUN_TEST(a_stored_slot_comes_back_unchanged);
  RUN_TEST(an_unoccupied_slot_reports_nothing_stored);
  RUN_TEST(an_out_of_range_slot_fails_rather_than_clamping);
  RUN_TEST(slots_are_independent);
  RUN_TEST(clearing_puts_it_back_to_blank_fram);
  RUN_TEST(a_copied_store_boots_a_second_instance_the_same_way);
  return TESTKIT_SUMMARY();
}
