#include "slot_store.h"
#include <string.h>

// Both return the int8_t "did it work" the core's PresetIo expects, and both
// treat an out-of-range slot as a failure rather than clamping - a caller that
// computed a bad slot wants to hear about it, and scene.c raises an error flag
// when a load it asked for reports nothing.

static int8_t slot_put(void* user, const EngineConfig* cfg, int8_t slot)
{
  SlotStore* st = (SlotStore*) user;
  if (slot < 0 || slot >= FRAM_CONFIG_SLOTS)
    return 0;
  st->slots[slot]    = *cfg;
  st->occupied[slot] = 1;
  return 1;
}

static int8_t slot_get(void* user, EngineConfig* cfg, int8_t slot)
{
  SlotStore* st = (SlotStore*) user;
  if (slot < 0 || slot >= FRAM_CONFIG_SLOTS || !st->occupied[slot])
    return 0;
  *cfg = st->slots[slot];
  return 1;
}

static int8_t slot_wipe(void* user)
{
  slot_store_clear((SlotStore*) user);
  return 1;
}

void slot_store_init(SlotStore* st, PresetIo* io)
{
  // Cleared, not just wired. `occupied` is what decides whether a slot holds a
  // config, so a store that has not been zeroed reports every slot as full and
  // the next instance boots on whatever was in that memory - which is not a
  // crash but a module with a -23131 offset and both scenes set to the same
  // one. The simulator calloc'd its way past this; a VCV Rack Module is
  // allocated with `new` and its plain members hold heap noise.
  slot_store_clear(st);

  io->store = slot_put;
  io->load  = slot_get;
  io->clear = slot_wipe;
  io->user  = st;
}

void slot_store_clear(SlotStore* st) { memset(st, 0, sizeof(*st)); }
