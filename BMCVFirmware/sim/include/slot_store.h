#ifndef BMCV_SLOT_STORE_H_
#define BMCV_SLOT_STORE_H_

#include "config.h"
#include "ux_state.h"
#include <stdint.h>

// The preset slots, in memory.
//
// On the module these live in FRAM behind presets.c; on a host they are just
// bytes the host keeps somewhere - a browser's localStorage, a Rack patch
// file. Every host needs the same thing, so it is here beside sim_rt.h rather
// than reimplemented per frontend.
//
// Records are plain EngineConfigs, not the FRAM record format, so a blob is
// not interchangeable with a chip dump. It does not need to be: the config
// carries its own version and config_validate() rejects what it cannot read.

typedef struct
{
  EngineConfig slots[FRAM_CONFIG_SLOTS];
  uint8_t occupied[FRAM_CONFIG_SLOTS];
} SlotStore;

// Empty the store and point a PresetIo at it. The store must outlive the
// instance using it, which for every host so far means both are fields of the
// same struct - and that struct is not necessarily zeroed, which is why this
// clears rather than assuming.
void slot_store_init(SlotStore* st, PresetIo* io);

// Forget every slot. An instance initialised afterwards comes up on the
// first-boot defaults, as a module with a blank FRAM does.
void slot_store_clear(SlotStore* st);

#endif /* BMCV_SLOT_STORE_H_ */
