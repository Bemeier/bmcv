#include "presets.h"
#include "config_migrate.h"
#include "fram.h"
#include "helpers.h"

_Static_assert(sizeof(EngineConfigRecord) <= FRAM_CONFIG_SLOT_SIZE, "ConfigState too large for FRAM slot");

int8_t preset_store(const EngineConfig* cfg, int8_t dst)
{
  // Both ends: a negative slot would produce a wrapped uint16_t address and
  // write somewhere arbitrary in FRAM.
  if (dst < 0 || dst >= FRAM_CONFIG_SLOTS)
    return 0;

  EngineConfigRecord rec = {.hdr =
                                {
                                    .magic   = FRAM_MAGIC,
                                    .version = CONFIG_STATE_VERSION,
                                    .length  = sizeof(EngineConfig),
                                    .crc     = crc32(cfg, sizeof(EngineConfig)),
                                },
                            .data = *cfg};
  uint16_t addr          = FRAM_CONFIG_BASE_ADDR + dst * FRAM_CONFIG_SLOT_SIZE;
  fram_Write(addr, (uint8_t*) &rec, sizeof(rec));
  return 1;
}

int8_t preset_load(EngineConfig* cfg, int8_t src)
{
  if (src < 0 || src >= FRAM_CONFIG_SLOTS)
    return 0;

  EngineConfigRecord rec;
  uint16_t addr = FRAM_CONFIG_BASE_ADDR + src * FRAM_CONFIG_SLOT_SIZE;

  fram_Read(addr, (uint8_t*) &rec, sizeof(rec));

  /* Validate header */
  if (rec.hdr.magic != FRAM_MAGIC)
    return 0;

  // The payload is whatever EngineConfig was when it was written, so the CRC
  // covers hdr.length bytes rather than the current struct's size. Bounded
  // both ways: a corrupt length must not send crc32 past what was read out of
  // FRAM, and a zero-length record has nothing to check.
  if (rec.hdr.length == 0 || rec.hdr.length > sizeof(rec.data))
    return 0;

  if (rec.hdr.crc != crc32(&rec.data, rec.hdr.length))
    return 0;

  // Intact is not the same as readable. config_migrate decides whether this
  // build knows the version, converts it if so, and validates what it produced.
  return config_migrate(rec.hdr.version, rec.hdr.length, &rec.data, cfg);
}
