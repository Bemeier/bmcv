#include "presets.h"
#include "fram.h"
#include "helpers.h"

_Static_assert(sizeof(EngineConfigRecord) <= FRAM_CONFIG_SLOT_SIZE, "ConfigState too large for FRAM slot");

int8_t preset_store(EngineConfig* cfg, int8_t dst)
{
    if (dst >= FRAM_CONFIG_SLOTS)
        return false;

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
    return true;
}

int8_t preset_load(EngineConfig* cfg, int8_t src)
{
    if (src >= FRAM_CONFIG_SLOTS)
        return false;

    EngineConfigRecord rec;
    uint16_t addr = FRAM_CONFIG_BASE_ADDR + src * FRAM_CONFIG_SLOT_SIZE;

    fram_Read(addr, (uint8_t*) &rec, sizeof(rec));

    /* Validate header */
    if (rec.hdr.magic != FRAM_MAGIC)
        return false;

    if (rec.hdr.version != CONFIG_STATE_VERSION)
        return false; // Or trigger migration

    if (rec.hdr.length != sizeof(EngineConfig))
        return false;

    if (rec.hdr.crc != crc32(&rec.data, sizeof(EngineConfig)))
        return false;

    *cfg = rec.data;
    return true;
}
