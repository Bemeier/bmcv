#ifndef INC_PRESETS_H_
#define INC_PRESETS_H_

#include "state.h"
#include <stdint.h>

#define FRAM_MAGIC 0x424D4356
#define FRAM_CONFIG_SLOT_SIZE 896
#define FRAM_CONFIG_SLOTS 8
#define FRAM_CONFIG_BASE_ADDR 0x0000
#define CONFIG_STATE_VERSION 2

typedef struct __attribute__((packed))
{
  FramRecordHeader hdr;
  EngineConfig data;
} EngineConfigRecord;

int8_t preset_store(EngineConfig* cfg, int8_t dst);

int8_t preset_load(EngineConfig* cfg, int8_t src);

#endif /* INC_PRESETS_H_ */
