#ifndef ERROR_H
#define ERROR_H

#include "engine_state.h"
#include <stdint.h>

// Error codes are a bitmask, one bit per scene button, rendered as an overlay
// by ui_render and cleared by the next interaction. Held in EngineState rather
// than a file static so each module instance has its own.
//
// The bit *is* the scene button that blinks, so the numbering is a user-facing
// fact and not free to renumber. Bit 6 used to mean "no stored config on
// boot", which is a module's normal first-run state and no longer reports.
typedef enum
{
  ERR_PRESET_LOAD  = 5, // a slot the user explicitly asked for would not read
  ERR_PRESET_STORE = 4, // ... or would not write
} ErrorBit;

uint8_t error_any(const EngineState* es);
uint8_t error_get(const EngineState* es, uint8_t bit);
void error_set(EngineState* es, uint8_t bit);
void error_clear(EngineState* es);

#endif
