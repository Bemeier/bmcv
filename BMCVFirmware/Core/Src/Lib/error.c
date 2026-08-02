#include "error.h"
#include <stdint.h>

void error_set(EngineState* es, uint8_t bit) { es->error_flags |= (uint8_t) (1u << bit); }

void error_clear(EngineState* es) { es->error_flags = 0; }

uint8_t error_get(const EngineState* es, uint8_t bit) { return (es->error_flags >> bit) & 1u; }

uint8_t error_any(const EngineState* es) { return es->error_flags > 0; }
