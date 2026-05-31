#include "error.h"
#include <stdint.h>

static volatile uint8_t error_flags = 0;

void error_set(uint8_t bit) { error_flags |= (1 << bit); }

void error_clear() { error_flags = 0; }

uint8_t error_get(uint8_t bit) { return (error_flags >> bit) & 1; }

uint8_t error_any() { return error_flags > 0; }
