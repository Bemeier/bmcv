#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

uint8_t error_any();
uint8_t error_get(uint8_t bit);
void error_set(uint8_t bit);
void error_clear();


#endif