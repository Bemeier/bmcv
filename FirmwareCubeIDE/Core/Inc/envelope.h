#ifndef INC_ENVELOPE_H_
#define INC_ENVELOPE_H_
#include "stm32g4xx_hal.h"

#define ENV_MAX 8192

typedef enum { IDLE, ATTACK, DECAY } EnvelopeState;

typedef struct {
	EnvelopeState state;
	uint16_t value;
	uint32_t attack_us;
	uint32_t decay_us;
} ENVELOPE;

void trigger_envelope(ENVELOPE * env, uint32_t attack_ms, uint32_t decay_ms);
uint16_t update_envelope(ENVELOPE * env, uint32_t elapsed_us);

#endif /* INC_ENVELOPE_H_ */
