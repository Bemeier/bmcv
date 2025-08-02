#ifndef INC_ENVELOPE_H_
#define INC_ENVELOPE_H_
#include "stm32g4xx_hal.h" // IWYU pragma: keep

#define ENV_MAX 16384

typedef enum
{
    IDLE,
    ATTACK,
    DECAY
} EnvelopeState;

typedef struct
{
    EnvelopeState state;
    int16_t value;
    int16_t smoothed_value;
    int8_t shape;
    uint32_t phase_progress;  // 0 to ENV_MAX
    uint32_t phase_increment; // How fast phase progresses
    uint32_t phase_decrement; // How fast phase progresses
} ENVELOPE;

void trigger_envelope(ENVELOPE* env, uint32_t attack_ms, uint32_t decay_ms, int8_t shape);
int16_t update_envelope(ENVELOPE* env, uint32_t elapsed_us);

#endif /* INC_ENVELOPE_H_ */
