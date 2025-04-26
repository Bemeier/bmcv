#include "envelope.h"

void trigger_envelope(ENVELOPE * env, uint32_t attack_us, uint32_t decay_us) {
    // Calculate step sizes (avoiding float)
	env->attack_us = attack_us;
	env->decay_us  = decay_us;
	env->value = 0;
	env->state = ATTACK;
}

uint16_t update_envelope(ENVELOPE * env, uint32_t elapsed_us) {
    switch (env->state) {
        case ATTACK:
            if (env->value < ENV_MAX) {
                uint16_t step = (elapsed_us * ENV_MAX) / env->attack_us;
                env->value += step;
                if (env->value >= ENV_MAX) {
                	env->value = ENV_MAX;
                    env->state = DECAY;
                }
            }
            break;

        case DECAY:
            if (env->value > 0) {
            	uint16_t step = (elapsed_us * ENV_MAX) / env->decay_us;
                if (step >= env->value) {
                	env->value = 0;
                	env->state = IDLE;
                } else {
                	env->value -= step;
                }
            }
            break;

        case IDLE:
        default:
        	env->value = 0;
            break;
    }
    return env->value;
}
