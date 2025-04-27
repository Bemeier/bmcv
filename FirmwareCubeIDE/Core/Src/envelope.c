#include "envelope.h"

void trigger_envelope(ENVELOPE * env, uint32_t attack_us, uint32_t decay_us, int8_t shape) {
    // Calculate step sizes (avoiding float)
	env->phase_increment = ENV_MAX * 1000 / attack_us;
	env->phase_decrement = ENV_MAX * 1000 / decay_us;
	env->shape = shape;
	env->state = ATTACK;
}

uint32_t fast_sqrt(uint32_t x) {
    // Rough and fast sqrt approximation for embedded
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; // The second-to-top bit

    // "bit" starts at the highest power of four <= x
    while (bit > x) bit >>= 2;

    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

uint16_t apply_curve(uint16_t env, int8_t curve_amount) {
    if (curve_amount == 0) return env; // Linear

    uint32_t curved = env;

    curved = (curved * curved) / ENV_MAX;

    if (curve_amount < 0) {
        // Inverse curve
        curved = ENV_MAX - curved;
        curve_amount = -curve_amount;
    }

    uint32_t blended = ((env * (127 - curve_amount)) + (curved * curve_amount)) / 127;

    return blended;
}


uint32_t apply_curve_blend(uint32_t progress, int8_t shape) {
    if (shape == 0) return progress;

    uint32_t curved;
    uint32_t pp = progress;

    for (uint8_t i = 0; i < 2; i++) {
		if (shape > 0) {
			curved = (pp * pp) / ENV_MAX;
		} else {
			curved = fast_sqrt(pp * ENV_MAX);
		}
		pp = curved;
    }

    if (shape < 0) {
    	shape = -shape;
    }

    // Now blend between linear and curved
    uint32_t blended = ((progress * (127 - shape)) + (curved * shape)) / 127;

    return blended;
}


uint16_t update_envelope(ENVELOPE * env, uint32_t elapsed_us) {
    switch (env->state) {
        case ATTACK:
        	env->phase_progress += env->phase_increment * elapsed_us / 1000;
			if (env->phase_progress >= ENV_MAX) {
				env->state = DECAY;
				env->phase_progress = 0;
				// set different shape for decay?
			}
            break;

        case DECAY:
        	env->phase_progress += env->phase_decrement * elapsed_us / 1000;
            if (env->phase_progress >= ENV_MAX) {
            	env->state = IDLE;
				env->phase_progress = 0;
            }
            break;
        case IDLE:
        default:
			env->phase_progress = 0;
            break;
    }


    if (env->state == ATTACK) {
    	env->value = apply_curve_blend(env->phase_progress, env->shape);
    } else if (env->state == DECAY) {
    	env->value = ENV_MAX - apply_curve_blend(env->phase_progress, env->shape);
    } else {
    	env->value = 0;
    }
    env->smoothed_value += ((env->value - env->smoothed_value) >> 3);
    return env->smoothed_value;
}
