#ifndef INC_LIB_CHANNEL_H_
#define INC_LIB_CHANNEL_H_

#include "state.h"
#include <stdint.h>

#define N_SCENES 7

typedef struct
{
    uint8_t button;
    uint8_t led;
    uint8_t encoder;
    uint8_t dac_channel;
    int16_t level[N_SCENES];

    // amplitude, frequency, phase, shape ...
} Channel;

void init_channel(Channel* ch);

// Updating => calculating output value
void update_channel(Channel* ch, State* state);

#endif /* INC_LIB_CHANNEL_H_ */
