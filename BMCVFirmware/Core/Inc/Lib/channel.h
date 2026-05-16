#ifndef INC_LIB_CHANNEL_H_
#define INC_LIB_CHANNEL_H_

#include "ux_state.h"
#include <stdint.h>

void init_channel(const ChannelSetup* ch, UxState* state);

void reset_channel(const ChannelSetup* ch, UxState* state, int8_t scene);

void update_channel(const ChannelSetup* ch, UxState* state);

void reset_channel_phase(const ChannelSetup* ch, UxState* state);

void compute_channel(const ChannelSetup* ch, UxState* state);

void write_channel_led(const ChannelSetup* ch, UxState* state);

void write_channel_dac(const ChannelSetup* ch, UxState* state);

uint8_t read_channel_trig_state(const ChannelSetup* ch);

#endif /* INC_LIB_CHANNEL_H_ */
