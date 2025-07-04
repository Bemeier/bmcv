#ifndef INC_LIB_STATE_H_
#define INC_LIB_STATE_H_

#include <stdint.h>

#define N_ENCODERS 8
#define N_BUTTONS 24

enum ChannelEditMode {
  DEFAULT, // default setting based on shape?
  LEVEL,
  AMPLITUDE,
  FREQUENCY,
  PHASE
}

typedef struct {
    
    uint8_t active_scene_id; // TODO: will be multiple, position based?
    uint8_t active_channel_id;
    enum ChannelEditMode channel_edit_mode;
    uint8_t active_channel_edit_mode; // enum: NONE, AMPL OPFFSET

    uint32_t time; // timestamp of state
    uint16_t dt; // time since last state
    //uint32_t button_pressed_start[N_BUTTONS]; // if > 0, button is currently pressed. holds the value for how long it's already pressed.
    uint8_t button_state[N_BUTTONS]; // if > 0, button is currently pressed. holds the value for how long it's already pressed.
    uint16_t button_pressed_t[N_BUTTONS]; // if > 0, button is currently pressed. holds the value for how long it's already pressed.
    uint16_t button_released_t[N_BUTTONS]; // if > 0, holds the value of for how long this button was pressed once it was released.
    int16_t encoder_state[N_ENCODERS];
    int8_t encoder_delta[N_ENCODERS]; // change of encoder since last state
    uint16_t slider_position;
} State;

// Setup ring buffer

#endif /* INC_LIB_STATE_H_ */