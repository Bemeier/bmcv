#ifndef INC_LIB_STATE_H_
#define INC_LIB_STATE_H_

#include <stdint.h>

#define N_ENCODERS 8
#define N_BUTTONS 24

typedef enum
{
    DEFAULT, // default setting based on shape?
    LEVEL,
    AMPLITUDE,
    FREQUENCY,
    PHASE
} ChannelEditMode;

typedef enum
{
    NONE,
    AMPL_OFFSET,
    // add more as needed
} ChannelEditSubMode;

typedef struct
{
    uint8_t active_scene_id; // TODO: will be multiple, position based?
    uint8_t active_channel_id;
    ChannelEditMode channel_edit_mode;
    ChannelEditSubMode active_channel_edit_mode;

    uint32_t time; // timestamp of state
    uint16_t dt;   // time since last state

    uint8_t button_state[N_BUTTONS];       // if > 0, button is currently pressed
    uint16_t button_pressed_t[N_BUTTONS];  // how long button is pressed so far
    uint16_t button_released_t[N_BUTTONS]; // how long button was pressed once released

    int16_t encoder_state[N_ENCODERS];
    int8_t encoder_delta[N_ENCODERS]; // change of encoder since last state

    uint16_t slider_position;
} State;

#endif /* INC_LIB_STATE_H_ */