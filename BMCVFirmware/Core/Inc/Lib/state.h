#ifndef INC_LIB_STATE_H_
#define INC_LIB_STATE_H_

#include <stdint.h>

#define N_INPUTS 4
#define N_ENCODERS 8
#define N_CHANNELS 8
#define N_BUTTONS 24
#define N_PARAMS 5
#define N_SCENES 7
#define N_CTRL_BUTTONS 9

#define PARAM_OFS 0
#define PARAM_AMP 1
#define PARAM_FRQ 2
#define PARAM_SHP 3
#define PARAM_PHS 4

#define CTRL_DEFAULT 0

#define SLIDER_MIN_VALUE 400
#define SLIDER_MAX_VALUE 7661
#define LATCH_DEADZONE 400

#define FAST_BLINK_PERIOD 300
#define SLOW_BLINK_PERIOD 800

#define CTRL_SHP (1 << 0) // LFO Shape
#define CTRL_PHS (1 << 1) // LFO Phase
#define CTRL_AMP (1 << 2) // LFO Amplitude
#define CTRL_OFS (1 << 3) // LFO Amplitude
#define CTRL_FRQ (1 << 4) // LFO Frequency

#define CTRL_MRP (1 << 5) // Channel Morph
#define CTRL_INP (1 << 6) // Channel Input assign (press encoder cycles through input, left/right is amplification)

#define CTRL_STL (1 << 7)  // Set Scene on Slider Left
#define CTRL_STR (1 << 8)  // Set Scene on slider Right
#define CTRL_LAT (1 << 9)  // Latch/activate scene (on button release!) - will be activated at current slider position
#define CTRL_SEQ (1 << 10) // Sequence Scenes
#define CTRL_SYS (1 << 11) // System configuration (input config, clock division, slew, play mode)
#define CTRL_MON (1 << 12) // $%? - auditions cv on buttons and shows currently assigned input for last touched channel

#define CTRL_QNT (1 << 13) // Quantizer control state
#define CTRL_CPY (1 << 14) // Copy
#define CTRL_CLR (1 << 15) // Clear

#define ALT_CTRL_IN1 (1 << 0) // Input 1
#define ALT_CTRL_IN2 (1 << 1) // Input 2
#define ALT_CTRL_IN3 (1 << 2) // Input 3
#define ALT_CTRL_IN4 (1 << 3) // Input 4

#define ALT_CTRL_CLK (1 << 4) // Clock control
#define ALT_CTRL_PLY (1 << 5) // Play/stop

#define HUE_RED 245
#define HUE_ORANGE 30
#define HUE_YELLOW 65
#define HUE_GREEN 80
#define HUE_CYAN 120
#define HUE_BLUE 160
#define HUE_MAGENTA 200

// Consider:
//  - Slew time, ...
//  - MUTE

typedef enum
{
    INPUT_DEFAULT, // Available for quantizing/adding
    INPUT_CLOCK,
    INPUT_RESET,
    INPUT_SLIDER,
} InputMode;

typedef enum
{
    QUANTIZE_DISABLED,
    QUANTIZE_CONTINUOUS,
    QUANTIZE_TRIG_INPUT,
    QUANTIZE_TRACK_INPUT,
    QUANTIZE_MODE_COUNT,
} ChannelQuantizeMode;

// TODO: Channel Mute?

// TODO: Configure quantization pre/post LFO?
//   - When we add offset from cv, pre LFO could be nice for vibrato
//   - Otherwise, post LFO makes more sense, as we can use it for sequences
// ...

typedef struct
{
    uint8_t active_scene_id; // TODO: will be multiple, position based?
    uint8_t active_channel_id;

    uint32_t time; // timestamp of state
    uint16_t dt;   // time since last state

    uint8_t button_state[N_BUTTONS];       // if > 0, button is currently pressed
    uint16_t button_pressed_t[N_BUTTONS];  // how long button is pressed so far
    uint16_t button_released_t[N_BUTTONS]; // how long button was pressed once released

    int16_t encoder_state[N_ENCODERS];
    int16_t encoder_delta[N_ENCODERS]; // change of encoder since last state

    int16_t input_state[N_INPUTS];

    int16_t channel_level[N_CHANNELS];

    uint8_t scene_l;
    uint8_t scene_r;
    int8_t scene_latch;
    uint16_t scene_latch_position;

    uint8_t clock_div;
    uint8_t playing_sequence;
    uint8_t slew_amount;

    uint16_t slider_position;

    uint16_t ctrl_active_t;
    uint16_t ctrl_flags;
    int8_t ctrl_scene_hold;
    int8_t ctrl_scene_released;
    int8_t ctrl_last_channel_touched;

    int8_t blink_slow;
    int8_t blink_fast;
    uint16_t quantize_mask;
} State;

#endif /* INC_LIB_STATE_H_ */
