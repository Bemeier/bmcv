#ifndef INC_LIB_STATE_H_
#define INC_LIB_STATE_H_

#include <stdint.h>

#define N_INPUTS 4
#define N_ENCODERS 8
#define N_CHANNELS 8
#define N_BUTTONS 24
#define N_PARAMS 6
#define N_SCENES 7
#define N_CTRL_BUTTONS 9

#define PARAM_OFS 0
#define PARAM_AMP 1
#define PARAM_FRQ 2
#define PARAM_SHP 3
#define PARAM_PHS 4
#define PARAM_INP_AMP 5

#define CTRL_DEFAULT 0

#define SLIDER_MIN_VALUE 400
#define SLIDER_MAX_VALUE 7661

#define FAST_BLINK_PERIOD 300
#define SLOW_BLINK_PERIOD 800

#define CTRL_FRQ (1 << 0) // LFO Frequency
#define CTRL_SHP (1 << 1) // LFO Shape
#define CTRL_PHS (1 << 2) // LFO Phase
#define CTRL_INP (1 << 3) // Channel Input assign (press encoder cycles through input, left/right is amplification)
#define CTRL_AMP (1 << 4) // LFO Amplitude
#define CTRL_OFS (1 << 5) // LFO Amplitude

#define CTRL_STL (1 << 7)  // Set Scene on Slider Left
#define CTRL_SAV (1 << 8)  // ???
#define CTRL_SYS (1 << 9)  // System configuration (input config, clock division, slew, play mode)
#define CTRL_MON (1 << 10) // $%? - auditions cv on buttons and shows currently assigned input for last touched channel
#define CTRL_SEQ (1 << 11) // Sequence Scenes
#define CTRL_STR (1 << 12) // Set Scene on slider Right

#define CTRL_QNT (1 << 13) // Quantizer control state
#define CTRL_CPY (1 << 14) // Copy
#define CTRL_CLR (1 << 15) // Clear

#define HUE_RED 252
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
    INPUT_TRIG_QUANTIZE,
    INPUT_MODE_COUNT
} InputMode;

typedef enum
{
    QUANTIZE_DISABLED,
    QUANTIZE_CONTINUOUS,
    QUANTIZE_TRIG_INPUT,
    QUANTIZE_MODE_COUNT,
} ChannelQuantizeMode;

#define FRAM_MAGIC 0x424D4356
#define FRAM_CONFIG_SLOT_SIZE 896
#define FRAM_CONFIG_SLOTS 9
#define FRAM_CONFIG_BASE_ADDR 0x0000
#define CONFIG_STATE_VERSION 1

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t crc;
} FramRecordHeader;

typedef struct __attribute__((packed))
{
    int8_t src_input;
    ChannelQuantizeMode quantize_mode;
    int16_t params[N_SCENES][N_PARAMS];
} ChannelState;

typedef struct __attribute__((packed))
{
    uint8_t clock_div;
    uint8_t scene_l;
    uint8_t scene_r;
    uint8_t current_preset;
    InputMode input_mode[N_INPUTS];
    ChannelState channel_state[N_ENCODERS];
} ConfigState;

typedef struct __attribute__((packed))
{
    FramRecordHeader hdr;
    ConfigState data;
} ConfigStateRecord;

_Static_assert(sizeof(ConfigStateRecord) <= FRAM_CONFIG_SLOT_SIZE, "ConfigState too large for FRAM slot");

// TODO: Configure quantization pre/post LFO?
//   - When we add offset from cv, pre LFO could be nice for vibrato
//   - Otherwise, post LFO makes more sense, as we can use it for sequences
// ...

typedef struct
{
    uint32_t time; // timestamp of state
    uint16_t dt;   // time since last state

    uint8_t button_state[N_BUTTONS];       // if > 0, button is currently pressed
    uint16_t button_pressed_t[N_BUTTONS];  // how long button is pressed so far
    uint16_t button_released_t[N_BUTTONS]; // how long button was pressed once released

    int16_t encoder_state[N_ENCODERS];
    int16_t encoder_delta[N_ENCODERS]; // change of encoder since last state

    int16_t input_state[N_INPUTS];

    // Remove these?
    uint16_t ctrl_active_t;
    int8_t ctrl_scene_hold;
    int8_t ctrl_scene_released;
} SystemState;

typedef struct
{
    int8_t active_scene;
    uint8_t selected_param;
    uint8_t shift_state;

    uint8_t blink_slow;
    uint8_t blink_fast;
    uint16_t slider;
    uint16_t quantize_mask;

    SystemState* system;
} BaseState;

#endif /* INC_LIB_STATE_H_ */
