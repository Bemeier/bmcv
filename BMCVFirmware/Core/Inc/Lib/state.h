#ifndef INC_LIB_STATE_H_
#define INC_LIB_STATE_H_

#include "hw_setup.h"
#include <stdint.h>

#define STATE_RINGBUF_SIZE 2

#define CTRL_DEFAULT 0

#define SLIDER_MIN_VALUE 400
#define SLIDER_MAX_VALUE 7661

#define FAST_BLINK_PERIOD 300000
#define SLOW_BLINK_PERIOD 800000

typedef enum
{
    INPUT_DEFAULT, // Available for quantizing/adding
    INPUT_CLOCK,
    INPUT_RESET,
    INPUT_SLIDER,
    INPUT_MODE_COUNT
} InputMode;

typedef enum
{
    SHAPE_LFO,
    SHAPE_STEPPED_RANDOM,
    SHAPE_MODE_COUNT,
} ChannelShapeMode;

typedef enum
{
    QUANTIZE_DISABLED,
    QUANTIZE_CONTINUOUS,
    QUANTIZE_TRIG_SRC,
    QUANTIZE_MODE_COUNT,
} ChannelQuantizeMode;

typedef enum
{
    CH_PARAM_FRQ,
    CH_PARAM_SHP,
    CH_PARAM_MOD,
    CH_PARAM_PHS,
    CH_PARAM_AMP,
    CH_PARAM_OFS,
    CH_PARAM_INP,
    CH_PARAM_COUNT
} ChannelParameters;

typedef enum
{
    SHIFT_STATE_STA,
    SHIFT_STATE_SYS,
    SHIFT_STATE_QNT,
    SHIFT_STATE_MON,
    SHIFT_STATE_SAV,
    SHIFT_STATE_STB,
    SHIFT_STATE_MUT,
    SHIFT_STATE_CPY,
    SHIFT_STATE_CLR,
    SHIFT_STATE_NONE,
    SHIFT_STATE_COUNT
} ShiftStates;

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
    int8_t src_trig;
    int8_t shape_mode;
    ChannelQuantizeMode quantize_mode;
    int16_t params[N_SCENES][CH_PARAM_COUNT];
} ChannelConfig;

typedef struct __attribute__((packed))
{
    uint8_t clock_div;
    uint8_t scene_a;
    uint8_t scene_b;
    uint8_t current_preset;
    uint16_t quantize_mask;
    InputMode input_mode[N_INPUTS];
    ChannelConfig channel_state[N_ENCODERS];
} EngineConfig;

// TODO: Configure quantization pre/post LFO?
//   - When we add offset from cv, pre LFO could be nice for vibrato
//   - Otherwise, post LFO makes more sense, as we can use it for sequences
// ...

typedef struct
{
    uint32_t time; // timestamp of state
    uint32_t dt;   // time since last state

    uint16_t slider_state;

    uint8_t trigger_src[N_INPUTS + N_CHANNELS];

    uint8_t button_state[N_BUTTONS];       // if > 0, button is currently pressed
    uint32_t button_pressed_t[N_BUTTONS];  // how long button is pressed so far
    uint32_t button_released_t[N_BUTTONS]; // how long button was pressed once released

    int16_t encoder_state[N_ENCODERS];
    int16_t encoder_delta[N_ENCODERS]; // change of encoder since last state

    int16_t input_state[N_INPUTS];
} HwState;

typedef struct
{
    // Scene
    uint8_t scenes_contribution[N_SCENES];

    // Channel
    uint32_t channels_mark_for[N_CHANNELS];
    uint8_t channels_mark_hue[N_CHANNELS];
    int16_t channels_output_level[N_CHANNELS];
    float channels_shared_phase[N_CHANNELS];
    float channels_phase_correction[N_CHANNELS];

    int16_t cgcd[N_CHANNELS];
    float cphsc[N_CHANNELS];
    float csphs[N_CHANNELS];
    float cfrm[N_CHANNELS];
    float cshp[N_CHANNELS];
    float cmod[N_CHANNELS];

    // UX State
    int8_t active_scene;
    int8_t momentary_scene;
    uint16_t momentary_active_for;
    uint8_t selected_param;
    uint8_t shift_state;

    // Render State
    uint8_t blink_slow;
    uint8_t blink_fast;

    float engine_fps;
    float dac_fps;

} EngineState;

#endif /* INC_LIB_STATE_H_ */
