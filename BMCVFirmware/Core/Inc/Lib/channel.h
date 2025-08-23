#ifndef INC_LIB_CHANNEL_H_
#define INC_LIB_CHANNEL_H_

#include "scene.h"
#include "state.h"
#include <stdint.h>

#define N_FREQ_MULTIPLIERS 24
#define N_FREQ_SCALE 255

static const int16_t quantized_multipliers[N_FREQ_MULTIPLIERS] = {
    -128 * 255, // 1/128
    -64 * 255,  // 1/64
    -32 * 255,  // 1/32
    -24 * 255,  // 1/24
    -16 * 255,  // 1/16
    -12 * 255,  // 1/12
    -8 * 255,   // 1/8
    -6 * 255,   // 1/6
    -5 * 255,   // 1/5
    -4 * 255,   // 1/4
    -3 * 255,   // 1/3
    -2 * 255,   // 1/2
    1 * 255,    // 1
    2 * 255,    // 2
    3 * 255,    // 3
    4 * 255,    // 4
    5 * 255,    // 5
    6 * 255,    // 6
    8 * 255,    // 8
    12 * 255,   // 12
    16 * 255,   // 16
    24 * 255,   // 24
    32 * 255,   // 32
    64 * 255,   // 64
};

// TODO: Even dividers: green
static const uint8_t quantized_multipliers_colors[N_FREQ_MULTIPLIERS] = {
    HUE_GREEN,  // 1/128
    HUE_CYAN,   // 1/64
    HUE_GREEN,  // 1/32
    HUE_RED,    // 1/24
    HUE_GREEN,  // 1/16
    HUE_RED,    // 1/12
    HUE_GREEN,  // 1/8
    HUE_RED,    // 1/6
    HUE_YELLOW, // 1/5
    HUE_GREEN,  // 1/4
    HUE_RED,    // 1/3
    HUE_GREEN,  // 1/2
    HUE_CYAN,   // 1
    HUE_GREEN,  // 2
    HUE_RED,    // 3
    HUE_GREEN,  // 4
    HUE_YELLOW, // 5
    HUE_RED,    // 6
    HUE_GREEN,  // 8
    HUE_RED,    // 12
    HUE_GREEN,  // 16
    HUE_RED,    // 24
    HUE_CYAN,   // 32
    HUE_GREEN   // 64
};

static uint8_t quantize_mode_color[QUANTIZE_MODE_COUNT] = {HUE_RED, HUE_MAGENTA, HUE_CYAN, HUE_GREEN};

typedef struct
{
    int8_t button;
    int8_t led;
    int8_t encoder;
    int8_t dac_channel;
    int8_t assigned_adc;
    int8_t adc_polarized;
    uint8_t morph_amount;
    int16_t params[N_PARAMS][N_SCENES];
    int16_t offset[N_SCENES];
    int16_t amplitude[N_SCENES];
    int16_t frequency[N_SCENES];
    int16_t phase[N_SCENES];
    int16_t shape[N_SCENES];
    uint32_t blink_until;
    uint32_t blink_hue;
    float shared_phase;
    ChannelQuantizeMode quantize_mode;

    // amplitude, frequency, phase, shape ...
} Channel;

void init_channel(Channel* ch);

// Updating => calculating output value
void update_channel(Channel* ch, State* state);

void compute_channel(Channel* ch, State* state, Scene* scenes);

void write_channel(Channel* ch, State* state);

#endif /* INC_LIB_CHANNEL_H_ */
