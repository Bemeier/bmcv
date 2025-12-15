#ifndef INC_LIB_CHANNEL_H_
#define INC_LIB_CHANNEL_H_

#include "scene.h"
#include "state.h"
#include <stdint.h>

#define N_FREQ_MULTIPLIERS 31
#define N_FREQ_SCALE 255

static const int16_t quantized_multipliers[N_FREQ_MULTIPLIERS] = {

    -127 * 255, // 1/128
    -63 * 255,  // 1/64
    -31 * 255,  // 1/32
    -23 * 255,  // 1/24
    -15 * 255,  // 1/16
    -11 * 255,  // 1/12
    -7 * 255,   // 1/8
    -5 * 255,   // 1/6
    -4 * 255,   // 1/5
    -3 * 255,   // 1/4
    -2 * 255,   // 1/3
    -1 * 255,   // 1/2
    -127,       // 2/3
    -85,        // 3/4
    -64,        // 4/5
    0,          // 1
    64,         // 5/4
    85,         // 4/3
    127,        // 3/2
    1 * 255,    // 2
    2 * 255,    // 3
    3 * 255,    // 4
    4 * 255,    // 5
    5 * 255,    // 6
    7 * 255,    // 8
    11 * 255,   // 12
    15 * 255,   // 16
    23 * 255,   // 24
    31 * 255,   // 32
    63 * 255,   // 32
    127 * 255,  // 128
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
    HUE_CYAN,   // 2/3
    HUE_CYAN,   // 3/4
    HUE_CYAN,   // 5/4
    HUE_GREEN,  // 1
    HUE_CYAN,   // 5/4
    HUE_CYAN,   // 4/3
    HUE_CYAN,   // 3/2
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
    HUE_GREEN,  // 64
    HUE_CYAN    // 64
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
    int8_t src_input;
    uint8_t id;
    uint8_t morph_amount;
    int16_t params[N_PARAMS][N_SCENES];
    int16_t offset[N_SCENES];
    int16_t amplitude[N_SCENES];
    int16_t input_amplitude[N_SCENES];
    int16_t frequency[N_SCENES];
    int16_t phase[N_SCENES];
    int16_t shape[N_SCENES];
    uint32_t blink_until;
    uint32_t blink_hue;
    int16_t output_level;
    int16_t gcd;
    float target_phase;
    float freq_multiplier;
    float normalized_phase;
    float shared_phase;
    float phase_correction;
    float diff;
    ChannelQuantizeMode quantize_mode;
} Channel;

void init_channel(Channel* ch);

void update_channel(Channel* ch, State* state);

void compute_channel(Channel* ch, State* state, Scene* scenes);

void write_channel(Channel* ch, State* state);

static float k_sync = 0.01f;

#endif /* INC_LIB_CHANNEL_H_ */
