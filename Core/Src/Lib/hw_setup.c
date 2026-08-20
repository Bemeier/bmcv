#include "hw_setup.h"
#include "color_presets.h"

static const HwSetup hw_setup = {

    // ADC/DAC
    .input_adc_idx   = {2, 3, 0, 1},
    .channel_dac_idx = {7, 3, 5, 1, 6, 2, 4, 0},

    // Buttons & Encoders
    .channel_encoder_idx  = {3, 2, 4, 5, 1, 0, 7, 6},
    .channel_button_idx   = {1, 2, 4, 0, 3, 5, 7, 6},
    .quantizer_button_idx = {11, 10, 9, 8, 13, 14, 15, 18, 16, 19, 17, 20},
    .ctrl_button_idx      = {10, 8, 12, 15, 16, 17, 23, 22, 21},
    .scene_button_idx     = {11, 9, 13, 14, 18, 19, 20},

    // LEDs
    .channel_led_idx          = {5, 4, 7, 6, 3, 2, 0, 1},
    .quantizer_button_led_idx = {20, 8, 19, 9, 18, 17, 11, 16, 12, 15, 13, 14},
    .ctrl_button_led_idx      = {8, 9, 10, 11, 12, 13, -1, -1, -1},
    .scene_button_led_idx     = {20, 19, 18, 17, 16, 15, 14},

    // The six lit caps, and the one place red and green are not a voltage: a cap
    // is a label. It never shows a value, so it cannot be misread as one, which
    // is why PHS keeps red and AMP keeps green while the rings around them do
    // not. FRQ's hue is the same 200 it always was; only its name changed when
    // HUE_MAGENTA and HUE_PURPLE turned out to be the same position.
    .ctrl_button_color = {HUE_PURPLE, HUE_YELLOW, HUE_CYAN, HUE_RED, HUE_GREEN, HUE_BLUE, 0, 0, 0},
};

const HwSetup* HwSetup_Get(void) { return &hw_setup; }
