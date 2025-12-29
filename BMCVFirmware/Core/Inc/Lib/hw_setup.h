#ifndef INC_LIB_HW_SETUP_H_
#define INC_LIB_HW_SETUP_H_

#include <stdint.h>

#define N_INPUTS 4
#define N_ENCODERS 8
#define N_CHANNELS 8
#define N_BUTTONS 24
#define N_SCENES 7
#define N_CTRL_BUTTONS 9
#define N_SEMITONES 12

typedef struct
{
    // ADC/DAC
    uint8_t input_adc_idx[N_INPUTS];
    int8_t channel_dac_idx[N_ENCODERS];

    // Buttons & Encoders
    int8_t channel_encoder_idx[N_ENCODERS];
    int8_t channel_button_idx[N_ENCODERS];
    uint8_t quantizer_button_idx[N_SEMITONES];
    int8_t ctrl_button_idx[N_CTRL_BUTTONS];
    int8_t scene_button_idx[N_SCENES];

    // LEDs
    int8_t channel_led_idx[N_ENCODERS];
    int8_t scene_button_led_idx[N_SCENES];
    uint8_t quantizer_button_led_idx[N_SEMITONES];
    int8_t ctrl_button_led_idx[N_CTRL_BUTTONS];
    uint8_t ctrl_button_color[N_CTRL_BUTTONS];
} HwSetup;

const HwSetup* HwSetup_Get(void);

#endif /* INC_LIB_HW_SETUP_H_ */
