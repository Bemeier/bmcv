#ifndef INC_DRIVERS_BUTTONS_ENCODERS_H_
#define INC_DRIVERS_BUTTONS_ENCODERS_H_
#include "hw_setup.h"      // N_ENCODERS
#include "stm32g4xx_hal.h" // IWYU pragma: keep

uint8_t get_btn_state(uint8_t buttonIndex);

int16_t get_enc_state(uint8_t encoderIndex);

#endif /* INC_DRIVERS_BUTTONS_ENCODERS_H_ */
