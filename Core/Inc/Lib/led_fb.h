#ifndef INC_LIB_LED_FB_H_
#define INC_LIB_LED_FB_H_

#include "ux_state.h"
#include <stdint.h>

// LED presentation layer: the UX code renders colours into
// EngineState.leds[] through these, and a separate flush step pushes the
// buffer to the WS2812 driver. Nothing here touches hardware, so LED
// behaviour is fully assertable in host tests.

void led_set_rgb(UxState* state, int16_t idx, uint8_t r, uint8_t g, uint8_t b);

void led_set_hsv(UxState* state, int16_t idx, uint8_t h, uint8_t s, uint8_t v);

// Bipolar CV level -> colour: green positive, red negative, blue ramping in
// with magnitude past 5V. `half_scale` is whatever counts as 5V in the
// caller's converter domain.
void led_set_bipolar(UxState* state, int16_t idx, int32_t val, int32_t half_scale);

// The same, for the two domains that actually occur.
void led_set_adcr(UxState* state, int16_t idx, int16_t val);
void led_set_dac(UxState* state, int16_t idx, int32_t val);

void led_clear_all(UxState* state);

#endif /* INC_LIB_LED_FB_H_ */
