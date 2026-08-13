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

// Lift an LED toward white without darkening anything already there, stopping
// short of white: it is brightened to the value `v` with its hue intact, then
// pulled toward neutral, keeping `keep` of the distance between its primaries.
// SAT_MAX keeps the colour exactly, 0 goes fully white.
//
// `v` is a value in the palette's sense - a brightness, not a duty - so a
// marker is the same strength whatever hue it lands on.
//
// The assignment marker is light laid *over* an element, not a colour replacing
// it: a marker that replaced would make a candidate *dimmer* than its
// neighbours through the quiet end of every pulse, which says the opposite of
// what the marker is for. And washing rather than whitening is what makes it
// subtle enough to leave running - an element pushed all the way to white loses
// the colour that says what it is.
//
// Washing rather than whitening is what makes the marker subtle enough to leave
// running. An element pushed all the way to white loses the colour that says
// what it is, so the marker and the element compete for the same LED; stopping
// part way, it reads as the element glowing.
//
// Over a *cleared* LED there is no colour to keep and this is white, which is
// what a held source's destinations want: there is nothing underneath them, and
// white is the whole vocabulary of assignment.
void led_wash(UxState* state, int16_t idx, uint16_t v, uint8_t keep);

// Bipolar CV level -> colour: green positive, red negative, amber at rest, and
// no third colour family. `half_scale` is whatever counts as 5V in the caller's
// converter domain; the ramp runs to twice it. The shape of it is in
// led_curve.h, which is the file to edit.
void led_set_bipolar(UxState* state, int16_t idx, int32_t val, int32_t half_scale);

// The same, for the two domains that actually occur.
void led_set_adcr(UxState* state, int16_t idx, int16_t val);
void led_set_dac(UxState* state, int16_t idx, int32_t val);

void led_clear_all(UxState* state);

// Per-LED dither state: the sub-step remainder each primary is owed.
typedef struct
{
  uint8_t r, g, b;
} LedDither;

// 8.8 framebuffer -> the driver's bytes, carrying the fraction forward. `out`
// is 3 * count bytes, in the driver's r,g,b order.
void led_fb_quantize(const LedRgb* fb, LedDither* acc, uint8_t* out, int16_t count);

#endif /* INC_LIB_LED_FB_H_ */
