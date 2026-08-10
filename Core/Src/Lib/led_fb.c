#include "led_fb.h"
#include "color_presets.h"
#include "hw_setup.h"

// Colour conversion moved here from ws2811.c: it is presentation logic, not
// driver logic, and keeping it hardware-free makes it testable.

void led_set_rgb(UxState* state, int16_t idx, uint8_t r, uint8_t g, uint8_t b)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  LedRgb* led = &state->engine_state->leds[idx];
  led->r      = r;
  led->g      = g;
  led->b      = b;
}

void led_set_hsv(UxState* state, int16_t idx, uint8_t h, uint8_t s, uint8_t v)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  LedRgb* led = &state->engine_state->leds[idx];

  if (s == 0)
  {
    led->r = v;
    led->g = v;
    led->b = v;
    return;
  }

  uint8_t region    = h / 43;
  uint8_t remainder = (h - region * 43) * 6;

  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region)
  {
  case 0:
    led->r = v;
    led->g = t;
    led->b = p;
    break;
  case 1:
    led->r = q;
    led->g = v;
    led->b = p;
    break;
  case 2:
    led->r = p;
    led->g = v;
    led->b = t;
    break;
  case 3:
    led->r = p;
    led->g = q;
    led->b = v;
    break;
  case 4:
    led->r = t;
    led->g = p;
    led->b = v;
    break;
  default:
    led->r = v;
    led->g = p;
    led->b = q;
    break;
  }
}

// Bipolar level -> colour, in whichever converter domain the caller is in:
// green for positive, red for negative, with blue mixed in above half scale so
// the top half of the range stays readable. `half_scale` is the count that
// corresponds to 5V in that domain - ADC_5V or DAC_5V.
void led_set_bipolar(UxState* state, int16_t idx, int32_t val, int32_t half_scale)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  LedRgb* led = &state->engine_state->leds[idx];

  // Both callers pass a converter constant, so per_step is 32 or 128 and this
  // never fires. It is here because this is the exported entry point and the
  // two below are the convenience wrappers: a half_scale under VAL_MAX makes
  // per_step zero and the two divides below undefined, which on the module is a
  // hard fault rather than a wrong colour.
  int32_t per_step = half_scale / VAL_MAX;
  if (per_step <= 0)
    return;

  // Widened before negating and saturated on the way back: val is int32 and
  // -INT32_MIN is not representable in one, so the obvious `(val < 0) ? -val`
  // is undefined at that single input.
  int64_t wide       = (val < 0) ? -(int64_t) val : (int64_t) val;
  int32_t abs_val    = (int32_t) (wide > INT32_MAX ? INT32_MAX : wide);
  int32_t blue_range = abs_val - half_scale;
  uint8_t base_val   = VAL_MAX;

  if (blue_range < 0)
  {
    blue_range = 0;
    base_val   = (uint8_t) ((abs_val / per_step) & 0xFF);
  }

  led->b = (uint8_t) ((blue_range / per_step) & 0xFF);
  if (val > 0)
  {
    led->g = base_val;
    led->r = 0;
  }
  else
  {
    led->r = base_val;
    led->g = 0;
  }
}

void led_set_adcr(UxState* state, int16_t idx, int16_t val) { led_set_bipolar(state, idx, val, ADC_5V); }

void led_set_dac(UxState* state, int16_t idx, int32_t val) { led_set_bipolar(state, idx, val, DAC_5V); }

void led_clear_all(UxState* state)
{
  for (int16_t i = 0; i < LED_COUNT; i++)
  {
    led_set_rgb(state, i, 0, 0, 0);
  }
}
