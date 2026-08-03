#ifndef BMCV_LED_COLOR_H_
#define BMCV_LED_COLOR_H_

#include "color_presets.h"
#include "engine_state.h"
#include <math.h>

// Framebuffer bytes -> what a screen should show.
//
// led_fb.c drives real WS2812s, which read as painfully bright above VAL_MED,
// so the layered renderer caps confirmations there and draws base layers at
// VAL_LOW (8). Handing those numbers straight to a display as 8/255 and 32/255
// makes almost everything the renderer draws invisible. So: treat VAL_MED as
// full scale, normalise the hue to full saturation so a dim red still reads as
// red rather than as near-black, and put the brightness through a perceptual
// curve.
//
// web/leds.js:ledStyle() is the same two constants and the same maths. They
// have to agree, or the two frontends show different modules.

#define LED_FULL_VALUE VAL_MED
#define LED_GAMMA 0.45f

typedef struct
{
  float r, g, b; // 0..1, hue at full saturation scaled by the level below
} LedColor;

static inline LedColor led_color_of(LedRgb led)
{
  uint8_t peak = led.r > led.g ? led.r : led.g;
  if (led.b > peak)
    peak = led.b;

  LedColor c = {0.0f, 0.0f, 0.0f};
  if (peak == 0)
    return c;

  float level = powf((float) peak / (float) LED_FULL_VALUE, LED_GAMMA);
  if (level > 1.0f)
    level = 1.0f;

  float k = level / (float) peak;
  c.r     = (float) led.r * k;
  c.g     = (float) led.g * k;
  c.b     = (float) led.b * k;
  return c;
}

#endif /* BMCV_LED_COLOR_H_ */
