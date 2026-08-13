#ifndef BMCV_LED_COLOR_H_
#define BMCV_LED_COLOR_H_

#include "engine_state.h"
#include "led_curve.h"
#include <math.h>

// Framebuffer -> what a screen should show.
//
// Three steps, and the middle one is the whole reason this file is not two
// lines long.
//
// Duty -> light. The three dies are not equally efficient - green is a little
// under four times blue - so equal duty is nowhere near equal light.
//
// Light -> colour. This is the step that is easy to miss and looks like a
// palette problem when it is missing. How much light a die puts out and how
// blue something looks are different questions: a blue at SAT_HIG carries a
// third as much green duty as blue, and after the efficiency above that green
// out-luminates the blue - so ranking the primaries by light says "green", when
// the thing is plainly blue to look at. Dividing each die's light by what the
// matching screen primary contributes to a picture undoes that, and is what
// turns emitted light back into something with a hue.
//
// Colour -> pixels. sRGB is not linear in light, so the last step encodes.
//
// Brightness rides separately, on the total light rather than on the brightest
// primary. led_set_hsv balances every colour to the same total for a given
// value, so this makes what a screen draws depend on the value and not on which
// dies the hue happened to use - which is the point of balancing it.
//
// web/leds.js:ledStyle() is the same maths against the same constants. They
// have to agree, or the two frontends show different modules.

typedef struct
{
  float r, g, b; // 0..1, hue at full saturation scaled by the level below
} LedColor;

static inline float led_encode(float linear)
{
  if (linear <= 0.0f)
    return 0.0f;
  if (linear >= 1.0f)
    return 1.0f;
  return powf(linear, 1.0f / LED_DISPLAY_GAMMA);
}

// Total light out, in the units LED_PALETTE_REF and LED_CV_CEIL are written in.
static inline float led_light_of(LedRgb led)
{
  return ((float) led.r * LED_W_RED + (float) led.g * LED_W_GREEN + (float) led.b * LED_W_BLUE) / (float) LED_UNIT;
}

// What the caller should spend on opacity: the total light against the
// brightest thing the renderer draws.
static inline float led_level_of(LedRgb led) { return led_encode(led_light_of(led) / LED_DISPLAY_FULL); }

static inline LedColor led_color_of(LedRgb led)
{
  // Each die's light, then back to the screen primary that would contribute the
  // same share of the picture.
  float sr = (float) led.r * LED_W_RED / LED_Y_RED;
  float sg = (float) led.g * LED_W_GREEN / LED_Y_GREEN;
  float sb = (float) led.b * LED_W_BLUE / LED_Y_BLUE;

  float peak = sr > sg ? sr : sg;
  if (sb > peak)
    peak = sb;

  LedColor c = {0.0f, 0.0f, 0.0f};
  if (peak <= 0.0f)
    return c;

  // Hue at full saturation, brightness left to the caller to spend on opacity:
  // an SVG fill at a low RGB reads as a dark patch, not as a dim emitter.
  float level = led_level_of(led);
  c.r         = led_encode(sr / peak) * level;
  c.g         = led_encode(sg / peak) * level;
  c.b         = led_encode(sb / peak) * level;
  return c;
}

#endif /* BMCV_LED_COLOR_H_ */
