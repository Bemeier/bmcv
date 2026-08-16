#include "pwm.h"
#include "helpers.h"

// PWM: a gate with an envelope on it.
//
// SHP is the pulse width. MOD splits ramp time between the two edges, each
// confined to its own segment - the rise inside the on-time, the fall inside
// the off-time - so the width still means what it says at either extreme. MOD 0
// is a hard gate, negative snaps up and decays, positive swells and then drops.
//
// The ramps are curved rather than linear, which is what makes the negative
// side read as an envelope rather than as a triangle: the attack is concave
// (quick off the floor, easing into the plateau) and the decay convex (steep,
// then a tail). One multiply each, and no expf on a path that runs eight times
// a tick.
float pwm_shape(float phase, float shape, float mod)
{
  // Off both end stops, so the pulse never disappears entirely.
  float width = fclamp(0.5f + shape * 0.48f, 0.02f, 0.98f);

  if (phase < width)
  {
    float rise = (mod > 0.0f) ? mod * width : 0.0f;
    if (phase >= rise)
      return 1.0f; // covers rise == 0, where the edge is instant
    float x = phase / rise;
    return -1.0f + 2.0f * (2.0f * x - x * x);
  }

  float fall = (mod < 0.0f) ? -mod * (1.0f - width) : 0.0f;
  float t    = phase - width;
  if (t >= fall)
    return -1.0f;
  float y = 1.0f - t / fall;
  return -1.0f + 2.0f * y * y;
}
