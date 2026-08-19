#include "pwm.h"
#include "helpers.h"

// PWM: a gate with an envelope on it.
//
// SHP is the pulse width. MOD is how much of that pulse is ramp rather than
// plateau, and its sign is how the ramp is split between the two ends: 0 is a
// hard gate, the ends are one ramp filling the pulse - a pure decay one way, a
// pure attack the other - and everything between is attack, plateau, decay.
//
// MOD used to give each edge its own segment: the rise inside the on-time, the
// fall inside the off-time. That kept the width honest but it meant the two
// ramps lived on opposite sides of the pulse edge and so could never coexist,
// so no setting on the knob was an AD envelope. It also ran the decay's length
// *inverse* to the width, since what it had to spend was the off-time - a wide
// pulse had nowhere to decay into.
//
// Both ramps now live inside the pulse, which makes SHP the envelope's length
// as well as the gate's width. A short trigger with a long tail is a wide SHP
// at MOD -1 rather than a narrow one; nothing bleeds past the pulse either way.
//
// The ramps are curved rather than linear, which is what makes them read as an
// envelope rather than as a triangle: the attack is concave (quick off the
// floor, easing into the plateau) and the decay convex (steep, then a tail).
// One multiply each, and no expf on a path that runs eight times a tick.
float pwm_shape(float phase, float shape, float mod)
{
  // Off both end stops, so the pulse never disappears entirely.
  const float width = fclamp(0.5f + shape * 0.48f, 0.02f, 0.98f);

  if (phase >= width)
    return -1.0f;

  // Clamped because a scene crossfade of INT16_MIN divides to -1.000031, which
  // would make the ramp longer than the pulse it has to fit in: plateau_end
  // goes negative, the decay starts a fraction in, and MOD hard over stops
  // peaking at 1.0. Worth one clamp.
  const float m      = fclamp(mod, -1.0f, 1.0f);
  const float ramp   = (m < 0.0f ? -m : m) * width;
  const float attack = ramp * (0.5f + 0.5f * m);
  const float decay  = ramp - attack; // so attack + decay is exactly `ramp`

  if (phase < attack) // false when attack is 0, so nothing divides by it
  {
    float x = phase / attack;
    return -1.0f + 2.0f * (2.0f * x - x * x);
  }

  // Reached only while phase < width, so this is entered only when the decay
  // has room - which is what keeps the divide below safe at MOD >= 0.
  const float plateau_end = width - decay;
  if (phase >= plateau_end)
  {
    float y = 1.0f - (phase - plateau_end) / decay;
    return -1.0f + 2.0f * y * y;
  }

  return 1.0f;
}
