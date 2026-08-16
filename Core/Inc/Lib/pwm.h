#ifndef INC_LIB_PWM_H_
#define INC_LIB_PWM_H_

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
//
//   phase  [0,1)  position in the cycle, PLL-corrected by the caller
//   shape  [-1,1] the pulse width, off both end stops so the pulse never
//                 disappears entirely
//   mod    [-1,1] the ramps: 0 is a hard gate, negative snaps up and decays,
//                 positive swells and then drops
//
// Returns [-1,1]. Stateless and deterministic, like every other shape here.
float pwm_shape(float phase, float shape, float mod);

#endif /* INC_LIB_PWM_H_ */
