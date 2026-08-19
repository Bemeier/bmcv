#ifndef INC_LIB_PWM_H_
#define INC_LIB_PWM_H_

// PWM: a gate with an envelope on it.
//
// SHP is the pulse width. MOD is how much of that pulse is ramp rather than
// plateau, and its sign is how the ramp is split between the two ends: 0 is a
// hard gate, the ends are one ramp filling the pulse - a pure decay one way, a
// pure attack the other - and everything between is attack, plateau, decay.
// Both ramps live inside the pulse, so SHP is the envelope's length as much as
// the gate's width and nothing bleeds past it. See pwm.c for what MOD used to
// mean and why it stopped.
//
// The ramps are curved rather than linear, which is what makes them read as an
// envelope rather than as a triangle: the attack is concave (quick off the
// floor, easing into the plateau) and the decay convex (steep, then a tail).
// One multiply each, and no expf on a path that runs eight times a tick.
//
//   phase  [0,1)  position in the cycle, PLL-corrected by the caller
//   shape  [-1,1] the pulse width, off both end stops so the pulse never
//                 disappears entirely
//   mod    [-1,1] the envelope: 0 is a hard gate, negative shifts the ramp
//                 budget toward the decay, positive toward the attack, and the
//                 ends are one ramp filling the pulse
//
// Returns [-1,1]. Stateless and deterministic, like every other shape here.
float pwm_shape(float phase, float shape, float mod);

#endif /* INC_LIB_PWM_H_ */
