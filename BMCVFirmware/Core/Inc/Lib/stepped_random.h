#ifndef STEPPED_RANDOM_H
#define STEPPED_RANDOM_H

// Fraction of each step spent sitting at its value before easing to the next.
// Selected per channel by ChannelShapeMode.
#define SR_HOLD_SMOOTH 0.0f
#define SR_HOLD_SEMI 0.5f
#define SR_HOLD_HARD 0.85f

// Rhythmic random LFO shape.
//
//   phase  [0,1)  position in the cycle (already PLL-corrected by the caller)
//   shape  [-1,1] morph: continuously reshapes the pattern, periodic so the
//                 knob wrapping past its end lands back where it started
//   mod    [-1,1] pattern length, snapped to a curated set of step counts
//   hold   [0,1)  how step-like the curve is (see SR_HOLD_* above)
//
// Returns [-1,1]. Stateless and deterministic: the same arguments always give
// the same value, which is what lets the caller re-derive phase from the PLL
// every tick and blend scenes without the pattern drifting.
float stepped_random(float phase, float shape, float mod, float hold);

#endif
