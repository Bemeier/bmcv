#ifndef STEPPED_RANDOM_H
#define STEPPED_RANDOM_H

// Fraction of each step spent sitting at its value before easing to the next.
// Selected per channel by ChannelShapeMode.
#define SR_HOLD_SMOOTH 0.0f
#define SR_HOLD_SEMI 0.5f
#define SR_HOLD_HARD 0.85f

// Which pattern length a CH_PARAM_MOD value selects, as an index into the
// curated set. Separate from stepped_random() so the caller can decide *when*
// a change takes effect - switching length mid-cycle moves the step grid under
// the playhead and jumps the output by up to a near-full-scale 1.8 of 2.0.
int sr_length_index_from_mod(float mod);

// Number of steps for a given index, for callers that want to display it.
int sr_length_for_index(int length_idx);

// Rhythmic random LFO shape.
//
//   phase       [0,1)  position in the cycle (already PLL-corrected by caller)
//   shape       [-1,1] morph: continuously reshapes the pattern, periodic so
//                      the knob wrapping past its end lands where it started
//   length_idx         pattern length, from sr_length_index_from_mod()
//   hold        [0,1)  how step-like the curve is (see SR_HOLD_* above)
//
// Returns [-1,1]. Stateless and deterministic: the same arguments always give
// the same value, which is what lets the caller re-derive phase from the PLL
// every tick and blend scenes without the pattern drifting.
//
// Slot values depend only on the slot index and the morph - never on the
// length - so slot 0 reads the same at every length. That is what makes
// changing length at a cycle boundary seamless.
float stepped_random(float phase, float shape, int length_idx, float hold);

#endif
