#ifndef STEPPED_RANDOM_H
#define STEPPED_RANDOM_H

// Fraction of each step spent sitting at its value before easing to the next.
//
// A property of the curve, not a menu: only SR_HOLD_SMOOTH is wired to a shape
// mode today, since three modes running the same algorithm at three hold values
// made a long list out of one idea. The other two stay because they are what
// the parameter's range means, and the tests exercise it - if hold comes back
// it should come back as a per-channel setting like the pattern length, not as
// more shape modes.
#define SR_HOLD_SMOOTH 0.0f
#define SR_HOLD_SEMI 0.5f
#define SR_HOLD_HARD 0.85f

// Number of steps for a given index, for callers that want to display it.
// The index itself is a per-channel setting (ChannelConfig.sr_length_idx), and
// the caller decides *when* a change to it takes effect - switching length
// mid-cycle moves the step grid under the playhead and jumps the output by up
// to a near-full-scale 1.8 of 2.0.
int sr_length_for_index(int length_idx);

// Rhythmic random LFO shape.
//
//   phase       [0,1)  position in the cycle (already PLL-corrected by caller)
//   shape       [-1,1] morph: continuously reshapes the pattern, periodic so
//                      the knob wrapping past its end lands where it started.
//                      Moves three things around one closed loop - which values
//                      the steps hold, how those values are distributed (calm
//                      and close together, or pushed to the extremes), and how
//                      melodic the contour is (a walk, or independent leaps).
//   mod         [-1,1] density: how often a step ties to the previous value
//                      instead of taking a new one. 0 is the neutral 30% the
//                      pattern was designed around, -1 gives a new value every
//                      step, +1 leaves a handful of long notes per cycle.
//                      Alongside it and moving with it: which steps tie, how
//                      far the beat swings, and whether the cycle repeats a
//                      quarter-length phrase. Not periodic - the density is
//                      monotone across the knob, so -1 and +1 are opposite
//                      ends rather than the same place.
//   length_idx         pattern length; see sr_length_for_index()
//   hold        [0,1)  how step-like the curve is (see SR_HOLD_* above)
//
// Returns [-1,1]. Stateless and deterministic: the same arguments always give
// the same value, which is what lets the caller re-derive phase from the PLL
// every tick and blend scenes without the pattern drifting.
//
// Neither knob changes the level: the correction in the generated table holds
// the peak-to-peak near constant and the pattern centred, so a turn changes
// what the shape does and not how loud or how high it sits. That is what AMP
// and OFFSET are for.
//
// Slot values depend only on the slot index and the morph - never on the
// length - so slot 0 reads the same at every length, and the correction's
// constant is shared by every length too. That is what makes changing length at
// a cycle boundary seamless.
float stepped_random(float phase, float shape, float mod, int length_idx, float hold);

#endif
