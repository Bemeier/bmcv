#ifndef STEPPED_RANDOM_H
#define STEPPED_RANDOM_H

#include "stepped_random_norm.h"
#include <stdint.h>

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

// The same shape, told what its correction is instead of working it out.
//
// Working it out means measuring the whole pattern - `length` slot evaluations
// - which is affordable once but not four thousand times a second per channel.
// So the engine keeps an SrScan per channel, spreads that measurement over the
// cycle, and passes the result in here. Everything else is identical; at a
// standing setting the two agree exactly.
float stepped_random_with(float phase, float shape, float mod, int length_idx, float hold, const SrNorm* norm);

// The correction for one setting, measured in full. O(length): for tests,
// tools, and the moments where a channel cannot wait for a scan - the first
// tick in a stepped mode, and a change of pattern length, which swaps the whole
// pattern at once.
SrNorm sr_norm_exact(float shape, float mod, int length_idx);

// A channel's rolling measurement of its own pattern.
//
// Zeroed is "nothing measured yet", which is what a fresh EngineState gives and
// what makes the first tick take the exact route.
typedef struct
{
  SrNorm norm;                // what the shape is being corrected by now
  SrNorm target;              // what the last completed pass asked for
  float lo, hi;               // the pass in progress
  float anchor;               // slot 0, captured when the pass starts on it
  float shape_seen, mod_seen; // where the knobs were on the last tick
  int16_t slot;               // the next slot to measure
  int8_t length_idx;
  uint8_t measured;
  uint8_t moved; // the knobs moved during the pass in progress
} SrScan;

// One slot of the measurement, to be called once per engine tick per stepped
// channel before stepped_random_with().
//
// Costs nothing once a pattern is standing still and its measurement has
// settled: re-measuring an unchanged pattern can only produce the answer it
// already has. So what this really costs is a knob being turned, or a CV moving
// one - which is when it is doing something.
//
// `may_measure` is the caller's budget, and it matters. Measuring one slot is
// most of the cost of the shape itself - a step value walks a run of ties, each
// slot of it a four-tap contour and a motif fold - so eight channels each
// measuring every tick costs about what eight channels of the shape cost.
// Measured on the module: 88us added to a 310us tick, which the engine answered
// by dropping every third one. So the engine hands out one measurement per
// tick, in turn.
//
// The result is slewed rather than swapped in, so that a pass whose pattern
// changed under it - a length switch, a fast turn of SHP - moves the level
// smoothly instead of stepping it. `dt_s` is the tick's own length, because the
// engine has to be right at whatever rate its host ticks it.
void sr_norm_scan(SrScan* s, float shape, float mod, int length_idx, float dt_s, int may_measure);

#endif
