#ifndef STEPPED_RANDOM_H
#define STEPPED_RANDOM_H

#include "stepped_norm.h"
#include <stdint.h>

// Fraction of each step spent sitting at its value before easing to the next.
//
// A property of the curve, not a menu: only ST_HOLD_SMOOTH is wired to a shape
// mode today, since three modes running the same algorithm at three hold values
// made a long list out of one idea. The other two stay because they are what
// the parameter's range means, and the tests exercise it - if hold comes back
// it should come back as a per-channel setting like the pattern length, not as
// more shape modes.
#define ST_HOLD_SMOOTH 0.0f
#define ST_HOLD_SEMI 0.5f
#define ST_HOLD_HARD 0.85f

// Where SHP and MOD go, which is most of what this shape is.
//
// The two used to be routed two ways - one for notes, one for modulation - and
// the notes one was dropped: it is where this arrived from rather than a second
// thing worth keeping, and everything it did well this reaches with a knob
// position instead of a mode. What survives is one routing, described here so
// that what the mode *is* lives in one function rather than spread through the
// value path.
typedef struct
{
  float hold;    // how much of a step is spent sitting at its value
  float bias;    // the distribution the finished value is reshaped into
  float terrace; // how far the values are gathered onto a few levels
} StDrive;

StDrive st_drive(float shape, float mod);

// Two reshapings of the finished value, and between them most of what a turn of
// SHP sounds like. Both are monotone, so neither moves a step out of order
// however far it moves the values - which is what lets a knob drive them hard
// without the result reading as a fresh draw, and is exactly what the suite's
// small-turn rule measures. Both land after the correction, so neither can
// disturb the level it just established, nor the loop point, nor the continuity
// of the curve.

// Levels per unit, so 1.5 puts them at 0 and +/-2/3 - three plateaus across a
// pattern that spans about 1.5. More would make each cell too narrow for a
// value to be pulled anywhere audible, which is what 3 did.
#define ST_TERRACE_LEVELS 1.5f

// Never the whole way. At full depth the map is flat across most of a cell,
// which is a quantiser: neighbouring settings would collapse onto identical
// patterns, the opposite of what this is for.
#define ST_TERRACE_LIMIT 0.9f

// Gathers the values onto those levels, so the pattern reads as a few plateaus
// rather than as a continuous spread.
//
// Not a quantiser, for a reason that had to be measured: snapping to the
// nearest level is a staircase, and a staircase applied to a moving value is a
// discontinuity - 0.23 of a jump at every crossing, which is a click. So each
// cell is compressed toward its own centre instead. Values bunch on the levels,
// the ramps between them survive, and because a cell still spans exactly its
// own width the curve joins at every boundary.
//
// The compressor is a quartic rather than st_shape_blend, which was the first
// try: halving a cell moves a value by a fraction of that cell's width and
// nothing reaches a plateau - measured, the pattern's values shifted by 0.04 at
// full depth. The quartic pulls eight times harder in the middle of a cell,
// still reaches exactly its edge, and costs two multiplies.
static inline float st_terrace_map(float v, float depth)
{
  if (depth <= 0.0f)
  {
    return v;
  }

  float u    = v * ST_TERRACE_LEVELS;
  float cell = (float) (int) (u + (u < 0.0f ? -0.5f : 0.5f)); // the nearest level
  float x    = 2.0f * (u - cell);                             // -1..1 across the cell
  float x2   = x * x;

  return (cell + 0.5f * lerp(x, copysignf(x2 * x2, x), fclamp(depth, 0.0f, ST_TERRACE_LIMIT))) / ST_TERRACE_LEVELS;
}

// Leans the values, without moving the ends of the range.
//
//   bias < 0  most pushed low, a few still reaching the top: modulation that
//             sits down and occasionally spikes
//   bias > 0  pushed away from the middle: gate-like, mostly high or mostly low
//
// This is the one thing the value path cannot do. st_shape_blend() is odd - it
// works on the middle symmetrically - so a distribution that leans, which is
// what most of the useful modulation shapes are, was out of reach however the
// levers were set. Applied after the correction for the same reason as the
// terracing, and for one more: the correction would otherwise measure the lean
// and centre it straight back out.
//
// Two stages each way, because one reaches u^2, which is a lean and not a bias.
static inline float st_bias_map(float v, float bias)
{
  float u = 0.5f * (fclamp(v, -1.0f, 1.0f) + 1.0f);

  if (bias < 0.0f)
  {
    float d = -bias;
    u       = u * (1.0f - d + d * u);
    u       = u * (1.0f - d + d * u);
  }
  else if (bias > 0.0f)
  {
    // smoothstep has zero slope at both ends, so values bunch against the rails
    u = lerp(u, smoothstep(u), bias);
    u = lerp(u, smoothstep(u), bias);
  }

  return 2.0f * u - 1.0f;
}

// Number of steps for a given index, for callers that want to display it.
// The index itself is a per-channel setting (ChannelConfig.st_length_idx), and
// the caller decides *when* a change to it takes effect - switching length
// mid-cycle moves the step grid under the playhead and jumps the output by up
// to a near-full-scale 1.8 of 2.0.
int st_length_for_index(int length_idx);

// Rhythmic random shape: a pattern of random values, locked to the beat.
//
//   phase       [0,1)  position in the cycle (already PLL-corrected by caller)
//   shape       [-1,1] the distribution the values are drawn into, and which
//                      pattern they come from. Left: mostly low, with the peaks
//                      still reaching - a modulation that sits down and
//                      occasionally spikes. Centre: even. Right: bunched
//                      against both rails, gate-like. The pattern advances
//                      underneath it, so the knob reaches different patterns as
//                      well as different distributions, and it is periodic -
//                      past the end is where it started.
//   mod         [-1,1] motion: at one end a new value every step, fully slewed
//                      between them; at the other mostly tied, sitting still on
//                      each value. Density, which steps tie, how far the beat
//                      swings, whether the cycle repeats a quarter-length
//                      phrase, and the ease all move together along it. Not
//                      periodic - its ends are opposite ends.
//   length_idx         pattern length; see st_length_for_index()
//   hold        [0,1)  how much of a step is spent sitting at its value. A
//                      parameter here because the shape is general, but a
//                      channel does not set it: st_drive() takes it from MOD.
//
// Returns [-1,1]. Stateless and deterministic: the same arguments always give
// the same value, which is what lets the caller re-derive phase from the PLL
// every tick and blend scenes without the pattern drifting.
//
// The knobs do not change how loud it is. The correction holds the pattern's
// peak-to-peak near constant and its centre near zero before SHP's bias leans
// it, so a turn changes what the shape does rather than how much of it there
// is. Where it sits afterwards is SHP's business; how loud is AMP's.
//
// Slot values depend only on the slot index and the morph - never on the
// length - so slot 0 reads the same at every length, and the correction's
// constant is shared by every length too. That is what makes changing length at
// a cycle boundary seamless.
float stepped_shape(float phase, float shape, float mod, int length_idx, float hold);

// The same shape, told how it is driven and what its correction is instead of
// working either out.
//
// Working it out means measuring the whole pattern - `length` slot evaluations
// - which is affordable once but not four thousand times a second per channel.
// So the engine keeps an StScan per channel, spreads that measurement over the
// cycle, and passes the result in here. Everything else is identical; at a
// standing setting the two agree exactly.
float stepped_shape_with(float phase, float shape, float mod, int length_idx, const StDrive* drive, const StNorm* norm);

// The correction for one setting, measured in full. O(length): for tests,
// tools, and the moments where a channel cannot wait for a scan - the first
// tick in a stepped mode, and a change of pattern length, which swaps the whole
// pattern at once.
StNorm st_norm_exact(float shape, float mod, int length_idx);

// What a channel remembers between ticks so it does not redo work that cannot
// have changed.
//
// The shape is sampled far faster than the pattern moves. What a step shows,
// and what the next one shows, depend on which step the playhead is in and not
// on where inside it - so at a 0.5Hz rate over 64 steps, ninety-one consecutive
// ticks were computing the same two numbers from scratch. All that actually
// varies within a step is the ease between them.
//
// Zeroed is "nothing cached", which is what a fresh EngineState gives.
//
// **The first four fields are a cache key, and they mirror the argument lists
// of st_morph() and st_step_pair() on purpose.** Every input those two read has
// to appear here, or a pattern that changed goes on being drawn from a stale
// pair - quietly, and sounding plausible. A lever added to StMorph that is
// driven by something not in this struct is the way this breaks;
// test_stepped_cache.c is what catches it, by asserting the cached path is
// bit-identical to the uncached one.
typedef struct
{
  float shape;
  float mod;
  int16_t length_idx;
  int16_t step; // which step from/to describe; -1 for none

  StMorph morph; // st_morph() for those knobs
  float from;    // what `step` shows
  float to;      // what the step after it shows
  uint8_t valid;
} StStepCache;

// stepped_shape_with(), with a channel's cache to work from.
//
// Returns bit-for-bit what stepped_shape_with() returns for the same arguments.
// The cache changes when the work happens, never what it produces, and the test
// holds it to that.
//
// Two ways it avoids work. Within a step - the common case by a wide margin -
// there is nothing to compute but the ease. Crossing into the *next* step, what
// the last one eased toward is by definition what this one eases from, so the
// pair advances for one slot evaluation instead of a walk of up to
// ST_JUMP_GRID. Only a jump - a phase correction, a knob turn, a rate high
// enough to skip a step outright - pays the full route.
float stepped_shape_cached(StStepCache* c, float phase, float shape, float mod, int length_idx, const StDrive* drive, const StNorm* norm);

// A channel's rolling measurement of its own pattern.
//
// Zeroed is "nothing measured yet", which is what a fresh EngineState gives and
// what makes the first tick take the exact route.
typedef struct
{
  StNorm norm;                // what the shape is being corrected by now
  StNorm target;              // what the last completed pass asked for
  float lo, hi;               // the pass in progress
  float anchor;               // slot 0, captured when the pass starts on it
  float shape_seen, mod_seen; // where the knobs were on the last tick
  int16_t slot;               // the next slot to measure
  int8_t length_idx;
  uint8_t measured;
  uint8_t moved; // the knobs moved during the pass in progress
} StScan;

// One slot of the measurement, to be called once per engine tick per stepped
// channel before stepped_shape_with().
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
void st_norm_scan(StScan* s, float shape, float mod, int length_idx, float dt_s, int may_measure);

#endif
