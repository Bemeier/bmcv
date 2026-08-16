#ifndef INC_LIB_STEPPED_RANDOM_PATTERN_H_
#define INC_LIB_STEPPED_RANDOM_PATTERN_H_

#include "helpers.h"
#include <stdint.h>

// The pattern itself: what value each step of the cycle shows, before the
// normalisation that stops short patterns collapsing.
//
// It lives in a header because two places need to agree on it exactly. The
// runtime asks for two steps per sample; tools/gen_stepped_table.c walks the whole
// cycle to measure its span and derive the normalisation. When the generator
// carried its own copy of the value path, the two could drift and the symptom
// would be a correction quietly aimed at a pattern the module no longer plays.
//
// The slot tables come in as a parameter rather than being included, so this
// header has no dependency on the generated one - which is what lets the
// generator include it while producing it.

// --------------------------------------------------------------------------
// What SHP and MOD reach.
//
// SHP moves three things here, so that a sweep is not a series of fresh random
// draws from one fixed distribution - the reason it used to read as "nothing is
// happening" however many distinct patterns it technically contained. Which
// values the slots hold (as it always did), how those values are distributed
// (ST_BEND), and how melodic the contour is (ST_CONTOUR). A fourth lives
// outside this header: st_bias_map() leans the finished value, which is the one
// thing the levers below cannot do, since ST_BEND is odd and so can only work
// on the middle symmetrically.
//
// None of them is level. Both knobs steer character and leave how loud the
// result is to AMP. SHP used to carry a lever that did move level, ST_SPAN,
// which ducked the peak-to-peak by up to 0.7; it read as the shape flattening
// as the knob turned rather than as character, and being a pure gain it moved
// none of the character measurements. The correction in stepped_norm.h
// is what holds level steady now.
//
// MOD moves six: density as before, which steps tie (ST_SPIN), where the beat
// sits (ST_SWING), whether the cycle repeats a sub-phrase (ST_MOTIF), a layer
// of the shaper (ST_BEND_MOD), and a share of the orbit itself (ST_MORPH_MOD).
// That last one is what lets MOD reach patterns SHP never lands on; without it
// MOD could only rearrange and recolour whichever pattern SHP had picked.
//
// Each lever has a depth and a *rate*: how many excursions it makes across its
// knob. One each spends the whole sweep on a single traversal of character
// space. Several, at rates that do not divide one another, means a given
// contour depth comes back a few times over the sweep but paired with a
// different shaper and a different density each time - which is where the extra
// range comes from without the knob getting twitchier. Cyclicity survives
// because the rates are integers: every lever is home again at the end.
//
// The drives are worked out once per sample in st_morph() while the expensive
// work is per-slot, so layering is close to free. What is actually being spent
// is how far a small turn may move the pattern.
//
// Depths and rates are set by measurement, in tools/stepped_explore/. The binding
// constraints are that small-turn limit, which the suite asserts at 0.35 from
// eight steps up, and keeping the minimum span across the whole parameter
// space above 0.5.
// --------------------------------------------------------------------------
#define ST_CONTOUR 1.0f // depth of the melodic-contour blend
#define ST_CONTOUR_RATE 3
#define ST_TAPS 4    // slots the contour averages over
#define ST_BEND 0.8f // depth of the value-distribution shaper, on SHP
#define ST_BEND_RATE 1
#define ST_BEND_MOD 0.45f // and the layer of it MOD drives, at its own rate
#define ST_BEND_MOD_RATE 3
// The two bend layers together may not reach 1, where the shaper would stop
// being monotone.
#define ST_BEND_LIMIT 0.95f
#define ST_MORPH_MOD 0.4f  // MOD's share of the orbit the slot values ride
#define ST_TIE_WIDTH 0.25f // gate units a tie crossfades over
#define ST_SPIN 0.5f       // turns of gate rotation across the MOD sweep
#define ST_SWING 0.4f      // step-width skew at half MOD
#define ST_SWING_RATE 1
#define ST_MOTIF 0.7f // depth of the sub-cycle fold at the MOD ends
#define ST_MOTIF_RATE 1
#define ST_MOTIF_MIN_LENGTH 16 // below this, a quarter cycle is not a phrase
#define ST_MOTIF_DIVISOR 4

// Chance that a step repeats the previous value instead of taking a new one.
// This is what MOD moves; the value here is what MOD 0 asks for, and the top of
// the range is ST_HOLD_MAX, which the generated table defines because it is
// that table's own axis.
#define ST_HOLD_NEUTRAL 0.30f

// Repeats are faded out on short patterns, where a single repeat would flatten
// too large a fraction of the cycle.
#define ST_HOLD_FADE_IN_STEPS 6.0f

typedef struct
{
  const float* base;   // where each slot's value starts on its orbit
  const uint8_t* rate; // turns it makes across one sweep of SHP
  const float* gate;   // its threshold for taking a new value
  const float* gate2;  // how fast MOD rotates that threshold
} StSlots;

// fractf() for values already known to be non-negative - avoids floorf(), which
// is a call on this target and would otherwise dominate the cost here.
static inline float st_fract_pos(float x) { return x - (float) (int) x; }

// Triangle, period 1, range [-1, 1]. Chosen over a sine because it gives a
// uniform spread of values: the random levels cover the output range evenly
// rather than bunching at the extremes.
static inline float st_tri(float x)
{
  float f = st_fract_pos(x);
  return 4.0f * fabsf(f - 0.5f) - 1.0f;
}

// Odd S-curve on [-1,1]. Stands in for a sine wherever a lever needs driving
// out and smoothly back: same value and same zero slope at both peaks, without
// the call. Nothing here needs a sine's spectrum, only its shape.
static inline float st_scurve(float t) { return t * (1.5f - 0.5f * t * t); }

// A train of `rate` humps across the knob: 0 at every cycle end, 1 in every
// middle, smooth throughout. For levers that only run one way.
static inline float st_bump(float x, int rate)
{
  float f = st_fract_pos(x * (float) rate);
  return smoothstep(1.0f - fabsf(2.0f * f - 1.0f));
}

// The same train, signed: `rate` excursions out and back, zero at every
// boundary. For levers that run both ways.
static inline float st_swingf(float x, int rate) { return st_scurve(-st_tri(x * (float) rate + 0.25f)); }

// Reshapes the spread of values without moving -1, 0 or +1.
//
//   bend > 0  compresses the middle: small intervals, a calm melodic line
//   bend < 0  expands it: values pushed to the extremes, gate-like
//
// A blend rather than a power law, for two reasons. Its slope at zero is
// 1 - |bend|, so it can never crush the whole pattern to nothing however hard
// it is driven - a power law could, and had to be faded out on short patterns
// to stop it, which made slot 0 length-dependent and broke seamless length
// switching. And powf() is not affordable here, while a multiply and a sqrtf()
// (one instruction on this core) are.
static inline float st_shape_blend(float v, float bend)
{
  if (bend >= 0.0f)
  {
    return (1.0f - bend) * v + bend * v * fabsf(v);
  }
  float b = -bend;
  return (1.0f + bend) * v + b * copysignf(sqrtf(fabsf(v)), v);
}

// The contour blend needs slots to work on: averaging four of them out of a
// three-step cycle leaves three equal values and nothing to hear.
static inline float st_dof_fade(int length) { return fclamp(((float) length - 4.0f) * (1.0f / 8.0f), 0.0f, 1.0f); }

static inline float st_tie_fade(int length) { return fclamp(((float) length - 2.0f) / ST_HOLD_FADE_IN_STEPS, 0.0f, 1.0f); }

// How often a step ties to the previous one instead of taking a new value: the
// pattern's density.
//
// Bipolar around the neutral 0.30, so MOD 0 is the behaviour the pattern was
// designed around: fully left every step is new and the cycle is at its
// busiest, fully right most steps are ties and the pattern reads as a handful
// of long notes.
static inline float st_hold_probability(float mod, int length, float hold_max)
{
  mod     = fclamp(mod, -1.0f, 1.0f);
  float p = (mod <= 0.0f) ? ST_HOLD_NEUTRAL * (1.0f + mod) : ST_HOLD_NEUTRAL + mod * (hold_max - ST_HOLD_NEUTRAL);
  return p * st_tie_fade(length);
}

// Everything about the pattern that depends only on the two knobs, worked out
// once per sample instead of once per slot.
typedef struct
{
  float morph;      // SHP as [0,1) - also the normalisation table's axis
  float orbit;      // where the slot values sit on their orbits: SHP's morph
                    // plus MOD's share of it. That share is what lets MOD reach
                    // patterns SHP never lands on, rather than only recolouring
                    // whichever one SHP picked
  float contour;    // depth of the contour blend here
  float bend;       // shaper drive here, both knobs' layers summed
  float motif;      // depth of the sub-cycle fold here
  int motif_period; // length of the sub-cycle, 0 when folding is off
  float threshold;  // density: gates at or above this keep their own value
  float spin;       // how far MOD has rotated the gates
  float tie_fade;   // ties faded out on short patterns
} StMorph;

// Built from the orbit rather than from SHP directly, because that is the
// parameterisation the normalisation table is indexed by.
//
// Every SHP-driven lever is a function of the orbit, and the orbit enters the
// slot values only through tri() at integer rates - so the whole value path is
// periodic in it with period 1. That is what lets a 128-bin axis cover it. Were
// the levers driven by SHP's own morph while the values rode the orbit, the
// pattern would depend on both, and MOD moving the orbit would make the table's
// 8-bin MOD axis far too coarse to interpolate across: measured, that put 1.3%
// of the parameter space below the flat-output floor.
static inline StMorph st_morph_at(float orbit, float mod, int length, float hold_max)
{
  StMorph m;
  float d = 0.5f * (fclamp(mod, -1.0f, 1.0f) + 1.0f);

  m.orbit = orbit;
  m.morph = st_fract_pos(orbit < 0.0f ? orbit + 1.0f : orbit);

  m.contour = ST_CONTOUR * st_dof_fade(length) * st_bump(m.morph, ST_CONTOUR_RATE);
  m.bend = fclamp(ST_BEND * st_swingf(m.morph, ST_BEND_RATE) + ST_BEND_MOD * st_swingf(d, ST_BEND_MOD_RATE), -ST_BEND_LIMIT, ST_BEND_LIMIT);

  m.threshold    = st_hold_probability(mod, length, hold_max);
  m.spin         = d * ST_SPIN;
  m.tie_fade     = st_tie_fade(length);
  m.motif        = 0.0f;
  m.motif_period = 0;
  if (length >= ST_MOTIF_MIN_LENGTH)
  {
    m.motif        = ST_MOTIF * (1.0f - st_bump(d, ST_MOTIF_RATE));
    m.motif_period = length / ST_MOTIF_DIVISOR;
  }
  return m;
}

// MOD's share of the orbit is what lets it reach patterns SHP never lands on,
// rather than only rearranging and recolouring whichever one SHP picked. SHP
// still wraps: a full turn of it moves the orbit by exactly 1, and everything
// downstream is periodic in the orbit with that period.
static inline StMorph st_morph(float shape, float mod, int length, float hold_max)
{
  float d     = 0.5f * (fclamp(mod, -1.0f, 1.0f) + 1.0f);
  float morph = 0.5f * (fclamp(shape, -1.0f, 1.0f) + 1.0f);
  return st_morph_at(morph + ST_MORPH_MOD * d, mod, length, hold_max);
}

// The raw value one slot carries: its orbit position, blended toward a running
// average of the slots before it, then reshaped.
//
// The average clamps at slot 0 rather than wrapping, which keeps slot 0 equal
// to its own orbit value at every length and every blend depth. That is what
// seamless pattern-length switching rests on.
// Consecutive slots ask for overlapping runs of orbit values, so a sample
// evaluates the same triangle up to ST_TAPS times. Caching them across the run
// was tried and measured *slower* - the compare and branch to read the cache
// costs more than st_tri()'s handful of arithmetic, and the indirection stops
// the compiler keeping the run in registers. Left alone deliberately.
static inline float st_slot_value(int slot, const StMorph* m, const StSlots* s)
{
  float raw = st_tri(s->base[slot] + m->orbit * (float) s->rate[slot]);

  float v = raw;
  if (m->contour > 0.0f)
  {
    float acc = raw; // tap 0 is the slot itself, already computed
    for (int t = 1; t < ST_TAPS; t++)
    {
      int j = slot - t;
      if (j < 0)
      {
        j = 0;
      }
      acc += st_tri(s->base[j] + m->orbit * (float) s->rate[j]);
    }
    v = lerp(raw, acc * (1.0f / (float) ST_TAPS), m->contour);
  }

  return st_shape_blend(v, m->bend);
}

// The value a slot offers the pattern: its own, folded toward the matching slot
// of a shorter sub-cycle so the pattern repeats a phrase.
//
// Folded here, before the ties are applied, rather than after: it costs one
// pass instead of three, and the repeat comes out varied rather than a literal
// copy, because the tie pattern still runs the full length of the cycle.
static inline float st_slot_offer(int slot, const StMorph* m, const StSlots* s)
{
  float v = st_slot_value(slot, m, s);
  if (m->motif > 0.0f && slot >= m->motif_period)
  {
    v = lerp(v, st_slot_value(slot % m->motif_period, m, s), m->motif);
  }
  return v;
}

// How much of its own value a slot takes, against the previous step's.
//
// A crossfade, not a decision. As a switch - which is what this used to be -
// the step flipped to the previous value the moment the density threshold
// crossed its gate, jumping up to 1.45 of a 2.0 range within 1% of knob travel,
// while doing nothing at all in between. Crossfaded, MOD moves the pattern
// everywhere on its travel and a half-tied step is a smaller interval rather
// than a glitch.
//
// Every ST_JUMP_GRID-th slot is pinned fully open, which bounds the look-back
// to O(1), keeps slot 0 taking its own value so the loop point always closes on
// it, and stops ties flattening a pattern.
static inline float st_tie_weight(int slot, const StMorph* m, const StSlots* s, int jump_grid)
{
  if (slot % jump_grid == 0)
  {
    return 1.0f;
  }
  // MOD rotates each slot's gate at its own rate, so which steps tie keeps
  // re-arranging instead of the tied set simply growing. Without it, ties nest:
  // once a step tied it stayed tied, so MOD could only ever thin one pattern
  // rather than reach a different one.
  float g = 0.5f * (1.0f + st_tri(s->gate[slot] + m->spin * (1.0f + s->gate2[slot])));
  float w = smoothstep(fclamp((g - m->threshold) / ST_TIE_WIDTH + 0.5f, 0.0f, 1.0f));
  return 1.0f - (1.0f - w) * m->tie_fade;
}

#define ST_MAX_JUMP_GRID 8

// Where a step's run of ties begins: the nearest earlier slot that takes its
// own value outright, or the pinned slot, whichever comes first.
//
// Walking back only as far as the ties actually reach matters because at low
// density most steps end the chain immediately. Weights are cheap - a triangle
// and a smoothstep - while the slot values they gate are the expensive part, so
// the start is worth finding before evaluating any of them.
static inline int st_tie_run(int step, const StMorph* m, const StSlots* s, int jump_grid, float* w)
{
  int first = step - (step % jump_grid);
  int start = step;

  while (start > first)
  {
    float wi = st_tie_weight(start, m, s, jump_grid);
    if (wi >= 1.0f)
    {
      break;
    }
    w[start - first] = wi;
    start--;
  }
  return start;
}

// The value shown at a step: walk forward from where its run of ties begins,
// each slot crossfading from what the previous one showed toward its own value.
// At most jump_grid slots, whatever the pattern length.
static inline float st_step_value(int step, const StMorph* m, const StSlots* s, int jump_grid)
{
  int first = step - (step % jump_grid);
  float w[ST_MAX_JUMP_GRID];
  int start = st_tie_run(step, m, s, jump_grid, w);

  float v = st_slot_offer(start, m, s);
  for (int i = start + 1; i <= step; i++)
  {
    v = lerp(v, st_slot_offer(i, m, s), w[i - first]);
  }
  return v;
}

// The two values one sample needs: what the current step shows and what the
// next one does, in a single walk.
//
// The next step is nearly always the one after this in the same run of ties, so
// it is the current value crossfaded one place further. Only when it opens a
// new run - or wraps to slot 0 - does it need a value of its own, and that is
// one call because a pinned slot has no look-back.
static inline void st_step_pair(int step, int next, const StMorph* m, const StSlots* s, int jump_grid, float* from, float* to)
{
  *from = st_step_value(step, m, s, jump_grid);

  if (next == step + 1 && (next % jump_grid) != 0)
  {
    *to = lerp(*from, st_slot_offer(next, m, s), st_tie_weight(next, m, s, jump_grid));
  }
  else
  {
    *to = st_step_value(next, m, s, jump_grid); // a pinned slot: no look-back
  }
}

// How wide each step is, as a fraction of the cycle.
//
// MOD skews alternate steps long and short. The widths are normalised to sum to
// one, so the cycle still closes wherever the knob is, and the skew is zero at
// MOD centre and at both extremes - straight time in the middle, leaning one
// way then the other on either side.
static inline float st_swing_amount(float mod)
{
  float d = 0.5f * (fclamp(mod, -1.0f, 1.0f) + 1.0f);
  return -ST_SWING * st_swingf(d, ST_SWING_RATE);
}

// Which step of the cycle a phase lands in, and how far through it.
//
// O(1) despite the uneven widths: a long step and the short one after it always
// span exactly two average steps between them, so the pair is found by
// division and only the split inside it has to be worked out. An odd length
// leaves one unpaired step at the end.
static inline int st_step_at(float phase, int length, float swing, float* within)
{
  int pairs   = length / 2;
  float total = (float) length + ((length & 1) ? swing : 0.0f);
  float lo    = 1.0f + swing; // even steps
  float hi    = 1.0f - swing; // odd steps
  float u     = fclamp(phase, 0.0f, 1.0f) * total;

  if ((length & 1) && u >= 2.0f * (float) pairs)
  {
    // the odd length's unpaired last step, which is a long one. Only odd
    // lengths have one: at an even length the pairs cover the whole cycle, and
    // taking this branch there would land phase 1.0 at the *start* of the last
    // step instead of its end - a jump at the loop point, which is the one
    // place the waveform must be seamless.
    *within = fclamp((u - 2.0f * (float) pairs) / lo, 0.0f, 1.0f);
    return length - 1;
  }

  int pair = (int) (u * 0.5f);
  if (pair >= pairs)
  {
    pair = pairs - 1;
  }
  float rem = u - 2.0f * (float) pair;

  if (rem < lo)
  {
    *within = rem / lo;
    return 2 * pair;
  }
  *within = fclamp((rem - lo) / hi, 0.0f, 1.0f);
  return 2 * pair + 1;
}

#endif /* INC_LIB_STEPPED_RANDOM_PATTERN_H_ */
