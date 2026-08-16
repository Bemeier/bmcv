#ifndef INC_LIB_STEPPED_RANDOM_NORM_H_
#define INC_LIB_STEPPED_RANDOM_NORM_H_

#include "stepped_random_pattern.h"
#include <math.h>
#include <stdint.h>

// The correction that holds the level steady: what `out = value * gain + offset`
// is built from, as a function rather than as a table.
//
// tools/gen_sr_table.c calls this to bake the table the firmware reads. It is a
// header, and takes the slot tables as a parameter, for the same reason
// stepped_random_pattern.h does: the generator has to be able to include it
// while producing the header that would otherwise define its inputs.
//
// It is split in two because its halves have opposite shapes, and that is what
// decides which of them can stay in flash and which cannot:
//
//   sr_norm_centre()  length-INdependent, expensive - an average across every
//                     pattern length of what would centre each. A channel knows
//                     only its own length, so it cannot work this out. 8 KB of
//                     table covers the whole (MOD, orbit) plane.
//   sr_gain_for()     length-dependent, cheap - one pattern's own extremes.
//                     Scanning them costs `length` slot evaluations, which is
//                     affordable amortised over a cycle.
//
// See docs/plans/stepped-random-modes.md.

// --------------------------------------------------------------------------
// What the correction aims for.
//
// Two jobs: hold the peak-to-peak near SR_NORM_TARGET, and put the pattern's
// own centre near zero. Both used to be one-sided - lift a collapsed pattern,
// never shrink a wide one, and leave the centre wherever the pattern happened
// to sit. Measured, that gave a peak-to-peak varying 2.3x between settings of
// the same knob and a centre that walked 0.76 of the 2.0 range along one sweep
// of SHP. Both read as the shape flattening and shifting as the knob turns,
// which is what AMP and OFFSET are for; SHP and MOD steer character.
//
// Neither job is absolute. SR_NORM_EXP leaves the natural loud/quiet ordering
// audible while compressing its range, and the centring is one constant shared
// by all the lengths rather than a fit to each.
// --------------------------------------------------------------------------
#define SR_NORM_TARGET 1.5f
#define SR_NORM_MAX_GAIN 40.0f
// A clamp rather than a setting: at SR_NORM_EXP 0.7 the smallest gain the
// normalisation ever asks for is (1.5/2.0)^0.7 = 0.82, so this only catches a
// re-tuning that went somewhere absurd.
#define SR_NORM_MIN_GAIN 0.4f

// How completely the peak-to-peak is normalised. 1 pins every setting to
// SR_NORM_TARGET; 0 leaves them all alone. At 0.7 a pattern whose natural span
// is 0.7 comes out at 1.20 and one at 2.0 comes out at 1.64 - the ordering
// survives, the 2.9x range does not.
#define SR_NORM_EXP 0.7f

// The span the correction guarantees even where it is otherwise shrinking, so
// the two-sided normalisation cannot make a dead setting.
#define SR_NORM_FLOOR 0.9f

// Everything the correction needs to know that is not a knob position: which
// pattern the slots make, which lengths exist, and the two constants the
// generated table owns because they are its own axes.
typedef struct
{
  const SrSlots* slots;
  const uint8_t* lengths; // the curated set, for the cross-length centring
  int length_count;
  int jump_grid;
  float hold_max;
} SrNormCtx;

typedef struct
{
  float gain;
  float offset;
} SrNorm;

// One pattern, measured: its extremes, its DC, and the value it starts on.
typedef struct
{
  float lo, hi, centre, anchor;
} SrExtent;

// The DC is weighted by how wide each step is, since MOD skews alternate steps
// long and short and a plain mean of the step values would not be the level the
// ear settles on.
static inline SrExtent sr_extent_of(const SrNormCtx* c, int length, float mod, float orbit)
{
  SrMorph m   = sr_morph_at(orbit, mod, length, c->hold_max);
  SrExtent e  = {1e9f, -1e9f, 0.0f, 0.0f};
  float swing = sr_swing_amount(mod);
  float dc = 0.0f, wsum = 0.0f;

  for (int i = 0; i < length; i++)
  {
    float v = sr_step_value(i, &m, c->slots, c->jump_grid);
    float w = (i & 1) ? 1.0f - swing : 1.0f + swing;
    if (v < e.lo)
      e.lo = v;
    if (v > e.hi)
      e.hi = v;
    dc += v * w;
    wsum += w;
  }

  e.centre = dc / wsum;
  e.anchor = sr_step_value(0, &m, c->slots, c->jump_grid);
  return e;
}

// The gain that carries a pattern's peak-to-peak toward `target`, and what the
// rails leave of it once the correction's constant `centre` is added.
//
// The pattern is scaled about slot 0, so the caps are asymmetric: how far the
// top of the pattern is above slot 0 decides how much expansion the top rail
// allows, and the bottom likewise. Where slot 0 sits relative to the pattern is
// therefore what the constant is really buying - it used to be pulled toward
// zero purely to open that headroom up, and centring the pattern does the same
// job as a side effect of doing something musically useful.
static inline float sr_gain_toward(float target, float expo, const SrExtent* e, float centre)
{
  float span = e->hi - e->lo;
  float g    = (span < 1e-6f) ? SR_NORM_MAX_GAIN : fclamp(powf(target / span, expo), SR_NORM_MIN_GAIN, SR_NORM_MAX_GAIN);

  if (e->hi > e->anchor)
  {
    float limit = (1.0f - centre) / (e->hi - e->anchor);
    if (limit < g)
      g = limit;
  }
  if (e->lo < e->anchor)
  {
    float limit = (1.0f + centre) / (e->anchor - e->lo);
    if (limit < g)
      g = limit;
  }
  return (g < 0.05f) ? 0.05f : g;
}

static inline float sr_gain_for(const SrExtent* e, float centre) { return sr_gain_toward(SR_NORM_TARGET, SR_NORM_EXP, e, centre); }

// What a point needs to stay clear of the flat floor, whatever the
// normalisation would otherwise ask for.
static inline float sr_gain_floor(const SrExtent* e, float centre) { return sr_gain_toward(SR_NORM_FLOOR, 1.0f, e, centre); }

// The constant that puts the pattern's centre as near zero as one number can.
//
// One number is all there is. The corrected value at the cycle boundary is
// slot 0's, and every length has to agree on it for the engine to be able to
// switch pattern length on the wrap - so a constant fitted per length would put
// a step in the signal exactly where there must not be one. Hence an average
// across the lengths rather than a fit to each: the minimax was tried and
// measured worse (0.54 of residual DC swing against 0.41), because two short
// patterns at the extremes drag it away from where the other nine sit.
//
// Iterated, because the gain the rails allow depends on the constant and the
// constant depends on the gain.
static inline float sr_norm_centre(const SrNormCtx* c, float mod, float orbit)
{
  float centre = 0.0f;

  for (int it = 0; it < 3; it++)
  {
    float sum = 0.0f;
    for (int li = 0; li < c->length_count; li++)
    {
      SrExtent e = sr_extent_of(c, c->lengths[li], mod, orbit);
      sum += (e.centre - e.anchor) * sr_gain_for(&e, centre);
    }
    centre = -sum / (float) c->length_count;
  }
  return fclamp(centre, -1.0f, 1.0f);
}

// out = centre + (v - anchor) * gain, as the affine the runtime applies.
//
// Both `centre` and `anchor` are the same at every length - the first by
// construction, the second because a slot's value never depends on how many
// slots the pattern has - so the value at the cycle boundary is too.
static inline SrNorm sr_norm_affine(float centre, float anchor, float gain)
{
  SrNorm n = {gain, centre - anchor * gain};
  return n;
}

#endif /* INC_LIB_STEPPED_RANDOM_NORM_H_ */
