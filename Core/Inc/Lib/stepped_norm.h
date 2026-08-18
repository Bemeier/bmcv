#ifndef INC_LIB_STEPPED_RANDOM_NORM_H_
#define INC_LIB_STEPPED_RANDOM_NORM_H_

#include "stepped_pattern.h"
#include <math.h>
#include <stdint.h>

// The correction that holds the level steady: what `out = value * gain + offset`
// is built from, as a function rather than as a table.
//
// tools/gen_stepped_table.c calls this to bake the table the firmware reads. It is a
// header, and takes the slot tables as a parameter, for the same reason
// stepped_pattern.h does: the generator has to be able to include it
// while producing the header that would otherwise define its inputs.
//
// It is split in two because its halves have opposite shapes, and that is what
// decides which of them can stay in flash and which cannot:
//
//   st_norm_centre()  length-INdependent, expensive - an average across every
//                     pattern length of what would centre each. A channel knows
//                     only its own length, so it cannot work this out. 8 KB of
//                     table covers the whole (MOD, orbit) plane.
//   st_gain_for()     length-dependent, cheap - one pattern's own extremes.
//                     Scanning them costs `length` slot evaluations, which is
//                     affordable amortised over a cycle.
//
// See docs/plans/stepped-modes.md.

// --------------------------------------------------------------------------
// What the correction aims for.
//
// Two jobs: hold the peak-to-peak near ST_NORM_TARGET, and put the pattern's
// own centre near zero. Both used to be one-sided - lift a collapsed pattern,
// never shrink a wide one, and leave the centre wherever the pattern happened
// to sit. Measured, that gave a peak-to-peak varying 2.3x between settings of
// the same knob and a centre that walked 0.76 of the 2.0 range along one sweep
// of SHP. Both read as the shape flattening and shifting as the knob turns,
// which is what AMP and OFFSET are for; SHP and MOD steer character.
//
// Neither job is absolute. ST_NORM_EXP leaves the natural loud/quiet ordering
// audible while compressing its range, and the centring is one constant shared
// by all the lengths rather than a fit to each.
// --------------------------------------------------------------------------
#define ST_NORM_TARGET 1.5f
#define ST_NORM_MAX_GAIN 40.0f
// A clamp rather than a setting: at ST_NORM_EXP 0.7 the smallest gain the
// normalisation ever asks for is (1.5/2.0)^0.7 = 0.82, so this only catches a
// re-tuning that went somewhere absurd.
#define ST_NORM_MIN_GAIN 0.4f

// How completely the peak-to-peak is normalised. 1 pins every setting to
// ST_NORM_TARGET; 0 leaves them all alone. At 0.7 a pattern whose natural span
// is 0.7 comes out at 1.20 and one at 2.0 comes out at 1.64 - the ordering
// survives, the 2.9x range does not.
#define ST_NORM_EXP 0.7f

// The span the correction guarantees even where it is otherwise shrinking, so
// the two-sided normalisation cannot make a dead setting.
#define ST_NORM_FLOOR 0.9f

// How much of the *outermost* value the span is measured from.
//
// 1.0 is peak-to-peak off the single highest and single lowest slot, which is
// what this always did - and it is why the shape could not produce an
// occasional tall spike. A pattern that sits low with one peak has the same
// peak-to-peak as one spread evenly, so it earns the same gain, so its spike
// comes out no higher than the other's ordinary maximum and its bulk far
// below. Crest factor is the one thing peak-to-peak normalisation flattens.
//
// Below 1 the span is measured between the *second* highest and second lowest
// instead, blended. A spiky pattern then reads as narrow, earns a larger gain,
// and its bulk comes up - while the spike is free to run to the rail, because
// the rail limit below is still taken from the true extremes and pins the true
// maximum at +/-1 rather than clipping it.
#define ST_NORM_ROBUST 0.5f

// Everything the correction needs to know that is not a knob position: which
// pattern the slots make, which lengths exist, and the two constants the
// generated table owns because they are its own axes.
typedef struct
{
  const StSlots* slots;
  const uint8_t* lengths; // the curated set, for the cross-length centring
  int length_count;
  int jump_grid;
  float hold_max;
} StNormCtx;

typedef struct
{
  // Before the reshapings: centre the pattern and scale it toward
  // ST_NORM_TARGET. out = centre + (v - anchor) * gain.
  float gain;
  float offset;

  // After them: hold the level the two above just set, which the reshapings
  // would otherwise give back. out = post_ref + (v - post_ref) * post_gain.
  //
  // A second affine rather than a correction folded into the first, and that is
  // the whole point. Changing the gain *before* the maps moves values against
  // the terracing's cell boundaries, so it changes which plateau each one lands
  // on - the character, not the level. Measured, that made the spread worse
  // rather than better (see docs/stepped.md). Applied after, it scales the
  // finished waveform uniformly: the plateaus stay plateaus, the loop still
  // closes, and only how loud it is moves.
  //
  // Anchored at post_ref, which is the reshaped value at phase 0. Every length
  // shares it - it comes from the length-independent centring constant - so
  // phase 0 lands in the same place whatever the gain, and switching pattern
  // length on the wrap stays seamless.
  float post_gain;
  float post_ref;
} StNorm;

// One pattern, measured: its extremes, the value it starts on, and - only where
// something asks for it - its DC.
typedef struct
{
  float lo, hi, centre, anchor;

  // Second-outermost, for ST_NORM_ROBUST. Equal to lo/hi on a pattern of fewer
  // than three distinct values, which is what makes the blend degrade to
  // peak-to-peak rather than to nonsense.
  float lo2, hi2;

  // How much of ST_NORM_ROBUST this pattern's length can actually carry - 1 is
  // peak-to-peak. Set where the extent is built, because that is what knows the
  // length; st_gain_toward only sees the extent.
  float robust;
} StExtent;

// How much robustness a pattern of this length can carry.
//
// A statistic that needs a distribution needs values to compute it from. At
// three steps hi2 and lo2 are *both* the middle value, so the robust span is
// zero, the gain runs to the rail, and a 1% turn of SHP moved the output 1.66
// of a 2.0 range against a 1.10 limit - measured. st_dof_fade is the same fade
// the contour blend uses, and for the same reason: it is written above as
// "averaging four slots out of a three-step cycle leaves three equal values and
// nothing to hear".
static inline float st_robust_for(int length) { return lerp(1.0f, ST_NORM_ROBUST, st_dof_fade(length)); }

// A pair of order statistics, kept as the pattern is walked. O(1) per slot.
static inline void st_extent_push(StExtent* e, float v)
{
  if (v > e->hi)
  {
    e->hi2 = e->hi;
    e->hi  = v;
  }
  else if (v > e->hi2)
  {
    e->hi2 = v;
  }

  if (v < e->lo)
  {
    e->lo2 = e->lo;
    e->lo  = v;
  }
  else if (v < e->lo2)
  {
    e->lo2 = v;
  }
}

// The extremes and the anchor, which is all the gain needs.
//
// Deliberately without the DC: that is the centring constant's business, it is
// baked once by the generator, and summing it here put a multiply and an add
// per slot on the path a channel walks.
static inline StExtent st_extent_of(const StNormCtx* c, int length, float mod, float orbit)
{
  StMorph m  = st_morph_at(orbit, mod, length, c->hold_max);
  StExtent e = {1e9f, -1e9f, 0.0f, 0.0f, 1e9f, -1e9f, st_robust_for(length)};

  for (int i = 0; i < length; i++)
  {
    st_extent_push(&e, st_step_value(i, &m, c->slots, c->jump_grid, NULL));
  }

  // A pattern with fewer than three distinct values never fills the seconds.
  if (e.hi2 < e.lo)
    e.hi2 = e.hi;
  if (e.lo2 > e.hi)
    e.lo2 = e.lo;

  e.anchor = st_step_value(0, &m, c->slots, c->jump_grid, NULL);
  return e;
}

// The same, plus the DC, weighted by how wide each step is - MOD skews alternate
// steps long and short, and a plain mean of the step values would not be the
// level the ear settles on. Only st_norm_centre() needs this, and only offline.
static inline StExtent st_extent_with_dc(const StNormCtx* c, int length, float mod, float orbit)
{
  StMorph m   = st_morph_at(orbit, mod, length, c->hold_max);
  StExtent e  = st_extent_of(c, length, mod, orbit);
  float swing = st_swing_amount(mod);
  float dc = 0.0f, wsum = 0.0f;

  for (int i = 0; i < length; i++)
  {
    float w = (i & 1) ? 1.0f - swing : 1.0f + swing;
    dc += st_step_value(i, &m, c->slots, c->jump_grid, NULL) * w;
    wsum += w;
  }

  e.centre = dc / wsum;
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
static inline float st_gain_toward(float target, float expo, const StExtent* e, float centre)
{
  // The span the gain is set from - see ST_NORM_ROBUST. The rails below stay on
  // the true extremes, so raising the gain lifts a spike toward +/-1 and stops
  // it there rather than clipping it.
  float hi_eff = lerp(e->hi2, e->hi, e->robust);
  float lo_eff = lerp(e->lo2, e->lo, e->robust);
  float span   = hi_eff - lo_eff;

  // powf() only where the exponent needs it. The floor below asks for one, and
  // pow(x, 1) on this target is a transcendental call standing in for a divide -
  // once per completed measurement per channel, which is not nothing.
  float want = (expo == 1.0f) ? target / span : powf(target / span, expo);
  float g    = (span < 1e-6f) ? ST_NORM_MAX_GAIN : fclamp(want, ST_NORM_MIN_GAIN, ST_NORM_MAX_GAIN);

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

static inline float st_gain_for(const StExtent* e, float centre) { return st_gain_toward(ST_NORM_TARGET, ST_NORM_EXP, e, centre); }

// What a point needs to stay clear of the flat floor, whatever the
// normalisation would otherwise ask for.
static inline float st_gain_floor(const StExtent* e, float centre) { return st_gain_toward(ST_NORM_FLOOR, 1.0f, e, centre); }

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
static inline float st_norm_centre(const StNormCtx* c, float mod, float orbit)
{
  float centre = 0.0f;

  for (int it = 0; it < 3; it++)
  {
    float sum = 0.0f;
    for (int li = 0; li < c->length_count; li++)
    {
      StExtent e = st_extent_with_dc(c, c->lengths[li], mod, orbit);
      sum += (e.centre - e.anchor) * st_gain_for(&e, centre);
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
// The pre-stage. The post-stage is filled in by whoever knows the drive - see
// st_norm_levelled() in stepped.c - and is the identity until then.
static inline StNorm st_norm_affine(float centre, float anchor, float gain)
{
  StNorm n = {gain, centre - anchor * gain, 1.0f, 0.0f};
  return n;
}

// The whole correction for one pattern, given the centring constant the table
// carries: scan the pattern, take the gain its extremes ask for, build the
// affine about slot 0.
//
// This is what a channel works out for itself. The scan is `length` slot
// evaluations - affordable amortised over a cycle, not per sample; see
// docs/plans/stepped-modes.md for the incremental version and when it is
// allowed to take effect.
//
// Everything the binned table has to approximate, this gets exactly. Nothing is
// interpolated, so out(0) lands on `centre` at every length instead of nearly
// so, and the gain needs no neighbourhood padding to survive being blended with
// its neighbours - which was a bias upward on every setting.
// `out_e`, when given, hands back the extent this was derived from. A caller
// that needs to know what the correction produces *after* the reshapings has to
// put those two endpoints through them itself, and the reshapings live a header
// above this one. NULL when it does not care.
static inline StNorm st_norm_at(const StNormCtx* c, int length, float mod, float orbit, float centre, StExtent* out_e)
{
  StExtent e = st_extent_of(c, length, mod, orbit);
  if (out_e != NULL)
  {
    *out_e = e;
  }

  // The floor is applied here, at the point itself. In the table it only ever
  // arrived through the neighbourhood loop - which was written to survive
  // interpolation and turned out to be carrying the floor as a side effect.
  // Computing the correction without it dropped the worst span anywhere from
  // 0.584 to 0.470, through the 0.5 the suite holds.
  float g  = st_gain_for(&e, centre);
  float gf = st_gain_floor(&e, centre);
  return st_norm_affine(centre, e.anchor, (gf > g) ? gf : g);
}

#endif /* INC_LIB_STEPPED_RANDOM_NORM_H_ */
