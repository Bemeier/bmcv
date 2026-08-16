#include "stepped.h"
#include "helpers.h"
#include "stepped_norm.h"
#include "stepped_pattern.h"
#include "stepped_table.h"

// Rhythmic random LFO.
//
// A cycle is divided into `length` steps, each holding a random value. The
// step lattice is read circularly, so the waveform closes seamlessly at the
// loop point and the pattern stays locked to the beat under the PLL. That
// requires `length` to be a whole number of steps per cycle - hence the
// curated table in the generated header rather than a continuously variable
// step count.
//
// Within a step the curve sits at its value for `hold` of the step, then eases
// to the next. The easing still reaches exactly 1.0 at the end of the step, so
// consecutive steps join continuously and, because the ease has zero slope at
// both ends, the derivative is zero at every boundary including the loop
// point. The result is a slewed sample-and-hold: audibly rhythmic, but a
// smooth curve with no clicks however hard the steps are set.
//
// What each step *shows* lives in stepped_pattern.h, shared with the
// table generator. This file is the part that only the runtime needs: finding
// the step a phase lands in, easing between two of them, and applying the
// correction the generator worked out.

// st_lengths[] (steps per cycle) lives in the generated table so the
// normalisation data cannot drift out of step with it. It is curated rather
// than every integer, so each knob position is musically distinct; the odd
// lengths (3, 5, 7) drift against a 4/4 clock and give polyrhythms rather than
// a locked loop.

static const StSlots st_slots = {st_slot_base, st_slot_rate, st_slot_gate, st_slot_gate2};

// st_step_value() keeps one tie weight per slot of a run on the stack.
_Static_assert(ST_JUMP_GRID <= ST_MAX_JUMP_GRID, "a run of ties must fit the weight buffer");

int st_length_for_index(int length_idx) { return st_lengths[iclamp(length_idx, 0, ST_LENGTH_COUNT - 1)]; }

// Where the centring constant is read from: the one part of the correction a
// channel cannot work out for itself, because it is an average across every
// pattern length and a channel knows only its own. Interpolated on both bin
// axes; both endpoints of MOD sit exactly on a bin, so the top one is only ever
// reached exactly.
static float st_centre_at(float mod, float morph)
{
  float bin_pos  = morph * (float) ST_NORM_BINS;
  int bin        = (int) bin_pos;
  float bin_frac = bin_pos - (float) bin;
  bin            = bin % ST_NORM_BINS;
  int bin_next   = (bin + 1) % ST_NORM_BINS;

  float mod_pos  = (fclamp(mod, -1.0f, 1.0f) + 1.0f) * 0.5f * (float) (ST_MOD_BINS - 1);
  int mod_bin    = (int) mod_pos;
  float mod_frac = mod_pos - (float) mod_bin;
  if (mod_bin >= ST_MOD_BINS - 1)
  {
    mod_bin  = ST_MOD_BINS - 2;
    mod_frac = 1.0f;
  }

  return lerp(lerp(st_centre_table[mod_bin][bin], st_centre_table[mod_bin][bin_next], bin_frac),
              lerp(st_centre_table[mod_bin + 1][bin], st_centre_table[mod_bin + 1][bin_next], bin_frac), mod_frac);
}

static const StNormCtx st_ctx = {&st_slots, st_lengths, ST_LENGTH_COUNT, ST_JUMP_GRID, ST_HOLD_MAX};

StNorm st_norm_exact(float shape, float mod, int length_idx)
{
  length_idx = iclamp(length_idx, 0, ST_LENGTH_COUNT - 1);
  StMorph m  = st_morph(shape, mod, st_lengths[length_idx], ST_HOLD_MAX);
  return st_norm_at(&st_ctx, st_lengths[length_idx], mod, m.orbit, st_centre_at(mod, m.morph));
}

// How long the correction takes to follow a pattern that changed under it.
//
// A measurement takes `length` ticks to complete, which is 16ms at the longest
// pattern and the engine's usual rate - so a comparable smoothing reads as the
// level settling rather than as two settings being crossfaded, and a pass that
// straddled a length switch cannot step the output.
#define ST_NORM_SMOOTH_S 0.02f

// Near enough that slewing further cannot be heard, and the point at which a
// standing channel is allowed to stop working.
#define ST_NORM_SETTLED 1e-6f

void st_norm_scan(StScan* s, float shape, float mod, int length_idx, float dt_s, int may_measure)
{
  length_idx = (int8_t) iclamp(length_idx, 0, ST_LENGTH_COUNT - 1);
  int length = st_lengths[length_idx];

  if (shape != s->shape_seen || mod != s->mod_seen)
  {
    // The pass in progress is now measuring two different patterns, so its
    // result cannot be trusted to be the last word on either.
    s->moved      = 1;
    s->shape_seen = shape;
    s->mod_seen   = mod;
  }

  // A standing pattern whose last pass ran start to finish without the knobs
  // moving has already been measured exactly; measuring it again is arithmetic
  // with a known answer. This is what keeps eight idle stepped channels free.
  if (!s->moved && s->slot == 0 && s->measured && s->length_idx == (int8_t) length_idx &&
      fabsf(s->norm.gain - s->target.gain) < ST_NORM_SETTLED && fabsf(s->norm.offset - s->target.offset) < ST_NORM_SETTLED)
  {
    s->norm = s->target; // land on it exactly, so this holds next tick too
    return;
  }

  if (!s->measured)
  {
    // A channel's first tick in this mode has nothing to correct with and
    // nothing to slew from, so it pays for one full measurement - the only
    // place that cost is taken, since a mode change is not a per-tick event.
    s->target     = st_norm_exact(shape, mod, length_idx);
    s->norm       = s->target;
    s->length_idx = (int8_t) length_idx;
    s->slot       = 0;
    s->measured   = 1;
    return;
  }

  if (s->length_idx != (int8_t) length_idx)
  {
    // A length change swaps the whole pattern at once, so the pass in progress
    // is measuring something that is no longer playing. Started again rather
    // than measured in full: the full route is `length` slot evaluations in one
    // tick, which is three ticks' worth of budget, and the encoder produces one
    // of these per detent.
    s->length_idx = (int8_t) length_idx;
    s->slot       = 0;
    s->moved      = 1;
  }

  // Not this channel's turn: the slew below still runs, so a correction already
  // measured keeps arriving smoothly.
  if (may_measure)
  {
    StMorph m = st_morph(shape, mod, length, ST_HOLD_MAX);
    float v   = st_step_value(s->slot, &m, &st_slots, ST_JUMP_GRID);

    if (s->slot == 0)
    {
      s->lo = s->hi = s->anchor = v; // the pass starts on the slot it is anchored to
      s->moved                  = 0; // and from here it is measuring one pattern
    }
    else if (v < s->lo)
    {
      s->lo = v;
    }
    else if (v > s->hi)
    {
      s->hi = v;
    }

    if (++s->slot >= (int16_t) length)
    {
      s->slot = 0;

      StExtent e   = {s->lo, s->hi, 0.0f, s->anchor}; // the DC is the table's job, not this one
      float centre = st_centre_at(mod, m.morph);
      float g      = st_gain_for(&e, centre);
      float gf     = st_gain_floor(&e, centre);
      s->target    = st_norm_affine(centre, s->anchor, (gf > g) ? gf : g);
    }
  }

  float k = fclamp(dt_s / ST_NORM_SMOOTH_S, 0.0f, 1.0f);
  s->norm.gain += (s->target.gain - s->norm.gain) * k;
  s->norm.offset += (s->target.offset - s->norm.offset) * k;
}

// How far MOD winds the ease up: fully slewed at one end, ST_HOLD_HARD at the
// other, so one knob carries a smooth wander through to a held gate.
#define ST_DRIVE_HOLD ST_HOLD_HARD

// How far the ends of SHP drive the distribution.
//
// Set against the small-turn budget, which in this style is mostly spent before
// the bias gets to it: at full hold the curve sits on its step values instead of
// smoothing between them, so a change to one shows up almost undiminished, and
// the worst turn measures 0.34 of the 0.35 limit with the bias at zero. What the
// bias adds on top is small - 0.33 at depth 0.5, 0.35 at 0.7 - because what the
// map expands at one end of the range it compresses at the other.
//
// At this depth the low end sits at a mean of -0.51 with the peaks still
// reaching, and the gate end empties the middle of the range from 54% of the
// time to 34%.
#define ST_BIAS_DEPTH 0.6f

// How deep the terracing goes, and how many times it comes and goes across SHP.
//
// A rate above one is the point of it: the depth can be changed quickly without
// the pattern being redrawn, so this is where a detent gets something to do.
// What it costs is nothing the small-turn rule charges for - rho is identical to
// six decimal places with it at zero and at full depth - and 0.10 of the leap
// guard, which is what that guard is for.
#define ST_TERRACE 0.7f
#define ST_TERRACE_RATE 5

StDrive st_drive(float shape, float mod)
{
  StDrive d;
  d.terrace = ST_TERRACE * st_bump(0.5f * (fclamp(shape, -1.0f, 1.0f) + 1.0f), ST_TERRACE_RATE);

  // SHP is the distribution, one traversal of it, with the pattern still
  // advancing underneath - a knob that only reshaped would be a knob that never
  // reaches a different pattern.
  d.bias = ST_BIAS_DEPTH * fclamp(shape, -1.0f, 1.0f);

  // MOD is motion: a new value every step and fully slewed at one end, mostly
  // tied and hard-stepped at the other. Density already moves with it, so this
  // is the same axis carrying the ease as well.
  d.hold = ST_DRIVE_HOLD * 0.5f * (fclamp(mod, -1.0f, 1.0f) + 1.0f);
  return d;
}

float stepped_shape_with(float phase, float shape, float mod, int length_idx, const StDrive* drive, const StNorm* norm)
{
  length_idx = iclamp(length_idx, 0, ST_LENGTH_COUNT - 1);
  int length = st_lengths[length_idx];

  StMorph m = st_morph(shape, mod, length, ST_HOLD_MAX);

  // Which step the playhead is in. The steps are not all the same width - MOD
  // skews alternate ones long and short - so this is a small search rather than
  // a multiply, but still O(1).
  float within;
  int step = st_step_at(phase, length, st_swing_amount(mod), &within);
  int next = (step + 1 == length) ? 0 : step + 1;

  float from, to;
  st_step_pair(step, next, &m, &st_slots, ST_JUMP_GRID, &from, &to);

  // Hold, then ease.
  float span = 1.0f - drive->hold;
  float ease = (span <= 0.0f) ? 1.0f : fclamp((within - drive->hold) / span, 0.0f, 1.0f);

  float value = lerp(from, to, smoothstep(ease));

  // The correction is one affine for the whole cycle, so the loop still closes
  // and the curve stays smooth. See stepped_norm.h for what it aims at.
  // The bias reshapes what comes out of it and is monotone, so it cannot break
  // either property: a continuous curve stays continuous and the loop point
  // still meets itself.
  // The reshaping is monotone and lands after the correction, so it can break
  // neither the loop point, nor the continuity, nor the level just established
  // - and it cannot reorder a step, which is what keeps a fast knob from
  // reading as a fresh draw.
  float corrected = fclamp(value * norm->gain + norm->offset, -1.0f, 1.0f);
  return st_terrace_map(st_bias_map(corrected, drive->bias), drive->terrace);
}

float stepped_shape(float phase, float shape, float mod, int length_idx, float hold)
{
  StNorm n  = st_norm_exact(shape, mod, length_idx);
  StDrive d = {hold, 0.0f, 0.0f};
  return stepped_shape_with(phase, shape, mod, length_idx, &d, &n);
}
