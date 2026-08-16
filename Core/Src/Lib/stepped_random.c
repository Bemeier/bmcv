#include "stepped_random.h"
#include "helpers.h"
#include "stepped_random_norm.h"
#include "stepped_random_pattern.h"
#include "stepped_random_table.h"

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
// What each step *shows* lives in stepped_random_pattern.h, shared with the
// table generator. This file is the part that only the runtime needs: finding
// the step a phase lands in, easing between two of them, and applying the
// correction the generator worked out.

// sr_lengths[] (steps per cycle) lives in the generated table so the
// normalisation data cannot drift out of step with it. It is curated rather
// than every integer, so each knob position is musically distinct; the odd
// lengths (3, 5, 7) drift against a 4/4 clock and give polyrhythms rather than
// a locked loop.

static const SrSlots sr_slots = {sr_slot_base, sr_slot_rate, sr_slot_gate, sr_slot_gate2};

// sr_step_value() keeps one tie weight per slot of a run on the stack.
_Static_assert(SR_JUMP_GRID <= SR_MAX_JUMP_GRID, "a run of ties must fit the weight buffer");

int sr_length_for_index(int length_idx) { return sr_lengths[iclamp(length_idx, 0, SR_LENGTH_COUNT - 1)]; }

// Where the centring constant is read from: the one part of the correction a
// channel cannot work out for itself, because it is an average across every
// pattern length and a channel knows only its own. Interpolated on both bin
// axes; both endpoints of MOD sit exactly on a bin, so the top one is only ever
// reached exactly.
static float sr_centre_at(float mod, float morph)
{
  float bin_pos  = morph * (float) SR_NORM_BINS;
  int bin        = (int) bin_pos;
  float bin_frac = bin_pos - (float) bin;
  bin            = bin % SR_NORM_BINS;
  int bin_next   = (bin + 1) % SR_NORM_BINS;

  float mod_pos  = (fclamp(mod, -1.0f, 1.0f) + 1.0f) * 0.5f * (float) (SR_MOD_BINS - 1);
  int mod_bin    = (int) mod_pos;
  float mod_frac = mod_pos - (float) mod_bin;
  if (mod_bin >= SR_MOD_BINS - 1)
  {
    mod_bin  = SR_MOD_BINS - 2;
    mod_frac = 1.0f;
  }

  return lerp(lerp(sr_centre_table[mod_bin][bin], sr_centre_table[mod_bin][bin_next], bin_frac),
              lerp(sr_centre_table[mod_bin + 1][bin], sr_centre_table[mod_bin + 1][bin_next], bin_frac), mod_frac);
}

static const SrNormCtx sr_ctx = {&sr_slots, sr_lengths, SR_LENGTH_COUNT, SR_JUMP_GRID, SR_HOLD_MAX};

SrNorm sr_norm_exact(float shape, float mod, int length_idx)
{
  length_idx = iclamp(length_idx, 0, SR_LENGTH_COUNT - 1);
  SrMorph m  = sr_morph(shape, mod, sr_lengths[length_idx], SR_HOLD_MAX);
  return sr_norm_at(&sr_ctx, sr_lengths[length_idx], mod, m.orbit, sr_centre_at(mod, m.morph));
}

// How long the correction takes to follow a pattern that changed under it.
//
// A measurement takes `length` ticks to complete, which is 16ms at the longest
// pattern and the engine's usual rate - so a comparable smoothing reads as the
// level settling rather than as two settings being crossfaded, and a pass that
// straddled a length switch cannot step the output.
#define SR_NORM_SMOOTH_S 0.02f

// Near enough that slewing further cannot be heard, and the point at which a
// standing channel is allowed to stop working.
#define SR_NORM_SETTLED 1e-6f

void sr_norm_scan(SrScan* s, float shape, float mod, int length_idx, float dt_s, int may_measure)
{
  length_idx = (int8_t) iclamp(length_idx, 0, SR_LENGTH_COUNT - 1);
  int length = sr_lengths[length_idx];

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
      fabsf(s->norm.gain - s->target.gain) < SR_NORM_SETTLED && fabsf(s->norm.offset - s->target.offset) < SR_NORM_SETTLED)
  {
    s->norm = s->target; // land on it exactly, so this holds next tick too
    return;
  }

  if (!s->measured)
  {
    // A channel's first tick in this mode has nothing to correct with and
    // nothing to slew from, so it pays for one full measurement - the only
    // place that cost is taken, since a mode change is not a per-tick event.
    s->target     = sr_norm_exact(shape, mod, length_idx);
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
    SrMorph m = sr_morph(shape, mod, length, SR_HOLD_MAX);
    float v   = sr_step_value(s->slot, &m, &sr_slots, SR_JUMP_GRID);

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

      SrExtent e   = {s->lo, s->hi, 0.0f, s->anchor}; // the DC is the table's job, not this one
      float centre = sr_centre_at(mod, m.morph);
      float g      = sr_gain_for(&e, centre);
      float gf     = sr_gain_floor(&e, centre);
      s->target    = sr_norm_affine(centre, s->anchor, (gf > g) ? gf : g);
    }
  }

  float k = fclamp(dt_s / SR_NORM_SMOOTH_S, 0.0f, 1.0f);
  s->norm.gain += (s->target.gain - s->norm.gain) * k;
  s->norm.offset += (s->target.offset - s->norm.offset) * k;
}

// How far MOD winds the ease up in the control style: fully slewed at one end,
// SR_HOLD_HARD at the other, so the same knob carries a smooth wander through
// to a held gate.
#define SR_CTRL_HOLD SR_HOLD_HARD

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
#define SR_CTRL_BIAS 0.6f

SrDrive sr_style_drive(int8_t style, float shape, float mod)
{
  SrDrive d = {shape, mod, SR_HOLD_SMOOTH, 0.0f};

  if (style == SR_STYLE_CONTROL)
  {
    // SHP is the distribution, one traversal of it, with the pattern still
    // advancing underneath - a knob that only reshaped would be a knob that
    // never reaches a different pattern.
    d.bias = SR_CTRL_BIAS * fclamp(shape, -1.0f, 1.0f);

    // MOD is motion: new value every step and fully slewed at one end, mostly
    // tied and hard-stepped at the other. Density already moves with it, so
    // this is the same axis carrying the ease as well.
    d.hold = SR_CTRL_HOLD * 0.5f * (fclamp(mod, -1.0f, 1.0f) + 1.0f);
  }
  return d;
}

float stepped_random_with(float phase, float shape, float mod, int length_idx, float hold, const SrNorm* norm, float bias)
{
  length_idx = iclamp(length_idx, 0, SR_LENGTH_COUNT - 1);
  int length = sr_lengths[length_idx];

  SrMorph m = sr_morph(shape, mod, length, SR_HOLD_MAX);

  // Which step the playhead is in. The steps are not all the same width - MOD
  // skews alternate ones long and short - so this is a small search rather than
  // a multiply, but still O(1).
  float within;
  int step = sr_step_at(phase, length, sr_swing_amount(mod), &within);
  int next = (step + 1 == length) ? 0 : step + 1;

  float from, to;
  sr_step_pair(step, next, &m, &sr_slots, SR_JUMP_GRID, &from, &to);

  // Hold, then ease.
  float span = 1.0f - hold;
  float ease = (span <= 0.0f) ? 1.0f : fclamp((within - hold) / span, 0.0f, 1.0f);

  float value = lerp(from, to, smoothstep(ease));

  // The correction is one affine for the whole cycle, so the loop still closes
  // and the curve stays smooth. See stepped_random_norm.h for what it aims at.
  // The bias reshapes what comes out of it and is monotone, so it cannot break
  // either property: a continuous curve stays continuous and the loop point
  // still meets itself.
  return sr_bias_map(fclamp(value * norm->gain + norm->offset, -1.0f, 1.0f), bias);
}

float stepped_random(float phase, float shape, float mod, int length_idx, float hold)
{
  SrNorm n = sr_norm_exact(shape, mod, length_idx);
  return stepped_random_with(phase, shape, mod, length_idx, hold, &n, 0.0f);
}
