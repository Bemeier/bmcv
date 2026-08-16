#include "drift.h"
#include "helpers.h"
#include "stepped_random.h"         // sr_bias_map
#include "stepped_random_pattern.h" // sr_tri, sr_scurve
#include "stepped_random_table.h"   // the slot constants, reused as phases here

// How many octaves. Four spans a factor of eight in rate, which is the range
// between "one swell per cycle" and "detail you can hear moving" at the rates
// this module runs at. A fifth would sit above the LFO rates most of these
// channels are used at and cost a triangle to be inaudible.
#define DRIFT_OCTAVES 4

// A smoothed triangle rather than a triangle: sr_scurve has zero slope at both
// peaks, so composing it over sr_tri rounds the corner where the triangle turns
// around. Without it the wander has audible kinks at every peak, which is the
// one thing a wander should not have. Neither is a sine, and neither costs a
// call.
static float wave(float x) { return sr_scurve(sr_tri(x)); }

// How far SHP carries the wander across its sweep. Three excursions, not one:
// measured, this shape spends only 0.09 of the 0.35 small-turn budget at one,
// so the knob can cover three times the ground and still move gently. What that
// buys is a detent that does something - the complaint every one of these modes
// started from.
//
// Whole excursions, so the knob still wraps: each octave's phase advances by
// three times its own whole-numbered rate over the sweep.
#define DRIFT_ORBIT_RATE 3.0f

// The single octave at the left of SHP swings the whole range on its own, and
// octaves that only partly line up take about 30% off that as they come in. So
// the sum is scaled back toward what one octave does, which holds the level
// steady across the knob rather than letting detail cost loudness. A ramp with
// two constants, against the stepped modes' table and per-channel scan - the
// whole reason this shape is the cheap one.
static float octave_gain(float rough) { return lerp(0.74f, 1.03f, smoothstep(fclamp(rough * 2.0f, 0.0f, 1.0f))); }

float drift_value(float phase, float shape, float mod)
{
  float rough = 0.5f * (fclamp(shape, -1.0f, 1.0f) + 1.0f);
  float orbit = rough * DRIFT_ORBIT_RATE;

  float sum = 0.0f, wsum = 0.0f, w = 1.0f;
  for (int k = 0; k < DRIFT_OCTAVES; k++)
  {
    // Each octave has its own starting phase and its own speed along SHP, both
    // read from the stepped modes' slot tables. They are already a set of
    // unrelated constants with whole-numbered rates, which is exactly what this
    // needs, and reusing them costs nothing where a second table would cost
    // flash.
    sum += w * wave((float) (1 << k) * phase + sr_slot_base[k] + orbit * (float) sr_slot_rate[k]);
    wsum += w;
    w *= rough;
  }

  // MOD leans the finished value, the same reshaping the control mode puts on
  // SHP: the map is monotone, so it cannot introduce a kink into a curve whose
  // smoothness is the point.
  return sr_bias_map(fclamp(sum / wsum * octave_gain(rough), -1.0f, 1.0f), fclamp(mod, -1.0f, 1.0f));
}
