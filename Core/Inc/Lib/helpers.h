#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include "math.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define FP_SCALE 1000
#define SEMITONE_DAC_FP 273067

static const float US_TO_S = 1e-6f;

#define US(x) ((uint32_t) (x))
#define MS(x) ((uint32_t) ((x) * 1000u))
#define S(x) ((uint32_t) ((x) * 1000000u))

// how close is inter it to right value
static inline uint8_t interpolate_clamped(uint16_t left, uint16_t right, uint16_t inter)
{
  if (inter <= left)
  {
    return 0;
  }
  else if (inter >= right)
  {
    return 255;
  }
  else
  {
    // Promote to 32-bit to prevent overflow
    uint32_t numerator   = (uint32_t) (inter - left) * 255;
    uint16_t denominator = right - left;

    // Round to nearest integer
    return (uint8_t) ((numerator + (denominator / 2)) / denominator);
  }
}

static inline uint8_t linear_step(uint8_t x, uint8_t c)
{
  if (x <= c)
  {
    return 0;
  }
  else if (x >= 255 - c)
  {
    return 255;
  }
  else
  {
    uint16_t numerator   = (uint16_t) (x - c) * 255;
    uint16_t denominator = 255 - 2 * c;
    return (uint8_t) ((numerator + (denominator / 2)) / denominator);
  }
}

static inline int16_t smin(int16_t a, int16_t b) { return b ^ ((a ^ b) & -(a < b)); }

static inline int16_t smax(int16_t a, int16_t b) { return a ^ ((a ^ b) & -(a < b)); }

static inline int16_t sclamp(int16_t x, int16_t min_val, int16_t max_val)
{
  x = x ^ ((x ^ min_val) & -(x < min_val));
  x = x ^ ((x ^ max_val) & -(x > max_val));
  return x;
}

static inline float fclamp(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

static inline int iclamp(int val, int min, int max)
{
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

static inline int imin(int a, int b) { return (a < b) ? a : b; }

static inline int imax(int a, int b) { return (a > b) ? a : b; }

static inline float warp(float x, float k) { return copysignf(powf(fabsf(x), k), x); }

static inline float warp_distance(float d, float mod)
{
  float k = 1.0f + 4.0f * fabsf(mod);

  if (mod > 0.0f)
  {
    // compress positive side, stretch negative side
    return (d >= 0.0f) ? powf(d, k) : -powf(-d, 1.0f / k);
  }
  else
  {
    // inverse asymmetry
    return (d >= 0.0f) ? powf(d, 1.0f / k) : -powf(-d, k);
  }
}

static inline float warp_phase(float p, float mod)
{
  float k = 1.0f + 4.0f * fabsf(mod);

  if (mod > 0.0f)
    return powf(p, k);
  else
    return 1.0f - powf(1.0f - p, k);
}

static inline uint32_t hash_u32(uint32_t x)
{
  x ^= x >> 16;
  x *= 0x7feb352d;
  x ^= x >> 15;
  x *= 0x846ca68b;
  x ^= x >> 16;
  return x;
}

// 0..1 float
static inline float hash01(uint32_t x) { return (hash_u32(x) & 0x00FFFFFF) * (1.0f / 16777216.0f); }

// -1..1 float
static inline float hash11(uint32_t x) { return hash01(x) * 2.0f - 1.0f; }

static inline float fractf(float x) { return x - floorf(x); }

static inline float lerp(float a, float b, float t) { return a + t * (b - a); }

static inline float smoothstep(float t) { return t * t * (3.0f - 2.0f * t); }

// Skew: leans a waveform early or late without moving where it starts or ends.
//
// One rational warp rather than the two straight segments this used to be,
// which had a slope discontinuity where they met - a visible kink halfway
// through the cycle - and a curvature term riding on the same parameter, so one
// knob did two things. f(0)=0, f(1)=1 and monotone, so the cycle still closes
// seamlessly; negative leans the shape early, positive late.
static inline float phase_mod(float phase, float mod)
{
  if (mod == 0.0f)
  {
    return phase; // guaranteed clean original
  }

  // Clamped off the ends: r goes to 0 or infinity there.
  mod = fclamp(mod, -0.98f, 0.98f);

  float r = (1.0f + mod) / (1.0f - mod); // 1 at centre, 1/99 .. 99 across the knob
  return phase / (phase + (1.0f - phase) * r);
}

// Round-to-nearest integer division, symmetric about zero. C's built-in
// division truncates toward zero, which biases every result toward 0V.
// den must be > 0.
static inline int32_t div_round_nearest(int32_t num, int32_t den)
{
  return (num >= 0) ? (num + den / 2) / den : -(((-num) + den / 2) / den);
}

// Snap a CV value to the nearest enabled semitone of the shared scale.
//
// The search runs in fixed-point DAC units (DAC * FP_SCALE) rather than whole
// semitones, so the input's position *within* a semitone survives and
// "nearest" really is nearest. Doing it in integer semitone space instead
// silently floors toward 0V and can pick a note almost a full semitone away.
//
// For each enabled scale degree we jump straight to its nearest octave
// transposition, so 12 candidates suffice and there is no octave window to
// get wrong at negative voltages.
//
// The result is an exact semitone and nothing else. This used to subtract a
// DAC_OFFSET_CORRECTION of 82 units - about 25mV - so that the *analog* output
// landed on the true note. That put a correction for the board's zero error
// inside the one function that happened to care about absolute voltage, which
// meant every unquantized channel carried the same error uncorrected. Measured
// on the bench it is not worth correcting: the outputs are close enough
// untouched that a two-point calibration could not improve on them.
static inline int16_t quantize_value(int16_t input, uint16_t scale_mask)
{
  if (scale_mask == 0)
    return input; // no notes enabled: pass through unquantized

  const int32_t x_fp      = (int32_t) input * FP_SCALE;
  const int32_t octave_fp = 12 * SEMITONE_DAC_FP;

  int32_t best_fp   = 0;
  int32_t best_dist = INT32_MAX;

  for (int n = 0; n < 12; n++)
  {
    if (!(scale_mask & (1u << n)))
      continue;

    int32_t note_fp = (int32_t) n * SEMITONE_DAC_FP;
    note_fp += div_round_nearest(x_fp - note_fp, octave_fp) * octave_fp;

    int32_t dist = x_fp - note_fp;
    if (dist < 0)
      dist = -dist;

    if (dist < best_dist)
    {
      best_dist = dist;
      best_fp   = note_fp;
    }
  }

  return (int16_t) iclamp(div_round_nearest(best_fp, FP_SCALE), INT16_MIN, INT16_MAX);
}

static inline uint32_t crc32(const void* data, size_t len)
{
  // Cast spelled out: implicit from void* is fine in C, but this header is
  // also read by the C++ side of the VCV Rack plugin.
  const uint8_t* p = (const uint8_t*) data;
  uint32_t crc     = 0xFFFFFFFF;

  while (len--)
  {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++)
    {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

static inline int16_t find_denominator(float value, int16_t max_mult, float tol)
{
  float ip;
  float frac = modff(value, &ip);

  if (frac < tol || (1.0f - frac) < tol)
    return 1;

  const float threshold = tol;

  for (int16_t mult = 1; mult <= max_mult; mult++)
  {
    float test = frac * (float) mult;

    float nearest = test + 0.5f;
    int32_t k     = (int32_t) nearest;

    if (k < 1)
      continue;

    if (fabsf(test - (float) k) < threshold)
      return mult;
  }

  return -1;
}

static inline float phase_error(float a, float b, float X)
{
  float d = a - b;

  if (d > X * 0.5f)
    d -= X;
  else if (d < -X * 0.5f)
    d += X;

  return d;
}

static inline int16_t val_neighbour(const int16_t val, const int16_t delta, const int16_t* values, const size_t n_values, size_t* idx)
{

  int16_t cdelta = (int16_t) iclamp(delta, -1, 1);
  if (cdelta == 0)
  {
    int32_t diff = INT32_MAX;
    for (uint8_t i = 0; i < n_values; i++)
    {
      int32_t val_diff = abs(values[i] - val);
      if (val_diff <= diff)
      {
        diff = val_diff;
        *idx = i;
      }
    }
  }
  else
  {

    size_t left  = 0;
    size_t right = n_values;

    while (left < right)
    {
      size_t mid = left + (right - left) / 2;
      if (values[mid] < val)
      {
        left = mid + 1;
      }
      else
      {
        right = mid;
      }
    }

    if (cdelta > 0)
    {
      /* Next higher */
      *idx = left;
      if (*idx < n_values && values[*idx] == val)
      {
        (*idx)++;
      }
      if (*idx >= n_values)
      {
        *idx = 0; // wrap
      }
    }
    else
    {
      /* Next lower */
      if (left < n_values && values[left] == val)
      {
        *idx = (left > 0) ? left - 1 : n_values - 1;
      }
      else
      {
        *idx = (left > 0) ? left - 1 : n_values - 1;
      }
    }
  }

  return values[*idx];
}

#endif /* INC_HELPERS_H_ */
