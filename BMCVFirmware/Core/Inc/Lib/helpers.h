#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include "dac_adc.h"
#include "math.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define LUT_SIZE (ADC_10V + ADC_10V + 1)

#define FP_SCALE 1000
#define SEMITONE_DAC_FP 273067

#define DAC_OFFSET_CORRECTION 82

static const float US_TO_S = 1e-6f;

#define US(x) ((uint32_t) (x))
#define MS(x) ((uint32_t) ((x) * 1000u))
#define S(x) ((uint32_t) ((x) * 1000000u))

static int16_t quantLUT[LUT_SIZE];
static inline void generate_quant_lut(void)
{
  for (int i = -ADC_10V; i <= ADC_10V; ++i)
  {
    int32_t dac_val    = i * 4;
    int64_t dac_val_fp = (int64_t) dac_val * FP_SCALE;

    // Quantize to nearest semitone
    int32_t semitone_index = (dac_val_fp + SEMITONE_DAC_FP / 2) / SEMITONE_DAC_FP;
    int32_t quantized_dac  = (semitone_index * SEMITONE_DAC_FP + FP_SCALE / 2) / FP_SCALE;

    quantized_dac -= DAC_OFFSET_CORRECTION;

    if (quantized_dac < -DAC_10V)
      quantized_dac = -DAC_10V;
    if (quantized_dac > DAC_10V)
      quantized_dac = DAC_10V;

    quantLUT[i + ADC_10V] = (int16_t) quantized_dac;
  }
}

static inline int16_t quantize_adc(int16_t input)
{
  if (input < -ADC_10V)
    input = -ADC_10V;
  if (input > ADC_10V)
    input = ADC_10V;
  return quantLUT[input + ADC_10V];
}

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

static inline float phase_mod(float phase, float mod)
{
  mod = fclamp(mod, -1.0f, 1.0f);

  if (mod == 0.0f)
  {
    return phase; // ← guaranteed clean original
  }

  float warp = 0.5f + mod * 0.45f; // safe range

  float p;
  if (phase < warp)
  {
    p = phase / (2.0f * warp);
  }
  else
  {
    p = 0.5f + (phase - warp) / (2.0f * (1.0f - warp));
  }

  // Optional curvature — scaled by |mod| so it disappears at mod=0
  float curve_amount = fabsf(mod) * 0.8f;         // 0.0 → 0.8 (adjust to taste)
  float curved       = p * p * (3.0f - 2.0f * p); // smoothstep

  // Blend between linear warped phase and curved version
  return lerp(p, curved, curve_amount);
}

static inline float smoothstep_edge(float edge0, float edge1, float x)
{
  // Clamp x to [edge0, edge1]
  x = fclamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);

  // Smooth hermite interpolation
  return x * x * (3.0f - 2.0f * x);
}

static inline int delta_modulo_step(int val, int delta, int maxVal)
{
  delta = iclamp(delta, -maxVal, maxVal);
  return (val + delta + maxVal) % maxVal;
}

static inline int16_t quantize_value(int16_t input, uint16_t scale_mask)
{
  // Convert DAC units → semitone index
  int32_t semitone = ((int32_t) input * FP_SCALE) / SEMITONE_DAC_FP;

  int32_t best_note = 0;
  int32_t best_dist = INT32_MAX;

  int32_t octave = semitone / 12;

  for (int n = 0; n < 12; n++)
  {
    if (scale_mask & (1u << n))
    {
      int32_t candidates[3] = {(octave - 1) * 12 + n, octave * 12 + n, (octave + 1) * 12 + n};

      for (int c = 0; c < 3; c++)
      {
        int32_t note = candidates[c];
        int32_t dist = semitone - note;
        if (dist < 0)
          dist = -dist;
        if (dist < best_dist)
        {
          best_dist = dist;
          best_note = note;
        }
      }
    }
  }

  // Convert quantized semitone back to DAC units
  int32_t quantized_dac = (best_note * SEMITONE_DAC_FP + FP_SCALE / 2) / FP_SCALE;

  return (int16_t) iclamp(quantized_dac, INT16_MIN, INT16_MAX);
}

static inline uint32_t crc32(const void* data, size_t len)
{
  const uint8_t* p = data;
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
