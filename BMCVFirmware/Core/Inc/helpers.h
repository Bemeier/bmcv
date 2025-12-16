#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include <stddef.h>
#include <stdint.h>

#define ADC_MIN (-8192)
#define ADC_MAX (8191)
#define LUT_SIZE (ADC_MAX - ADC_MIN + 1)

#define FP_SCALE 1000
#define SEMITONE_DAC_FP 273067

#define DAC_OFFSET_CORRECTION 82

static int16_t quantLUT[LUT_SIZE];
static inline void generate_quant_lut(void)
{
    for (int i = ADC_MIN; i <= ADC_MAX; ++i)
    {
        int32_t dac_val    = i * 4;
        int64_t dac_val_fp = (int64_t) dac_val * FP_SCALE;

        // Quantize to nearest semitone
        int32_t semitone_index = (dac_val_fp + SEMITONE_DAC_FP / 2) / SEMITONE_DAC_FP;
        int32_t quantized_dac  = (semitone_index * SEMITONE_DAC_FP + FP_SCALE / 2) / FP_SCALE;

        quantized_dac -= DAC_OFFSET_CORRECTION;

        if (quantized_dac < -32768)
            quantized_dac = -32768;
        if (quantized_dac > 32767)
            quantized_dac = 32767;

        quantLUT[i - ADC_MIN] = (int16_t) quantized_dac;
    }
}

static inline int16_t quantize_adc(int16_t input)
{
    if (input < ADC_MIN)
        input = ADC_MIN;
    if (input > ADC_MAX)
        input = ADC_MAX;
    return quantLUT[input - ADC_MIN];
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

#endif /* INC_HELPERS_H_ */
