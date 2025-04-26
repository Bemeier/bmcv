#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_


#include <stdint.h>

#define ADC_MIN   (-8192)
#define ADC_MAX   (8191)
#define LUT_SIZE  (ADC_MAX - ADC_MIN + 1)

#define FP_SCALE              1000
#define SEMITONE_DAC_FP       273067

#define DAC_OFFSET_CORRECTION 82

static int16_t quantLUT[LUT_SIZE];
static inline void generate_quant_lut(void) {
    for (int i = ADC_MIN; i <= ADC_MAX; ++i) {
        int32_t dac_val = i * 4;
        int64_t dac_val_fp = (int64_t)dac_val * FP_SCALE;

        // Quantize to nearest semitone
        int32_t semitone_index = (dac_val_fp + SEMITONE_DAC_FP / 2) / SEMITONE_DAC_FP;
        int32_t quantized_dac = (semitone_index * SEMITONE_DAC_FP + FP_SCALE / 2) / FP_SCALE;

        quantized_dac -= DAC_OFFSET_CORRECTION;

        if (quantized_dac < -32768) quantized_dac = -32768;
        if (quantized_dac >  32767) quantized_dac =  32767;

        quantLUT[i - ADC_MIN] = (int16_t) quantized_dac;
    }
}

static inline int16_t quantize_adc(int16_t input) {
    if (input < ADC_MIN) input = ADC_MIN;
    if (input > ADC_MAX) input = ADC_MAX;
    return quantLUT[input - ADC_MIN];
}

static inline int16_t smin(int16_t a, int16_t b) {
    return b ^ ((a ^ b) & -(a < b));
}

static inline int16_t smax(int16_t a, int16_t b) {
    return a ^ ((a ^ b) & -(a < b));
}

static inline int16_t sclamp(int16_t x, int16_t min_val, int16_t max_val) {
    x = x ^ ((x ^ min_val) & -(x < min_val));
    x = x ^ ((x ^ max_val) & -(x > max_val));
    return x;
}



#endif /* INC_HELPERS_H_ */
