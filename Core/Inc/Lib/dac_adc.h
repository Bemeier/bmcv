#ifndef INC_DRIVERS_DAC_ADC_H_
#define INC_DRIVERS_DAC_ADC_H_

#include "hw_setup.h" // IWYU pragma: keep - CV range and TRIG_THRESH constants
#include <stdint.h>

#define DAC_CHANNELS 4
#define DAC_CHANNEL_DATA_WIDTH 6

void dacadc_write(uint8_t idx, int16_t data);

// void dacadc_transaction();

void dac_init();

int8_t dacadc_error();

uint8_t dacadc_dma_next();

int16_t sign_extend_14bit(uint16_t val);

float adc_to_voltage(int16_t adc_value);

int16_t get_adc(uint8_t channel);

uint8_t adc_read_trig_state(uint8_t channel);

#endif /* INC_DRIVERS_DAC_ADC_H_ */
