#ifndef INC_DRIVERS_DAC_ADC_H_
#define INC_DRIVERS_DAC_ADC_H_

#include "hw_setup.h" // IWYU pragma: keep - CV range and TRIG_THRESH constants
#include <stdint.h>

#define DAC_CHANNELS 4
#define DAC_CHANNEL_DATA_WIDTH 6

void dacadc_write(uint8_t idx, int16_t data);

void dac_init();

// Arm the next transaction: the levels already in DAC_BUF go out, the previous
// conversion comes back. Cannot fail - it is a handful of register writes - so
// it says nothing, where the HAL version returned whether the transfer started.
void dacadc_dma_next(void);

int16_t get_adc(uint8_t channel);

uint8_t adc_read_trig_state(uint8_t channel);

#endif /* INC_DRIVERS_DAC_ADC_H_ */
