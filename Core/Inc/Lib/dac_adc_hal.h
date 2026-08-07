#ifndef INC_DRIVERS_DAC_ADC_HAL_H_
#define INC_DRIVERS_DAC_ADC_HAL_H_

#include "dac_adc.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep

typedef struct
{
  SPI_HandleTypeDef* spiHandle;

  GPIO_TypeDef* csadcPortHandle;
  uint16_t csadcPin;

  GPIO_TypeDef* cnvstPortHandle;
  uint16_t cnvstPin;

  GPIO_TypeDef* addrPortHandle;
  uint16_t adrrPin;

  GPIO_TypeDef* csdacPortHandle;
  uint16_t csdacPin;

  uint8_t CH_IDX;
  uint8_t rx_buf[DAC_CHANNEL_DATA_WIDTH];
  uint8_t offset;

  uint8_t DAC_BUF[DAC_CHANNELS * DAC_CHANNEL_DATA_WIDTH];

  volatile int16_t adc_i[DAC_CHANNELS];
  int16_t adc_i_prev[DAC_CHANNELS];
  volatile int8_t trig_state[DAC_CHANNELS];
  volatile int8_t trig_flag[DAC_CHANNELS];
} DAC_ADC;

void dacadc_init(SPI_HandleTypeDef* spi);

uint8_t dacadc_dma_complete(SPI_HandleTypeDef* hspi);

#endif /* INC_DRIVERS_DAC_ADC_HAL_H_ */
