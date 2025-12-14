#ifndef INC_DRIVERS_DAC_ADC_H_
#define INC_DRIVERS_DAC_ADC_H_

#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

#define TRIG_THRESH 1024
#define TRIG_THRESH_LOW 800
#define DAC_CHANNELS 4
#define DAC_CHANNEL_DATA_WIDTH 6

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

void dacadc_write(uint8_t idx, int16_t data);

// void dacadc_transaction();

void dac_init();

int8_t dacadc_error();

uint8_t dacadc_dma_next();
uint8_t dacadc_dma_complete(SPI_HandleTypeDef* hspi);

int16_t sign_extend_14bit(uint16_t val);

float adc_to_voltage(int16_t adc_value);

int16_t get_adc(uint8_t channel);

uint8_t adc_read_trig_state(uint8_t channel);

#endif /* INC_DRIVERS_DAC_ADC_H_ */
