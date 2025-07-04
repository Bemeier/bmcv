#ifndef INC_DRIVERS_DAC_ADC_H_
#define INC_DRIVERS_DAC_ADC_H_
#include "stm32g4xx_hal.h"

#define TRIG_THRESH 1024
#define TRIG_THRESH_LOW 800

typedef struct {
	SPI_HandleTypeDef *spiHandle;

	GPIO_TypeDef      *csadcPortHandle;
	uint16_t           csadcPin;

	GPIO_TypeDef      *cnvstPortHandle;
	uint16_t           cnvstPin;

	GPIO_TypeDef      *addrPortHandle;
	uint16_t           adrrPin;

	GPIO_TypeDef      *csdacPortHandle;
	uint16_t           csdacPin;

	uint8_t CH_IDX;
	uint8_t rx_buf[6];
	uint8_t offset;

	uint8_t DAC_BUF[24];
	uint16_t DAC_DATA[8];

	int16_t adc_i[4];
	int16_t adc_i_prev[4];
	int8_t trig_state[4];
	int8_t trig_flag[4];
} DAC_ADC;

void dacadc_init(SPI_HandleTypeDef * spi);

void dacadc_write(uint8_t idx, int16_t data);

void dacadc_transaction();

void dac_init();

uint8_t dacadc_dma_next();
void dacadc_dma_complete();

int16_t sign_extend_14bit(uint16_t val);

float adc_to_voltage(int16_t adc_value);

DAC_ADC* dacadc_instance(void);

#endif /* INC_DRIVERS_DAC_ADC_H_ */
