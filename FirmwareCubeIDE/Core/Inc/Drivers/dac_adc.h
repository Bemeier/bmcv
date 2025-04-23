#ifndef INC_DRIVERS_DAC_ADC_H_
#define INC_DRIVERS_DAC_ADC_H_
#include "stm32g4xx_hal.h"

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

	GPIO_TypeDef      *ldacPortHandle;
	uint16_t           ldacPin;

	GPIO_TypeDef      *clrPortHandle;
	uint16_t           clrPin;

	volatile uint8_t CH_IDX;
	uint8_t rx_buf[6];
	uint8_t offset;

	uint8_t DAC_BUF[24];
	uint16_t DAC_DATA[8];

	volatile int16_t adc_i[4];
	//volatile float adc_f[4];
} DAC_ADC;

void ADC_Init(DAC_ADC * dacadc);

void WRITE_DAC_VALUE(DAC_ADC * dacadc, int idx, int16_t data);

void ADC_DAC_Transaction(DAC_ADC * dacadc);

void DAC_Init(DAC_ADC * dacadc);

uint8_t DAC_ADC_DMA_Next(DAC_ADC * dacadc);
void DAC_ADC_DMA_Complete(DAC_ADC * dacadc);

int16_t sign_extend_14bit(uint16_t val);

float adc_to_voltage(int16_t adc_value);

#endif /* INC_DRIVERS_DAC_ADC_H_ */
