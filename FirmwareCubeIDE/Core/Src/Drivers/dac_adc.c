#include "dac_adc.h"

void ADC_Init(DAC_ADC * dacadc)
{
    // Set default pin levels
    HAL_GPIO_WritePin(dacadc->csadcPortHandle, dacadc->csadcPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dacadc->cnvstPortHandle, dacadc->cnvstPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dacadc->addrPortHandle, dacadc->adrrPin, GPIO_PIN_RESET);

    HAL_Delay(1); // Let things settle

    // Optional: do a dummy read to check communication
}


void WRITE_DAC_VALUE(DAC_ADC * dacadc, int idx, int16_t data) {
	dacadc->DAC_BUF[idx * 3 + 1] = (data >> 8) & 0xFF;
	dacadc->DAC_BUF[idx * 3 + 2] = data & 0xFF;
}

void ADC_DAC_Transaction(DAC_ADC * dacadc) {
    uint8_t rx_buf[6] = {0};
    uint16_t adc_a_raw = 0;
    uint16_t adc_b_raw = 0;

    for (uint8_t CH_IDX = 0; CH_IDX < 4; CH_IDX++) {
		uint8_t offset = (HAL_GPIO_ReadPin(dacadc->addrPortHandle, dacadc->adrrPin) == GPIO_PIN_SET) ? 2 : 0;
		HAL_GPIO_WritePin(dacadc->cnvstPortHandle, dacadc->cnvstPin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(dacadc->addrPortHandle, dacadc->adrrPin);
		HAL_GPIO_WritePin(dacadc->cnvstPortHandle, dacadc->cnvstPin, GPIO_PIN_SET);

		// TRANSMIT 1 channel to both DACs
		HAL_GPIO_WritePin(dacadc->csadcPortHandle, dacadc->csadcPin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dacadc->csdacPortHandle, dacadc->csdacPin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(dacadc->spiHandle, &(dacadc->DAC_BUF[CH_IDX*6]), rx_buf, 6, HAL_MAX_DELAY); //
		HAL_GPIO_WritePin(dacadc->csdacPortHandle, dacadc->csdacPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dacadc->csadcPortHandle, dacadc->csadcPin, GPIO_PIN_SET);
		adc_a_raw = ((rx_buf[0] << 6) | (rx_buf[1] >> 2)) & 0x3FFF;
		adc_b_raw = (((rx_buf[1] & 0x03) << 12) | (rx_buf[2] << 4) | (rx_buf[3] >> 4)) & 0x3FFF;
		dacadc->adc_i[0+offset] = sign_extend_14bit(adc_a_raw);
		dacadc->adc_i[1+offset] = sign_extend_14bit(adc_b_raw);
		//dacadc->adc_f[0+offset] = adc_to_voltage(adc_i[0+offset]);
		//dacadc->adc_f[1+offset] = adc_to_voltage(adc_i[1+offset]);
    }
}


void DAC_Init(DAC_ADC * dacadc) {
	dacadc->DAC_BUF[ 0] = (0b00000000); // DAC1 CHA
	dacadc->DAC_BUF[ 3] = (0b00000000); // DAC2 CHA
	dacadc->DAC_BUF[ 6] = (0b00000001); // DAC1 CHB
	dacadc->DAC_BUF[ 9] = (0b00000001); // DAC2 CHB
	dacadc->DAC_BUF[12] = (0b00000010); // DAC1 CHC
	dacadc->DAC_BUF[15] = (0b00000010); // DAC2 CHC
	dacadc->DAC_BUF[18] = (0b00000011); // DAC1 CHD
	dacadc->DAC_BUF[21] = (0b00000011); // DAC2 CHD

    // Step 1: Set all GPIOs to default state
	HAL_GPIO_WritePin(dacadc->csdacPortHandle, dacadc->csdacPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dacadc->ldacPortHandle, dacadc->ldacPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dacadc->clrPortHandle, dacadc->clrPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(dacadc->clrPortHandle, dacadc->clrPin, GPIO_PIN_SET);

    //control_bits |= (range_code & 0x07);  // Bits 2:0 = range select
    // All other control bits = 0 (normal operation, Slew Rate Off, etc.)

    HAL_Delay(1);

    uint16_t control_bits = 0b0000000000000100;

    uint8_t tx_buf[6];
    tx_buf[0] = (0b00001100);
    tx_buf[1] = (control_bits >> 8) & 0xFF;
    tx_buf[2] =  control_bits & 0xFF;
    tx_buf[3] = (0b00001100);
    tx_buf[4] = (control_bits >> 8) & 0xFF;
    tx_buf[5] =  control_bits & 0xFF;


    HAL_GPIO_WritePin(dacadc->csdacPortHandle, dacadc->csdacPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dacadc->spiHandle, tx_buf, 6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dacadc->csdacPortHandle, dacadc->csdacPin, GPIO_PIN_SET);

    HAL_Delay(1);

    tx_buf[0] = (0b00010000);
    tx_buf[1] = 0xFF;
    tx_buf[2] = 0xFF;
    tx_buf[3] = (0b00010000);
    tx_buf[4] = 0xFF;
    tx_buf[5] = 0xFF;

    HAL_GPIO_WritePin(dacadc->csdacPortHandle, dacadc->csdacPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dacadc->spiHandle, tx_buf, 6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dacadc->csdacPortHandle, dacadc->csdacPin, GPIO_PIN_SET);

    HAL_Delay(1);

    HAL_GPIO_WritePin(dacadc->ldacPortHandle, dacadc->ldacPin, GPIO_PIN_RESET);

}


int16_t sign_extend_14bit(uint16_t val) {
    return (int16_t)((int32_t)(val << 18) >> 18);
}

float adc_to_voltage(int16_t adc_value) {
    return ((float)adc_value / 8192.0f) * 10.0f;  // Assuming full scale ±10V
}
