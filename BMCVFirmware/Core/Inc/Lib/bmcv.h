#ifndef INC_LIB_BMCV_H_
#define INC_LIB_BMCV_H_

#include "stm32g474xx.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

void bmcv_init(uint16_t mpc_interrupt_pin, ADC_TypeDef* slider_adc);

void bmcv_main(uint32_t now_us);

uint8_t bmcv_state_update(uint32_t now);

void bmcv_flush_leds(void);

void bmcv_handle_adc_conversion_complete(ADC_HandleTypeDef* hadc);

void bmcv_poll_tasks();

void bmcv_handle_gpio_exti(uint16_t GPIO_Pin);

void bmcv_handle_txrx_complete(SPI_HandleTypeDef* hspi);

#endif /* INC_LIB_BMCV_H_ */
