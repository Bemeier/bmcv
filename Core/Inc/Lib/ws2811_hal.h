#ifndef INC_DRIVERS_WS2811_HAL_H_
#define INC_DRIVERS_WS2811_HAL_H_

#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include "ws2811.h"

void ws2811_init(TIM_HandleTypeDef* htim, uint32_t channel);

void ws2811_dma_complete_callback(TIM_HandleTypeDef* htim);

#endif /* INC_DRIVERS_WS2811_HAL_H_ */
