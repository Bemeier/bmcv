#ifndef INC_DRIVERS_WS2811_HAL_H_
#define INC_DRIVERS_WS2811_HAL_H_

#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include "ws2811.h"

void ws2811_init(TIM_HandleTypeDef* htim, uint32_t channel);

void ws2811_dma_complete_callback(TIM_HandleTypeDef* htim);

// Hooked to HAL_TIM_ErrorCallback. A DMA transfer error never reaches the
// completion callback above, so without this the driver would wait forever for
// a frame that has already failed.
void ws2811_dma_error_callback(TIM_HandleTypeDef* htim);

#endif /* INC_DRIVERS_WS2811_HAL_H_ */
