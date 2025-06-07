#ifndef INC_DRIVERS_FRAM_H_
#define INC_DRIVERS_FRAM_H_

#include "stm32g4xx_hal.h"

#define FRAM_WRITE   0x02
#define FRAM_READ    0x03
#define FRAM_WREN    0x06

#define FRAM_CS_LOW()    HAL_GPIO_WritePin(SPI3_FRAM_CS_GPIO_Port, SPI3_FRAM_CS_Pin, GPIO_PIN_RESET)
#define FRAM_CS_HIGH()    HAL_GPIO_WritePin(SPI3_FRAM_CS_GPIO_Port, SPI3_FRAM_CS_Pin, GPIO_PIN_SET)

#endif /* INC_DRIVERS_FRAM_H_ */