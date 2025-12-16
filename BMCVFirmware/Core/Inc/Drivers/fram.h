#ifndef INC_DRIVERS_FRAM_H_
#define INC_DRIVERS_FRAM_H_

#include "stm32g4xx_hal.h" // IWYU pragma: keep

#define FRAM_WRITE 0x02
#define FRAM_READ 0x03
#define FRAM_WREN 0x06

void fram_init(SPI_HandleTypeDef* spi, GPIO_TypeDef* port, uint16_t pin);

uint8_t fram_ReadByte(uint16_t addr);
void fram_WriteByte(uint16_t addr, uint8_t data);

void fram_Write(uint16_t addr, const uint8_t* data, uint16_t len);
void fram_Read(uint16_t addr, uint8_t* data, uint16_t len);

#endif /* INC_DRIVERS_FRAM_H_ */
