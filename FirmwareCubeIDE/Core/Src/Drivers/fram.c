#include "fram.h"

static SPI_HandleTypeDef * fram_spi;

static GPIO_TypeDef * fram_cs_port;
static uint16_t fram_cs_pin;

  /* Example:
	test_val = 0x5A;
	FRAM_WriteEnable();
	FRAM_WriteByte(0x0010, test_val);
	read_val = FRAM_ReadByte(0x0010);
  */

void fram_init(SPI_HandleTypeDef * spi, GPIO_TypeDef * port, uint16_t pin) {
    fram_spi = spi;
    fram_cs_port = port;
    fram_cs_pin = pin;
}

void fram_WriteEnable() {
    uint8_t tx[1] = {
    	FRAM_WREN
    };

    HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(fram_spi, tx, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_SET);
}


void fram_WriteByte(uint16_t addr, uint8_t data) {
    uint8_t tx[4] = {
        FRAM_WRITE,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        data
    };

    HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(fram_spi, tx, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_SET);
}

uint8_t fram_ReadByte(uint16_t addr) {
    uint8_t tx[4] = {
        FRAM_READ,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
		0
    };
    uint8_t rx[4] = { 0 };


    HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(fram_spi, tx, rx, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_SET);

    return rx[3];
}
