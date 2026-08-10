#include "fram.h"

static SPI_HandleTypeDef* fram_spi;

static GPIO_TypeDef* fram_cs_port;
static uint16_t fram_cs_pin;

void fram_init(SPI_HandleTypeDef* spi, GPIO_TypeDef* port, uint16_t pin)
{
  fram_spi     = spi;
  fram_cs_port = port;
  fram_cs_pin  = pin;
}

// Static: the part needs this immediately before every write, so fram_Write
// below issues it itself rather than leaving it to a caller to remember. It
// was exported, and the one exported thing that used to sit beside it -
// fram_WriteByte - did not call it, which is the mistake this arrangement
// stops anyone repeating.
static void fram_WriteEnable(void)
{
  uint8_t tx[1] = {FRAM_WREN};

  HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(fram_spi, tx, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_SET);
}

void fram_Write(uint16_t addr, const uint8_t* data, uint16_t len)
{
  uint8_t header[3] = {FRAM_WRITE, (addr >> 8) & 0xFF, addr & 0xFF};

  fram_WriteEnable();

  HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(fram_spi, header, 3, HAL_MAX_DELAY);
  HAL_SPI_Transmit(fram_spi, (uint8_t*) data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_SET);
}

void fram_Read(uint16_t addr, uint8_t* data, uint16_t len)
{
  uint8_t header[3] = {FRAM_READ, (addr >> 8) & 0xFF, addr & 0xFF};

  HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(fram_spi, header, 3, HAL_MAX_DELAY);
  HAL_SPI_Receive(fram_spi, data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_SET);
}
