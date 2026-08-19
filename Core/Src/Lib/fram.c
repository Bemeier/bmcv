#include "fram.h"

static SPI_HandleTypeDef* fram_spi;

static GPIO_TypeDef* fram_cs_port;
static uint16_t fram_cs_pin;

void fram_init(SPI_HandleTypeDef* spi, GPIO_TypeDef* port, uint16_t pin)
{
  fram_spi     = spi;
  fram_cs_port = port;
  fram_cs_pin  = pin;

  // Deselect the part before anything talks to it, and this is the only place
  // that can. MX_GPIO_Init brings PA15 up *low*, so the FRAM is selected from
  // the moment that pin becomes an output until the first transaction happens
  // to raise it - and the rest of the peripheral init runs inside that window,
  // MX_SPI3_Init switching PB3 from JTDO to SPI3_SCK among it. Something in
  // there clocks the part while it is listening for an opcode, because the
  // first READ after reset is answered by nothing: the module read FFFFFFFF
  // where the record's magic should be, on every reset, until this line.
  //
  // Which cost the module its startup state and nothing else, so it looked
  // like a storage fault rather than a boot one. The failing transaction still
  // left CS high on the way out, which put the state machine back in step for
  // every access after it - the autosave wrote correctly, an explicit save and
  // recall worked, and only the read at boot, the one that restores the last
  // patch, ever saw it.
  HAL_GPIO_WritePin(fram_cs_port, fram_cs_pin, GPIO_PIN_SET);
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
