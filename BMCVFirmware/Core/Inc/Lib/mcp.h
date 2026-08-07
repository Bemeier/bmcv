#ifndef INC_DRIVERS_MCP_H_
#define INC_DRIVERS_MCP_H_
#include "buttons_encoders.h" // get_btn_state / get_enc_state
#include "mcp_decode.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

// The SPI half. What the port bytes mean is McpDecode's, in mcp_decode.h,
// where it can be tested without a bus behind it.
typedef struct
{
  SPI_HandleTypeDef* spiHandle;
  GPIO_TypeDef* csPortHandle;
  uint16_t csPin;
  GPIO_TypeDef* resetPortHandle;
  uint16_t resetPin;
  uint8_t tx_buf[8];
  uint8_t rx_buf[8];
  volatile uint8_t spi_dma_state;

  McpDecode dec;
} BTNENC;

void mcp_init(SPI_HandleTypeDef* spi);

int8_t mcp_read();

void mcu_read_buttons();

void mcp_handle_txrx_complete(SPI_HandleTypeDef* hspi);

#endif /* INC_DRIVERS_MCP_H_ */
