#ifndef INC_DRIVERS_MCP_H_
#define INC_DRIVERS_MCP_H_
#include "buttons_encoders.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef* spiHandle;
    GPIO_TypeDef* csPortHandle;
    uint16_t csPin;
    GPIO_TypeDef* resetPortHandle;
    uint16_t resetPin;
    uint8_t gpioa_state;
    uint8_t gpiob_state;
    uint8_t a_state;
    uint8_t b_state;
    uint8_t a_state_prev;
    uint8_t b_state_prev;
    uint8_t enc_button_pins[N_ENCODERS];
    uint8_t bottom_button_pins[N_ENCODERS];
    uint8_t enc_pins_a[N_ENCODERS];
    uint8_t enc_pins_b[N_ENCODERS];
    volatile int16_t enc_position_state[N_ENCODERS]; // Tracked position per encoder
    uint8_t enc_pins_button[N_ENCODERS];
    uint8_t button_state[N_ENCODERS + 13 + 3];
    uint8_t tx_buf[8];
    uint8_t rx_buf[8];
    volatile uint8_t spi_dma_state;
} BTNENC;

void mcp_init(SPI_HandleTypeDef* spi);

int8_t mcp_read();

void mcu_read_buttons();

void mcp_handle_txrx_complete(SPI_HandleTypeDef* hspi);

#endif /* INC_DRIVERS_MCP_H_ */
