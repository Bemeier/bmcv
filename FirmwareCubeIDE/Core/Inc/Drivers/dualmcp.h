#ifndef INC_DRIVERS_DUALmcp_H_
#define INC_DRIVERS_DUALmcp_H_
#include "stm32g4xx_hal.h"

#define N_ENCODERS 8

typedef struct {
	volatile SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef      *csPortHandle;
	uint16_t           csPin;
	GPIO_TypeDef      *resetPortHandle;
	uint16_t           resetPin;
	uint8_t   gpioa_state;
	uint8_t   gpiob_state;
	uint8_t   a_state;
	uint8_t   b_state;
	uint8_t   a_state_prev;
	uint8_t   b_state_prev;
	uint8_t   enc_button_pins[N_ENCODERS];
	uint8_t   bottom_button_pins[N_ENCODERS];
	uint8_t   enc_pins_a[N_ENCODERS];
	uint8_t   enc_pins_b[N_ENCODERS];
	volatile int16_t   enc_position_state[N_ENCODERS]; // Tracked position per encoder
	uint8_t   enc_pins_button[N_ENCODERS];
	volatile int8_t    enc_button_state[N_ENCODERS];
	volatile int8_t    bottom_button_state[13];
	uint8_t   tx_buf[8];
	uint8_t   rx_buf[8];
	volatile uint8_t   spi_dma_state;
} DUALMCP;

#define MCP_IODIRA    0x00
#define MCP_IODIRB    0x01

#define MCP_IOPOLA    0x02
#define MCP_IOPOLB    0x03

#define MCP_GPINTENA  0x04
#define MCP_GPINTENB  0x05

#define MCP_DEFVALA   0x06
#define MCP_DEFVALB   0x07

#define MCP_INTCONA   0x08
#define MCP_INTCONB   0x09

#define MCP_IOCONA    0x0A
#define MCP_IOCONB    0x0B

#define MCP_GPPUA     0x0C
#define MCP_GPPUB     0x0D

#define MCP_INTFA     0x0E
#define MCP_INTFB     0x0F

#define MCP_INTCAPA   0x10
#define MCP_INTCAPB   0x11

#define MCP_GPIOA     0x12
#define MCP_GPIOB     0x13

#define MCP_OLATA     0x14
#define MCP_OLATB     0x15

#define MCP_HW_ADDR_0 0x00  // A2=0, A1=0, A0=1 = MCP for Encoders
#define MCP_HW_ADDR_1 0x01  // A2=0, A1=0, A0=1 = MCP for Switches

#define PORT_A_OFFSET 0
#define PORT_B_OFFSET 8

void MCP23S17_Init(DUALMCP * mcp);

uint8_t MCP23S17_TransmitRegister(DUALMCP * mcp, uint8_t hw_addr, uint8_t reg, uint8_t data, uint8_t write);

uint8_t MCP23S17_ReadRegister(DUALMCP * mcp, uint8_t hw_addr, uint8_t reg);

uint8_t MCP23S17_WriteRegister(DUALMCP * mcp, uint8_t hw_addr, uint8_t reg, uint8_t data);

void ProcessEncoderStates(DUALMCP * mcp);

void ProcessButtonStates(DUALMCP * mcp);

void UpdateEncoderPinStates(DUALMCP * mcp, uint8_t gpioa, uint8_t gpiob);

void ReadEncoders(DUALMCP * mcp);

void ReadButtons(DUALMCP * mcp);

uint8_t ReadButtonsDMA(DUALMCP * mcp);

void ReadEncoders(DUALMCP * mcp);

uint8_t ReadEncodersDMA(DUALMCP * mcp);

void MCP_DMA_Complete(DUALMCP * mcp);

#endif /* INC_DRIVERS_DUALmcp_H_ */
