#include "mcp.h"
#include "main.h"
#include <stdint.h>

static BTNENC btnenc;

/* PRIVATE */
uint8_t mcp_transmit_register(uint8_t hw_addr, uint8_t reg, uint8_t data, uint8_t write);

uint8_t mcp_read_register(uint8_t hw_addr, uint8_t reg);

uint8_t mcp_write_register(uint8_t hw_addr, uint8_t reg, uint8_t data);

uint8_t mcp_read_buttons();

uint8_t mcp_read_encoders();

void mcp_dma_complete();

#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01

#define MCP_IOPOLA 0x02
#define MCP_IOPOLB 0x03

#define MCP_GPINTENA 0x04
#define MCP_GPINTENB 0x05

#define MCP_DEFVALA 0x06
#define MCP_DEFVALB 0x07

#define MCP_INTCONA 0x08
#define MCP_INTCONB 0x09

#define MCP_IOCONA 0x0A
#define MCP_IOCONB 0x0B

#define MCP_GPPUA 0x0C
#define MCP_GPPUB 0x0D

#define MCP_INTFA 0x0E
#define MCP_INTFB 0x0F

#define MCP_INTCAPA 0x10
#define MCP_INTCAPB 0x11

#define MCP_GPIOA 0x12
#define MCP_GPIOB 0x13

#define MCP_OLATA 0x14
#define MCP_OLATB 0x15

#define MCP_HW_ADDR_0 0x00 // A2=0, A1=0, A0=1 = MCP for Encoders
#define MCP_HW_ADDR_1 0x01 // A2=0, A1=0, A0=1 = MCP for Switches

int8_t mcp_read()
{
  if (btnenc.spi_dma_state == 0)
  {
    mcp_read_buttons();
    return 1;
  }
  return 0;
}

uint8_t get_btn_state(uint8_t buttonIndex) { return btnenc.dec.button_state[buttonIndex]; }

int16_t get_enc_state(uint8_t encoderIndex) { return btnenc.dec.enc_position[encoderIndex]; }

void mcp_handle_txrx_complete(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == btnenc.spiHandle->Instance)
  {
    mcp_dma_complete();
  }
}

uint8_t mcp_transmit_register(uint8_t hw_addr, uint8_t reg, uint8_t data, uint8_t write)
{
  uint8_t opcode    = 0x40 | ((hw_addr & 0x07) << 1) | (write ? 0 : 1);
  uint8_t tx_buf[3] = {opcode, reg, data};
  uint8_t rx_buf[3] = {0};

  // MCP_CS_LOW();
  HAL_GPIO_WritePin(btnenc.csPortHandle, btnenc.csPin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(btnenc.spiHandle, tx_buf, rx_buf, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(btnenc.csPortHandle, btnenc.csPin, GPIO_PIN_SET);
  // MCP_CS_HIGH();

  return rx_buf[2];
}

uint8_t mcp_read_register(uint8_t hw_addr, uint8_t reg) { return mcp_transmit_register(hw_addr, reg, 0x00, 0); }

// Write version
uint8_t mcp_write_register(uint8_t hw_addr, uint8_t reg, uint8_t data) { return mcp_transmit_register(hw_addr, reg, data, 1); }

void mcu_read_buttons()
{
  btnenc.dec.button_state[N_ENCODERS * 2 + 0] = HAL_GPIO_ReadPin(IN_BTN_MCU1_GPIO_Port, IN_BTN_MCU1_Pin);
  btnenc.dec.button_state[N_ENCODERS * 2 + 1] = HAL_GPIO_ReadPin(IN_BTN_MCU2_GPIO_Port, IN_BTN_MCU2_Pin);
  btnenc.dec.button_state[N_ENCODERS * 2 + 2] = HAL_GPIO_ReadPin(IN_BTN_MCU3_GPIO_Port, IN_BTN_MCU3_Pin);
  btnenc.dec.button_state[N_ENCODERS * 2 + 3] = HAL_GPIO_ReadPin(IN_BTN_MCU4_GPIO_Port, IN_BTN_MCU4_Pin);
  btnenc.dec.button_state[N_ENCODERS * 2 + 4] = HAL_GPIO_ReadPin(IN_BTN_MCU5_GPIO_Port, IN_BTN_MCU5_Pin);
  btnenc.dec.button_state[N_ENCODERS * 2 + 5] = HAL_GPIO_ReadPin(MENU_BTN_2_GPIO_Port, MENU_BTN_2_Pin);
  btnenc.dec.button_state[N_ENCODERS * 2 + 6] = HAL_GPIO_ReadPin(MENU_BTN_BOOT_GPIO_Port, MENU_BTN_BOOT_Pin);
  btnenc.dec.button_state[N_ENCODERS * 2 + 7] = HAL_GPIO_ReadPin(MENU_BTN_3_GPIO_Port, MENU_BTN_3_Pin);
}

uint8_t mcp_read_buttons()
{
  btnenc.spi_dma_state = 2;
  HAL_GPIO_WritePin(btnenc.csPortHandle, btnenc.csPin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive_DMA(btnenc.spiHandle, &(btnenc.tx_buf[4]), &(btnenc.rx_buf[4]), 4) == HAL_OK)
  {
    return 1;
  }

  HAL_GPIO_WritePin(btnenc.csPortHandle, btnenc.csPin, GPIO_PIN_SET);
  btnenc.spi_dma_state = 0;
  return 0;
}

uint8_t mcp_read_encoders()
{
  btnenc.spi_dma_state = 1;
  HAL_GPIO_WritePin(btnenc.csPortHandle, btnenc.csPin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive_DMA(btnenc.spiHandle, &(btnenc.tx_buf[0]), &(btnenc.rx_buf[0]), 4) == HAL_OK)
  {
    return 1;
  }

  HAL_GPIO_WritePin(btnenc.csPortHandle, btnenc.csPin, GPIO_PIN_SET);
  btnenc.spi_dma_state = 0;
  return 0;
}

void mcp_dma_complete()
{
  HAL_GPIO_WritePin(btnenc.csPortHandle, btnenc.csPin, GPIO_PIN_SET);

  if (btnenc.spi_dma_state == 2)
  {
    mcp_decode_buttons(&btnenc.dec, btnenc.rx_buf[6], btnenc.rx_buf[7]);
    mcp_read_encoders();
    return;
  }

  if (btnenc.spi_dma_state == 1)
  {
    mcp_decode_encoders(&btnenc.dec, btnenc.rx_buf[2], btnenc.rx_buf[3]);
  }

  btnenc.spi_dma_state = 0;
}

void mcp_init(SPI_HandleTypeDef* spi)
{

  btnenc.spiHandle       = spi;
  btnenc.csPortHandle    = OUT_MCP_CS_GPIO_Port;
  btnenc.csPin           = OUT_MCP_CS_Pin;
  btnenc.resetPortHandle = OUT_MCP_RESET_GPIO_Port;
  btnenc.resetPin        = OUT_MCP_RESET_Pin;

  btnenc.spi_dma_state = 0;

  mcp_decode_init(&btnenc.dec);

  btnenc.tx_buf[0] = 0x40 | ((MCP_HW_ADDR_0 & 0x07) << 1) | 1;
  btnenc.tx_buf[1] = MCP_GPIOA;
  btnenc.tx_buf[2] = 0;
  btnenc.tx_buf[3] = 0;
  btnenc.tx_buf[4] = 0x40 | ((MCP_HW_ADDR_1 & 0x07) << 1) | 1;
  btnenc.tx_buf[5] = MCP_GPIOA;
  btnenc.tx_buf[6] = 0;
  btnenc.tx_buf[7] = 0;
  btnenc.rx_buf[0] = 0;
  btnenc.rx_buf[1] = 0;
  btnenc.rx_buf[2] = 0;
  btnenc.rx_buf[3] = 0;
  btnenc.rx_buf[4] = 0;
  btnenc.rx_buf[5] = 0;
  btnenc.rx_buf[6] = 0;
  btnenc.rx_buf[7] = 0;

  // Reset MCP
  HAL_GPIO_WritePin(btnenc.resetPortHandle, btnenc.resetPin, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(btnenc.resetPortHandle, btnenc.resetPin, GPIO_PIN_SET);
  HAL_Delay(5);

  // Set IOCON bits
  // 0    XXXX    0
  // 1	INTPOL	1	Active-high
  // 2	ODR	    0	Push/pull
  // 3	HAEN	1	Enable addressing
  // 4    DISSLW  0   Slew rate enabled (1 = disabled)
  // 5    SEQOP   1   Sequential operation disabled
  // 6	MIRROR	1	Mirror interrupt lines (we're only using A)
  // 7    BANK    0   Registers are in the same bank
  mcp_write_register(MCP_HW_ADDR_1, MCP_IOCONA, 0b01101010); // IOCON 0b01101010 - 0x6A
  mcp_write_register(MCP_HW_ADDR_1, MCP_IOCONB, 0b01101010); // IOCON 0b01101010 - 0x6A

  mcp_write_register(MCP_HW_ADDR_0, MCP_IOCONA, 0b01101010);
  mcp_write_register(MCP_HW_ADDR_0, MCP_IOCONB, 0b01101010);

  // Button/Switch MCP_HW_ADDR_1

  // Set both ports as inputs, no pullups (have external ones)
  mcp_write_register(MCP_HW_ADDR_1, MCP_IODIRA, 0xFF);
  mcp_write_register(MCP_HW_ADDR_1, MCP_IODIRB, 0xFF);
  mcp_write_register(MCP_HW_ADDR_1, MCP_GPPUA, 0x00);
  mcp_write_register(MCP_HW_ADDR_1, MCP_GPPUB, 0x00);

  // Disable interrupt
  mcp_write_register(MCP_HW_ADDR_1, MCP_GPINTENA, 0x00);
  mcp_write_register(MCP_HW_ADDR_1, MCP_GPINTENB, 0x00);

  // Disable interrupts, but all switches are pulled low by default:
  mcp_write_register(MCP_HW_ADDR_1, MCP_DEFVALA, 0x00);
  mcp_write_register(MCP_HW_ADDR_1, MCP_DEFVALB, 0x00);

  // Encoder MCP_HW_ADDR_0

  // Invert Logic (encoder pulses = HIGH)
  mcp_write_register(MCP_HW_ADDR_1, MCP_IOPOLA, 0x00);
  mcp_write_register(MCP_HW_ADDR_1, MCP_IOPOLB, 0x00);

  // Set both ports as inputs, no pullups (have external ones)
  mcp_write_register(MCP_HW_ADDR_0, MCP_IODIRA, 0xFF);
  mcp_write_register(MCP_HW_ADDR_0, MCP_IODIRB, 0xFF);
  mcp_write_register(MCP_HW_ADDR_0, MCP_GPPUA, 0x00);
  mcp_write_register(MCP_HW_ADDR_0, MCP_GPPUB, 0x00);

  // Enable interrupt-on-change for all pins
  mcp_write_register(MCP_HW_ADDR_0, MCP_GPINTENA, 0xFF);
  mcp_write_register(MCP_HW_ADDR_0, MCP_GPINTENB, 0xFF);

  // Trigger interrupt on *any change*, not compared to DEFVAL
  mcp_write_register(MCP_HW_ADDR_0, MCP_INTCONA, 0x00);
  mcp_write_register(MCP_HW_ADDR_0, MCP_INTCONB, 0x00);

  // Not used, but default value is HIGH for encoders
  mcp_write_register(MCP_HW_ADDR_0, MCP_DEFVALA, 0xFF);
  mcp_write_register(MCP_HW_ADDR_0, MCP_DEFVALB, 0xFF);
}
