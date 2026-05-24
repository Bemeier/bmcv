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

void enc_process_states();

void enc_update_pin_states(uint8_t gpioa, uint8_t gpiob);

void btn_process_states();

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

#define PORT_A_OFFSET 0
#define PORT_B_OFFSET 8

int8_t mcp_read()
{
    if (btnenc.spi_dma_state == 0)
    {
        mcp_read_buttons();
        return 1;
    }
    return 0;
}

uint8_t get_btn_state(uint8_t buttonIndex) { return btnenc.button_state[buttonIndex]; }

int16_t get_enc_state(uint8_t encoderIndex) { return btnenc.enc_position_state[encoderIndex]; }

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

void btn_process_states()
{
    for (uint8_t b = 0; b < N_ENCODERS; b++)
    {
        uint8_t pin = btnenc.enc_button_pins[b];
        if (pin < 8)
        {
            btnenc.button_state[b] = (btnenc.gpioa_state >> pin) & 0x01;
        }
        else
        {
            btnenc.button_state[b] = (btnenc.gpiob_state >> (pin - 8)) & 0x01;
        }
    }

    for (uint8_t b = 0; b < N_ENCODERS; b++)
    {
        uint8_t pin = btnenc.bottom_button_pins[b];
        if (pin < 8)
        {
            btnenc.button_state[N_ENCODERS + b] = (btnenc.gpioa_state >> pin) & 0x01;
        }
        else
        {
            btnenc.button_state[N_ENCODERS + b] = (btnenc.gpiob_state >> (pin - 8)) & 0x01;
        }
    }
}

void enc_process_states()
{
    const int8_t encoder_table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    for (int i = 0; i < N_ENCODERS; i++)
    {
        uint8_t a_old = (btnenc.a_state_prev >> i) & 0x01;
        uint8_t b_old = (btnenc.b_state_prev >> i) & 0x01;
        uint8_t a_new = (btnenc.a_state >> i) & 0x01;
        uint8_t b_new = (btnenc.b_state >> i) & 0x01;

        uint8_t prev_state = (a_old << 1) | b_old;
        uint8_t curr_state = (a_new << 1) | b_new;

        uint8_t index = (prev_state << 2) | curr_state;
        int8_t delta  = encoder_table[index];

        if (curr_state == 0b00)
        {
            btnenc.enc_position_state[i] += delta;
        }
        // enc_position_state[i] += delta;
    }

    btnenc.a_state_prev = btnenc.a_state;
    btnenc.b_state_prev = btnenc.b_state;
}

void enc_update_pin_states(uint8_t gpioa, uint8_t gpiob)
{
    uint8_t new_a_state = 0;
    uint8_t new_b_state = 0;

    for (int i = 0; i < N_ENCODERS; i++)
    {
        // Read pin A
        uint8_t pin_a       = btnenc.enc_pins_a[i];
        uint8_t pin_a_state = 0;

        if (pin_a < 8)
        {
            pin_a_state = (gpioa >> pin_a) & 0x01;
        }
        else
        {
            pin_a_state = (gpiob >> (pin_a - 8)) & 0x01;
        }

        // Read pin B
        uint8_t pin_b       = btnenc.enc_pins_b[i];
        uint8_t pin_b_state = 0;

        if (pin_b < 8)
        {
            pin_b_state = (gpioa >> pin_b) & 0x01;
        }
        else
        {
            pin_b_state = (gpiob >> (pin_b - 8)) & 0x01;
        }
        // Set bit i in result states
        new_a_state |= (pin_a_state << i);
        new_b_state |= (pin_b_state << i);
    }

    btnenc.a_state = new_a_state;
    btnenc.b_state = new_b_state;
}

void mcu_read_buttons()
{
    btnenc.button_state[N_ENCODERS * 2 + 0] = HAL_GPIO_ReadPin(IN_BTN_MCU1_GPIO_Port, IN_BTN_MCU1_Pin);
    btnenc.button_state[N_ENCODERS * 2 + 1] = HAL_GPIO_ReadPin(IN_BTN_MCU2_GPIO_Port, IN_BTN_MCU2_Pin);
    btnenc.button_state[N_ENCODERS * 2 + 2] = HAL_GPIO_ReadPin(IN_BTN_MCU3_GPIO_Port, IN_BTN_MCU3_Pin);
    btnenc.button_state[N_ENCODERS * 2 + 3] = HAL_GPIO_ReadPin(IN_BTN_MCU4_GPIO_Port, IN_BTN_MCU4_Pin);
    btnenc.button_state[N_ENCODERS * 2 + 4] = HAL_GPIO_ReadPin(IN_BTN_MCU5_GPIO_Port, IN_BTN_MCU5_Pin);
    btnenc.button_state[N_ENCODERS * 2 + 5] = HAL_GPIO_ReadPin(MENU_BTN_2_GPIO_Port, MENU_BTN_2_Pin);
    btnenc.button_state[N_ENCODERS * 2 + 6] = HAL_GPIO_ReadPin(MENU_BTN_BOOT_GPIO_Port, MENU_BTN_BOOT_Pin);
    btnenc.button_state[N_ENCODERS * 2 + 7] = HAL_GPIO_ReadPin(MENU_BTN_3_GPIO_Port, MENU_BTN_3_Pin);
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
        btnenc.gpioa_state = btnenc.rx_buf[6];
        btnenc.gpiob_state = btnenc.rx_buf[7];
        btn_process_states();
        mcp_read_encoders();
        return;
    }

    if (btnenc.spi_dma_state == 1)
    {
        enc_update_pin_states(btnenc.rx_buf[2], btnenc.rx_buf[3]);
        enc_process_states();
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

    btnenc.enc_button_pins[0] = PORT_A_OFFSET + 4; // Encoder 1
    btnenc.enc_button_pins[1] = PORT_A_OFFSET + 6; // Encoder 2
    btnenc.enc_button_pins[2] = PORT_A_OFFSET + 7; // Encoder 3
    btnenc.enc_button_pins[3] = PORT_B_OFFSET + 0; // Encoder 4
    btnenc.enc_button_pins[4] = PORT_A_OFFSET + 5; // Encoder 5 <- correct
    btnenc.enc_button_pins[5] = PORT_B_OFFSET + 1; // Encoder 6
    btnenc.enc_button_pins[6] = PORT_B_OFFSET + 2; // Encoder 7
    btnenc.enc_button_pins[7] = PORT_B_OFFSET + 3; // Encoder 8 <-correct

    btnenc.bottom_button_pins[0] = PORT_A_OFFSET + 0;
    btnenc.bottom_button_pins[1] = PORT_A_OFFSET + 1;
    btnenc.bottom_button_pins[2] = PORT_A_OFFSET + 2;
    btnenc.bottom_button_pins[3] = PORT_A_OFFSET + 3;

    btnenc.bottom_button_pins[4] = PORT_B_OFFSET + 4;
    btnenc.bottom_button_pins[5] = PORT_B_OFFSET + 5;
    btnenc.bottom_button_pins[6] = PORT_B_OFFSET + 6;
    btnenc.bottom_button_pins[7] = PORT_B_OFFSET + 7;
    // Offset: 0 1 2 3 4 5  6  7
    // Value:  0 1 2 4 6 8 16 32

    btnenc.enc_pins_a[0] = PORT_B_OFFSET + 2; // Encoder 1 A
    btnenc.enc_pins_a[1] = PORT_B_OFFSET + 1; // Encoder 2 A
    btnenc.enc_pins_a[2] = PORT_A_OFFSET + 7; // Encoder 3 A
    btnenc.enc_pins_a[3] = PORT_A_OFFSET + 5; // Encoder 4 A
    btnenc.enc_pins_a[4] = PORT_A_OFFSET + 2; // Encoder 5 A
    btnenc.enc_pins_a[5] = PORT_A_OFFSET + 0; // Encoder 6 A
    btnenc.enc_pins_a[6] = PORT_B_OFFSET + 4; // Encoder 7 A
    btnenc.enc_pins_a[7] = PORT_B_OFFSET + 6; // Encoder 8 A

    btnenc.enc_pins_b[0] = PORT_B_OFFSET + 3; // Encoder 1 B
    btnenc.enc_pins_b[1] = PORT_B_OFFSET + 0; // Encoder 2 B
    btnenc.enc_pins_b[2] = PORT_A_OFFSET + 6; // Encoder 3 B
    btnenc.enc_pins_b[3] = PORT_A_OFFSET + 4; // Encoder 4 B
    btnenc.enc_pins_b[4] = PORT_A_OFFSET + 3; // Encoder 5 B
    btnenc.enc_pins_b[5] = PORT_A_OFFSET + 1; // Encoder 6 B
    btnenc.enc_pins_b[6] = PORT_B_OFFSET + 5; // Encoder 7 B
    btnenc.enc_pins_b[7] = PORT_B_OFFSET + 7; // Encoder 8 B

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
