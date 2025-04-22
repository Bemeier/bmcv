#include "dualmpc.h"

uint8_t MCP23S17_TransmitRegister(DUALMPC * mpc, uint8_t hw_addr, uint8_t reg, uint8_t data, uint8_t write)
{
    uint8_t opcode = 0x40 | ((hw_addr & 0x07) << 1) | (write ? 0 : 1);
    uint8_t tx_buf[3] = {opcode, reg, data};
    uint8_t rx_buf[3] = {0};

    //MCP_CS_LOW();
    HAL_GPIO_WritePin(mpc->csPortHandle, mpc->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mpc->spiHandle, tx_buf, rx_buf, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(mpc->csPortHandle, mpc->csPin, GPIO_PIN_SET);
    //MCP_CS_HIGH();

    return rx_buf[2];
}

uint8_t MCP23S17_ReadRegister(DUALMPC * mpc, uint8_t hw_addr, uint8_t reg)
{
    return MCP23S17_TransmitRegister(mpc, hw_addr, reg, 0x00, 0);
}

// Write version
uint8_t MCP23S17_WriteRegister(DUALMPC * mpc, uint8_t hw_addr, uint8_t reg, uint8_t data)
{
    return MCP23S17_TransmitRegister(mpc, hw_addr, reg, data, 1);
}

void ProcessEncoderStates(DUALMPC * mpc)
{
    const int8_t encoder_table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    for (int i = 0; i < N_ENCODERS; i++) {
        uint8_t a_old = (mpc->a_state_prev >> i) & 0x01;
        uint8_t b_old = (mpc->b_state_prev >> i) & 0x01;
        uint8_t a_new = (mpc->a_state >> i) & 0x01;
        uint8_t b_new = (mpc->b_state >> i) & 0x01;

        uint8_t prev_state = (a_old << 1) | b_old;
        uint8_t curr_state = (a_new << 1) | b_new;

        uint8_t index = (prev_state << 2) | curr_state;
        int8_t delta = encoder_table[index];

        if (curr_state == 0b00) {
            mpc->enc_position_state[i] += delta;
        }
        //enc_position_state[i] += delta;
    }

	mpc->a_state_prev = mpc->a_state;
	mpc->b_state_prev = mpc->b_state;
}

void UpdateEncoderPinStates(DUALMPC * mpc, uint8_t gpioa, uint8_t gpiob)
{
    uint8_t new_a_state = 0;
    uint8_t new_b_state = 0;

    for (int i = 0; i < N_ENCODERS; i++) {
        // Read pin A
        uint8_t pin_a = mpc->enc_pins_a[i];
        uint8_t pin_a_state = 0;

        if (pin_a < 8) {
            pin_a_state = (gpioa >> pin_a) & 0x01;
        } else {
            pin_a_state = (gpiob >> (pin_a - 8)) & 0x01;
        }

        // Read pin B
        uint8_t pin_b = mpc->enc_pins_b[i];
        uint8_t pin_b_state = 0;

        if (pin_b < 8) {
            pin_b_state = (gpioa >> pin_b) & 0x01;
        } else {
            pin_b_state = (gpiob >> (pin_b - 8)) & 0x01;
        }
        // Set bit i in result states
        new_a_state |= (pin_a_state << i);
        new_b_state |= (pin_b_state << i);
    }

    mpc->a_state = new_a_state;
    mpc->b_state = new_b_state;
}

void ReadEncoders(DUALMPC * mpc) {
    HAL_GPIO_WritePin(mpc->csPortHandle, mpc->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mpc->spiHandle, &(mpc->tx_buf[0]), &(mpc->rx_buf[0]), 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(mpc->csPortHandle, mpc->csPin, GPIO_PIN_SET);

    int8_t enc_gpioa_state = mpc->rx_buf[2];
    int8_t enc_gpiob_state = mpc->rx_buf[3];

	UpdateEncoderPinStates(mpc, enc_gpioa_state, enc_gpiob_state);
	ProcessEncoderStates(mpc);
}

void ReadButtons(DUALMPC * mpc) {
    HAL_GPIO_WritePin(mpc->csPortHandle, mpc->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mpc->spiHandle, &(mpc->tx_buf[4]), &(mpc->rx_buf[4]), 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(mpc->csPortHandle, mpc->csPin, GPIO_PIN_SET);

    mpc->gpioa_state = mpc->rx_buf[6];
    mpc->gpiob_state = mpc->rx_buf[7];
}


void MCP23S17_Init(DUALMPC * mpc) {
	mpc->enc_pins_a[0] = PORT_B_OFFSET + 2;  // Encoder 1 A
	mpc->enc_pins_a[1] = PORT_B_OFFSET + 1;  // Encoder 2 A
	mpc->enc_pins_a[2] = PORT_A_OFFSET + 7;  // Encoder 3 A
	mpc->enc_pins_a[3] = PORT_A_OFFSET + 5;  // Encoder 4 A
	mpc->enc_pins_a[4] = PORT_A_OFFSET + 2;  // Encoder 5 A
	mpc->enc_pins_a[5] = PORT_A_OFFSET + 0;  // Encoder 6 A
	mpc->enc_pins_a[6] = PORT_B_OFFSET + 4;  // Encoder 7 A
	mpc->enc_pins_a[7] = PORT_B_OFFSET + 6;  // Encoder 8 A

	mpc->enc_pins_b[0] = PORT_B_OFFSET + 3,  // Encoder 1 B
	mpc->enc_pins_b[1] = PORT_B_OFFSET + 0;  // Encoder 2 B
	mpc->enc_pins_b[2] = PORT_A_OFFSET + 6;  // Encoder 3 B
	mpc->enc_pins_b[3] = PORT_A_OFFSET + 4;  // Encoder 4 B
	mpc->enc_pins_b[4] = PORT_A_OFFSET + 3;  // Encoder 5 B
	mpc->enc_pins_b[5] = PORT_A_OFFSET + 1;  // Encoder 6 B
	mpc->enc_pins_b[6] = PORT_B_OFFSET + 5;  // Encoder 7 B
	mpc->enc_pins_b[7] = PORT_B_OFFSET + 7;  // Encoder 8 B

	mpc->tx_buf[0] = 0x40 | ((MCP_HW_ADDR_0 & 0x07) << 1) | 1;
	mpc->tx_buf[1] = MCP_GPIOA;
	mpc->tx_buf[2] = 0;
	mpc->tx_buf[3] = 0;
	mpc->tx_buf[4] = 0x40 | ((MCP_HW_ADDR_1 & 0x07) << 1) | 1;
	mpc->tx_buf[5] = MCP_GPIOA;
	mpc->tx_buf[6] = 0;
	mpc->tx_buf[7] = 0;
	mpc->rx_buf[0] = 0;
	mpc->rx_buf[1] = 0;
	mpc->rx_buf[2] = 0;
	mpc->rx_buf[3] = 0;
	mpc->rx_buf[4] = 0;
	mpc->rx_buf[5] = 0;
	mpc->rx_buf[6] = 0;
	mpc->rx_buf[7] = 0;

    // Reset MCP
    HAL_GPIO_WritePin(mpc->resetPortHandle, mpc->resetPin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(mpc->resetPortHandle, mpc->resetPin, GPIO_PIN_SET);
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
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_IOCONA, 0b01101010);  // IOCON 0b01101010 - 0x6A
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_IOCONB, 0b01101010);  // IOCON 0b01101010 - 0x6A

    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_IOCONA, 0b01101010);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_IOCONB, 0b01101010);

    // Button/Switch MCP_HW_ADDR_1

    // Set both ports as inputs, no pullups (have external ones)
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_IODIRA, 0xFF);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_IODIRB, 0xFF);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_GPPUA, 0x00);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_GPPUB, 0x00);

    // Disable interrupt
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_GPINTENA, 0x00);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_GPINTENB, 0x00);

    // Disable interrupts, but all switches are pulled low by default:
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_DEFVALA, 0x00);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_DEFVALB, 0x00);


    // Encoder MCP_HW_ADDR_0

    // Invert Logic (encoder pulses = HIGH)
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_IOPOLA, 0x00);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_1, MCP_IOPOLB, 0x00);

    // Set both ports as inputs, no pullups (have external ones)
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_IODIRA, 0xFF);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_IODIRB, 0xFF);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_GPPUA, 0x00);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_GPPUB, 0x00);

    // Enable interrupt-on-change for all pins
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_GPINTENA, 0xFF);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_GPINTENB, 0xFF);

    // Trigger interrupt on *any change*, not compared to DEFVAL
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_INTCONA, 0x00);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_INTCONB, 0x00);

    // Not used, but default value is HIGH for encoders
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_DEFVALA, 0xFF);
    MCP23S17_WriteRegister(mpc, MCP_HW_ADDR_0, MCP_DEFVALB, 0xFF);

}
