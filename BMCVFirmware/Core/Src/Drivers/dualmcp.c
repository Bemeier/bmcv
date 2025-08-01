#include "dualmcp.h"
#include "main.h"

static DUALMCP mcp;


DUALMCP * mcp_instance(void)
{
    return &mcp;
}

uint8_t mcp_transmit_register(uint8_t hw_addr, uint8_t reg, uint8_t data, uint8_t write)
{
    uint8_t opcode = 0x40 | ((hw_addr & 0x07) << 1) | (write ? 0 : 1);
    uint8_t tx_buf[3] = {opcode, reg, data};
    uint8_t rx_buf[3] = {0};

    //MCP_CS_LOW();
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mcp.spiHandle, tx_buf, rx_buf, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_SET);
    //MCP_CS_HIGH();

    return rx_buf[2];
}

uint8_t mcp_read_register(uint8_t hw_addr, uint8_t reg)
{
    return mcp_transmit_register(hw_addr, reg, 0x00, 0);
}

// Write version
uint8_t mcp_write_register(uint8_t hw_addr, uint8_t reg, uint8_t data)
{
    return mcp_transmit_register(hw_addr, reg, data, 1);
}

void ProcessButtonStates()
{
    for (uint8_t b = 0; b < N_ENCODERS; b++) {
        uint8_t pin = mcp.enc_button_pins[b];
        if (pin < 8) {
            mcp.button_state[b] = (mcp.gpioa_state >> pin) & 0x01;
        } else {
            mcp.button_state[b] = (mcp.gpiob_state >> (pin - 8)) & 0x01;
        }
    }

    for (uint8_t b = 0; b < N_ENCODERS; b++) {
        uint8_t pin = mcp.bottom_button_pins[b];
        if (pin < 8) {
            mcp.button_state[N_ENCODERS+b] = (mcp.gpioa_state >> pin) & 0x01;
        } else {
            mcp.button_state[N_ENCODERS+b] = (mcp.gpiob_state >> (pin - 8)) & 0x01;
        }
    }
}

void ProcessEncoderStates()
{
    const int8_t encoder_table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    for (int i = 0; i < N_ENCODERS; i++) {
        uint8_t a_old = (mcp.a_state_prev >> i) & 0x01;
        uint8_t b_old = (mcp.b_state_prev >> i) & 0x01;
        uint8_t a_new = (mcp.a_state >> i) & 0x01;
        uint8_t b_new = (mcp.b_state >> i) & 0x01;

        uint8_t prev_state = (a_old << 1) | b_old;
        uint8_t curr_state = (a_new << 1) | b_new;

        uint8_t index = (prev_state << 2) | curr_state;
        int8_t delta = encoder_table[index];

        if (curr_state == 0b00) {
            mcp.enc_position_state[i] += delta;
        }
        //enc_position_state[i] += delta;
    }

	mcp.a_state_prev = mcp.a_state;
	mcp.b_state_prev = mcp.b_state;
}

void UpdateEncoderPinStates(uint8_t gpioa, uint8_t gpiob)
{
    uint8_t new_a_state = 0;
    uint8_t new_b_state = 0;

    for (int i = 0; i < N_ENCODERS; i++) {
        // Read pin A
        uint8_t pin_a = mcp.enc_pins_a[i];
        uint8_t pin_a_state = 0;

        if (pin_a < 8) {
            pin_a_state = (gpioa >> pin_a) & 0x01;
        } else {
            pin_a_state = (gpiob >> (pin_a - 8)) & 0x01;
        }

        // Read pin B
        uint8_t pin_b = mcp.enc_pins_b[i];
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

    mcp.a_state = new_a_state;
    mcp.b_state = new_b_state;
}

void ReadEncoders() {
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mcp.spiHandle, &(mcp.tx_buf[0]), &(mcp.rx_buf[0]), 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_SET);

    int8_t enc_gpioa_state = mcp.rx_buf[2];
    int8_t enc_gpiob_state = mcp.rx_buf[3];

	UpdateEncoderPinStates(enc_gpioa_state, enc_gpiob_state);
	ProcessEncoderStates();
}

void ReadButtons() {
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mcp.spiHandle, &(mcp.tx_buf[4]), &(mcp.rx_buf[4]), 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_SET);

    mcp.gpioa_state = mcp.rx_buf[6];
    mcp.gpiob_state = mcp.rx_buf[7];
}

uint8_t ReadButtonsDMA() {
	mcp.spi_dma_state = 2;
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(mcp.spiHandle, &(mcp.tx_buf[4]), &(mcp.rx_buf[4]), 4) == HAL_OK) {
    	return 1;
    }

    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_SET);
	mcp.spi_dma_state = 0;
    return 0;
}

uint8_t ReadEncodersDMA() {
	mcp.spi_dma_state = 1;
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(mcp.spiHandle, &(mcp.tx_buf[0]), &(mcp.rx_buf[0]), 4) == HAL_OK) {
    	return 1;
    }

    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_SET);
	mcp.spi_dma_state = 0;
    return 0;
}

void mcp_dma_complete() {
    HAL_GPIO_WritePin(mcp.csPortHandle, mcp.csPin, GPIO_PIN_SET);

    if (mcp.spi_dma_state == 2) {
        mcp.gpioa_state = mcp.rx_buf[6];
        mcp.gpiob_state = mcp.rx_buf[7];
        ProcessButtonStates();
    	ReadEncodersDMA();
    	return;
    }

    if (mcp.spi_dma_state == 1) {
		UpdateEncoderPinStates(mcp.rx_buf[2], mcp.rx_buf[3]);
		ProcessEncoderStates();
    }

    mcp.spi_dma_state = 0;
}


void mcp_init(SPI_HandleTypeDef * spi) {

    mcp.spiHandle = spi;
    mcp.csPortHandle = OUT_MCP_CS_GPIO_Port;
    mcp.csPin = OUT_MCP_CS_Pin;
    mcp.resetPortHandle = OUT_MCP_RESET_GPIO_Port;
    mcp.resetPin = OUT_MCP_RESET_Pin;

	mcp.spi_dma_state = 0;

	mcp.enc_button_pins[0] = PORT_A_OFFSET + 4;  // Encoder 1
	mcp.enc_button_pins[1] = PORT_A_OFFSET + 6;  // Encoder 2
	mcp.enc_button_pins[2] = PORT_A_OFFSET + 7;  // Encoder 3
	mcp.enc_button_pins[3] = PORT_B_OFFSET + 0;  // Encoder 4
	mcp.enc_button_pins[4] = PORT_A_OFFSET + 5;  // Encoder 5 <- correct
	mcp.enc_button_pins[5] = PORT_B_OFFSET + 1;  // Encoder 6
	mcp.enc_button_pins[6] = PORT_B_OFFSET + 2;  // Encoder 7
	mcp.enc_button_pins[7] = PORT_B_OFFSET + 3;  // Encoder 8 <-correct

    mcp.bottom_button_pins[0] = PORT_A_OFFSET + 0;
    mcp.bottom_button_pins[1] = PORT_A_OFFSET + 1;
    mcp.bottom_button_pins[2] = PORT_A_OFFSET + 2;
    mcp.bottom_button_pins[3] = PORT_A_OFFSET + 3;

    mcp.bottom_button_pins[4] = PORT_B_OFFSET + 4;
    mcp.bottom_button_pins[5] = PORT_B_OFFSET + 5;
    mcp.bottom_button_pins[6] = PORT_B_OFFSET + 6;
    mcp.bottom_button_pins[7] = PORT_B_OFFSET + 7;
    // Offset: 0 1 2 3 4 5  6  7
    // Value:  0 1 2 4 6 8 16 32

	mcp.enc_pins_a[0] = PORT_B_OFFSET + 2;  // Encoder 1 A
	mcp.enc_pins_a[1] = PORT_B_OFFSET + 1;  // Encoder 2 A
	mcp.enc_pins_a[2] = PORT_A_OFFSET + 7;  // Encoder 3 A
	mcp.enc_pins_a[3] = PORT_A_OFFSET + 5;  // Encoder 4 A
	mcp.enc_pins_a[4] = PORT_A_OFFSET + 2;  // Encoder 5 A
	mcp.enc_pins_a[5] = PORT_A_OFFSET + 0;  // Encoder 6 A
	mcp.enc_pins_a[6] = PORT_B_OFFSET + 4;  // Encoder 7 A
	mcp.enc_pins_a[7] = PORT_B_OFFSET + 6;  // Encoder 8 A

	mcp.enc_pins_b[0] = PORT_B_OFFSET + 3,  // Encoder 1 B
	mcp.enc_pins_b[1] = PORT_B_OFFSET + 0;  // Encoder 2 B
	mcp.enc_pins_b[2] = PORT_A_OFFSET + 6;  // Encoder 3 B
	mcp.enc_pins_b[3] = PORT_A_OFFSET + 4;  // Encoder 4 B
	mcp.enc_pins_b[4] = PORT_A_OFFSET + 3;  // Encoder 5 B
	mcp.enc_pins_b[5] = PORT_A_OFFSET + 1;  // Encoder 6 B
	mcp.enc_pins_b[6] = PORT_B_OFFSET + 5;  // Encoder 7 B
	mcp.enc_pins_b[7] = PORT_B_OFFSET + 7;  // Encoder 8 B

	mcp.tx_buf[0] = 0x40 | ((MCP_HW_ADDR_0 & 0x07) << 1) | 1;
	mcp.tx_buf[1] = MCP_GPIOA;
	mcp.tx_buf[2] = 0;
	mcp.tx_buf[3] = 0;
	mcp.tx_buf[4] = 0x40 | ((MCP_HW_ADDR_1 & 0x07) << 1) | 1;
	mcp.tx_buf[5] = MCP_GPIOA;
	mcp.tx_buf[6] = 0;
	mcp.tx_buf[7] = 0;
	mcp.rx_buf[0] = 0;
	mcp.rx_buf[1] = 0;
	mcp.rx_buf[2] = 0;
	mcp.rx_buf[3] = 0;
	mcp.rx_buf[4] = 0;
	mcp.rx_buf[5] = 0;
	mcp.rx_buf[6] = 0;
	mcp.rx_buf[7] = 0;

    // Reset MCP
    HAL_GPIO_WritePin(mcp.resetPortHandle, mcp.resetPin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(mcp.resetPortHandle, mcp.resetPin, GPIO_PIN_SET);
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
    mcp_write_register(MCP_HW_ADDR_1, MCP_IOCONA, 0b01101010);  // IOCON 0b01101010 - 0x6A
    mcp_write_register(MCP_HW_ADDR_1, MCP_IOCONB, 0b01101010);  // IOCON 0b01101010 - 0x6A

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
