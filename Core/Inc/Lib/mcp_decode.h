#ifndef INC_LIB_MCP_DECODE_H_
#define INC_LIB_MCP_DECODE_H_

#include "hw_setup.h" // N_ENCODERS, N_BUTTONS
#include <stdint.h>

// What the panel's two MCP23S17s mean, with no SPI in it.
//
// The expanders deliver two bytes per chip - GPIOA and GPIOB - and everything
// from there to "button 6 is down" or "encoder 3 moved a detent" is bit
// shuffling and a state machine. That half used to sit inside mcp.c behind the
// HAL, which is ARM-only, so the one piece of logic every gesture test above it
// depends on was the one piece nothing could exercise. It is plain C here, on
// the core source list, and tests/test_mcp_decode.c drives it from recorded
// port bytes.
//
// mcp.c keeps the SPI transfers, the DMA state machine and the chip setup.

#define MCP_BUTTON_COUNT N_BUTTONS

// Quadrature sub-steps per detent. A panel encoder rests in the detent with
// both channels at one state and passes through all four on the way to the
// next, so a detent is a full cycle.
#define MCP_STEPS_PER_DETENT 4

typedef struct
{
  // Which expander pin each control is wired to, port A as 0..7 and port B as
  // 8..15. Filled by mcp_decode_init; this is the board's wiring, so it is the
  // one thing here that a test cannot derive for itself.
  uint8_t enc_button_pins[N_ENCODERS];
  uint8_t bottom_button_pins[N_ENCODERS];
  uint8_t enc_pins_a[N_ENCODERS];
  uint8_t enc_pins_b[N_ENCODERS];

  // One bit per encoder, gathered from wherever that encoder's A and B pins
  // happen to sit, so the state machine below can work on all eight at once.
  uint8_t a_state;
  uint8_t b_state;
  uint8_t a_state_prev;
  uint8_t b_state_prev;
  uint8_t have_prev;

  // Quadrature sub-steps not yet worth a detent. See mcp_decode_encoders.
  int8_t enc_substep[N_ENCODERS];

  volatile int16_t enc_position[N_ENCODERS];
  uint8_t button_state[MCP_BUTTON_COUNT];
} McpDecode;

// Zeroes the decode state and fills in the board's pin map.
void mcp_decode_init(McpDecode* d);

// The switch expander's two port bytes -> button_state[0 .. 2*N_ENCODERS-1].
// The last few buttons are wired straight to the MCU and are filled in by
// mcp.c, which is the only thing that can read them.
void mcp_decode_buttons(McpDecode* d, uint8_t gpioa, uint8_t gpiob);

// The encoder expander's two port bytes -> enc_position[], in detents.
void mcp_decode_encoders(McpDecode* d, uint8_t gpioa, uint8_t gpiob);

#endif /* INC_LIB_MCP_DECODE_H_ */
