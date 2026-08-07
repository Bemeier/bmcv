#include "mcp_decode.h"
#include <string.h>

#define PORT_A_OFFSET 0
#define PORT_B_OFFSET 8

// One expander pin out of the pair of port bytes. Port A is 0..7, port B 8..15,
// which is how the pin maps below are written.
static inline uint8_t pin_level(uint8_t pin, uint8_t gpioa, uint8_t gpiob)
{
  return (pin < 8) ? ((gpioa >> pin) & 0x01) : ((gpiob >> (pin - 8)) & 0x01);
}

void mcp_decode_init(McpDecode* d)
{
  memset(d, 0, sizeof(*d));

  d->enc_button_pins[0] = PORT_A_OFFSET + 4; // Encoder 1
  d->enc_button_pins[1] = PORT_A_OFFSET + 6; // Encoder 2
  d->enc_button_pins[2] = PORT_A_OFFSET + 7; // Encoder 3
  d->enc_button_pins[3] = PORT_B_OFFSET + 0; // Encoder 4
  d->enc_button_pins[4] = PORT_A_OFFSET + 5; // Encoder 5
  d->enc_button_pins[5] = PORT_B_OFFSET + 1; // Encoder 6
  d->enc_button_pins[6] = PORT_B_OFFSET + 2; // Encoder 7
  d->enc_button_pins[7] = PORT_B_OFFSET + 3; // Encoder 8

  d->bottom_button_pins[0] = PORT_A_OFFSET + 0;
  d->bottom_button_pins[1] = PORT_A_OFFSET + 1;
  d->bottom_button_pins[2] = PORT_A_OFFSET + 2;
  d->bottom_button_pins[3] = PORT_A_OFFSET + 3;
  d->bottom_button_pins[4] = PORT_B_OFFSET + 4;
  d->bottom_button_pins[5] = PORT_B_OFFSET + 5;
  d->bottom_button_pins[6] = PORT_B_OFFSET + 6;
  d->bottom_button_pins[7] = PORT_B_OFFSET + 7;

  d->enc_pins_a[0] = PORT_B_OFFSET + 2; // Encoder 1 A
  d->enc_pins_a[1] = PORT_B_OFFSET + 1; // Encoder 2 A
  d->enc_pins_a[2] = PORT_A_OFFSET + 7; // Encoder 3 A
  d->enc_pins_a[3] = PORT_A_OFFSET + 5; // Encoder 4 A
  d->enc_pins_a[4] = PORT_A_OFFSET + 2; // Encoder 5 A
  d->enc_pins_a[5] = PORT_A_OFFSET + 0; // Encoder 6 A
  d->enc_pins_a[6] = PORT_B_OFFSET + 4; // Encoder 7 A
  d->enc_pins_a[7] = PORT_B_OFFSET + 6; // Encoder 8 A

  d->enc_pins_b[0] = PORT_B_OFFSET + 3; // Encoder 1 B
  d->enc_pins_b[1] = PORT_B_OFFSET + 0; // Encoder 2 B
  d->enc_pins_b[2] = PORT_A_OFFSET + 6; // Encoder 3 B
  d->enc_pins_b[3] = PORT_A_OFFSET + 4; // Encoder 4 B
  d->enc_pins_b[4] = PORT_A_OFFSET + 3; // Encoder 5 B
  d->enc_pins_b[5] = PORT_A_OFFSET + 1; // Encoder 6 B
  d->enc_pins_b[6] = PORT_B_OFFSET + 5; // Encoder 7 B
  d->enc_pins_b[7] = PORT_B_OFFSET + 7; // Encoder 8 B
}

void mcp_decode_buttons(McpDecode* d, uint8_t gpioa, uint8_t gpiob)
{
  for (uint8_t b = 0; b < N_ENCODERS; b++)
  {
    d->button_state[b]              = pin_level(d->enc_button_pins[b], gpioa, gpiob);
    d->button_state[N_ENCODERS + b] = pin_level(d->bottom_button_pins[b], gpioa, gpiob);
  }
}

void mcp_decode_encoders(McpDecode* d, uint8_t gpioa, uint8_t gpiob)
{
  // Gray-code transition table, indexed by (previous << 2) | current, where a
  // state is (A << 1) | B. The four impossible entries - both channels moving
  // in one sample - are zero, so a missed sample costs a step rather than
  // inventing a direction.
  static const int8_t transition[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  uint8_t new_a = 0;
  uint8_t new_b = 0;

  for (uint8_t i = 0; i < N_ENCODERS; i++)
  {
    new_a |= (uint8_t) (pin_level(d->enc_pins_a[i], gpioa, gpiob) << i);
    new_b |= (uint8_t) (pin_level(d->enc_pins_b[i], gpioa, gpiob) << i);
  }

  d->a_state = new_a;
  d->b_state = new_b;

  // The first sample establishes where the encoders are; it is not movement.
  // Without this, whatever the ports happen to read at power-on decodes as a
  // transition out of 00 and the module boots with the encoders already turned.
  if (!d->have_prev)
  {
    d->a_state_prev = d->a_state;
    d->b_state_prev = d->b_state;
    d->have_prev    = 1;
    return;
  }

  for (uint8_t i = 0; i < N_ENCODERS; i++)
  {
    uint8_t prev_state = (uint8_t) ((((d->a_state_prev >> i) & 0x01) << 1) | ((d->b_state_prev >> i) & 0x01));
    uint8_t curr_state = (uint8_t) ((((d->a_state >> i) & 0x01) << 1) | ((d->b_state >> i) & 0x01));

    d->enc_substep[i] = (int8_t) (d->enc_substep[i] + transition[(prev_state << 2) | curr_state]);

    // A detent is a whole quadrature cycle, and it is only counted once the
    // whole cycle has happened.
    //
    // This used to count a step whenever the state landed on 00, which meant an
    // encoder dithering between its detent and one neighbour - 00, 01, 00, 01 -
    // scored +1 on every return and walked the parameter without being turned.
    // Only the return leg was counted, so the two halves of the dither never
    // cancelled. Accumulating instead makes them cancel exactly, and a real
    // turn still yields the same one count per detent it always did.
    while (d->enc_substep[i] >= MCP_STEPS_PER_DETENT)
    {
      d->enc_substep[i] = (int8_t) (d->enc_substep[i] - MCP_STEPS_PER_DETENT);
      d->enc_position[i]++;
    }
    while (d->enc_substep[i] <= -MCP_STEPS_PER_DETENT)
    {
      d->enc_substep[i] = (int8_t) (d->enc_substep[i] + MCP_STEPS_PER_DETENT);
      d->enc_position[i]--;
    }
  }

  d->a_state_prev = d->a_state;
  d->b_state_prev = d->b_state;
}
