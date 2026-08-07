// The panel expanders' port bytes -> buttons and encoder detents.
//
// This is the bottom of the input path: every gesture test in the suite starts
// from HwState.button_state and encoder_delta, and this is the code that fills
// them on hardware. It used to be unreachable from a host build, sitting behind
// the HAL inside mcp.c.

#include "mcp_decode.h"
#include "testkit.h"

// Drive one encoder's A and B lines, leaving the other seven alone. Returns the
// (gpioa, gpiob) pair the expander would present.
typedef struct
{
  uint8_t a;
  uint8_t b;
} Ports;

static Ports ports_for(const McpDecode* d, uint8_t enc, uint8_t a_level, uint8_t b_level)
{
  Ports p     = {0, 0};
  uint8_t pin = d->enc_pins_a[enc];
  if (a_level)
  {
    if (pin < 8)
      p.a |= (uint8_t) (1u << pin);
    else
      p.b |= (uint8_t) (1u << (pin - 8));
  }
  pin = d->enc_pins_b[enc];
  if (b_level)
  {
    if (pin < 8)
      p.a |= (uint8_t) (1u << pin);
    else
      p.b |= (uint8_t) (1u << (pin - 8));
  }
  return p;
}

// (A << 1) | B, the order mcp_decode_encoders reads them in.
static void feed_state(McpDecode* d, uint8_t enc, uint8_t state)
{
  Ports p = ports_for(d, enc, (state >> 1) & 1u, state & 1u);
  mcp_decode_encoders(d, p.a, p.b);
}

// One detent's worth of states, in order, from the 00 the encoder rests in.
// "Up" is the direction the transition table scores positive; which way that
// is on the panel is the wiring's business, and test_ui_input covers what a
// positive delta then does.
static const uint8_t up[4]   = {0b10, 0b11, 0b01, 0b00};
static const uint8_t down[4] = {0b01, 0b11, 0b10, 0b00};

static void settle(McpDecode* d, uint8_t enc)
{
  feed_state(d, enc, 0b00); // first sample only establishes position
  feed_state(d, enc, 0b00);
}

TEST_CASE(a_full_cycle_is_one_detent)
{
  McpDecode d;
  mcp_decode_init(&d);
  settle(&d, 0);

  for (int i = 0; i < 4; i++)
    feed_state(&d, 0, up[i]);

  CHECK(d.enc_position[0] == 1);

  for (int i = 0; i < 4; i++)
    feed_state(&d, 0, down[i]);

  CHECK(d.enc_position[0] == 0);
}

TEST_CASE(ten_detents_count_ten)
{
  McpDecode d;
  mcp_decode_init(&d);
  settle(&d, 3);

  for (int turn = 0; turn < 10; turn++)
  {
    for (int i = 0; i < 4; i++)
      feed_state(&d, 3, up[i]);
  }

  CHECK(d.enc_position[3] == 10);
}

// An encoder sitting in its detent with a noisy contact walks between the rest
// state and one neighbour. It has not been turned, and it must not count.
//
// The previous decoder scored the return leg of each dither and ignored the
// outward one, so this drifted by +1 per bounce - a parameter creeping on its
// own while nobody touched the panel.
TEST_CASE(dither_at_the_detent_does_not_accumulate)
{
  McpDecode d;
  mcp_decode_init(&d);
  settle(&d, 5);

  for (int i = 0; i < 50; i++)
  {
    feed_state(&d, 5, 0b01);
    feed_state(&d, 5, 0b00);
  }

  CHECK(d.enc_position[5] == 0);

  // ...and the same on the other neighbour
  for (int i = 0; i < 50; i++)
  {
    feed_state(&d, 5, 0b10);
    feed_state(&d, 5, 0b00);
  }

  CHECK(d.enc_position[5] == 0);
}

// A partial turn that goes back the way it came is not a detent either.
TEST_CASE(a_partial_turn_reversed_nets_zero)
{
  McpDecode d;
  mcp_decode_init(&d);
  settle(&d, 1);

  feed_state(&d, 1, 0b01);
  feed_state(&d, 1, 0b11);
  feed_state(&d, 1, 0b01);
  feed_state(&d, 1, 0b00);

  CHECK(d.enc_position[1] == 0);
}

// Whatever the ports read at power-on is where the encoders are, not a move.
TEST_CASE(the_first_sample_is_a_position_not_a_movement)
{
  McpDecode d;
  mcp_decode_init(&d);

  mcp_decode_encoders(&d, 0xFF, 0xFF);
  mcp_decode_encoders(&d, 0xFF, 0xFF);

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    CHECK(d.enc_position[e] == 0);
  }
}

// Turning one encoder must not move any other. This is the check that the pin
// map is actually a permutation rather than something with a duplicate in it.
TEST_CASE(encoders_are_independent)
{
  McpDecode d;
  mcp_decode_init(&d);

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    mcp_decode_init(&d);
    settle(&d, e);

    for (int i = 0; i < 4; i++)
      feed_state(&d, e, up[i]);

    for (uint8_t other = 0; other < N_ENCODERS; other++)
    {
      CHECK(d.enc_position[other] == (other == e ? 1 : 0));
    }
  }
}

// Every pin the map names has to be distinct: two controls on one expander pin
// would read as one control, and swapping two entries is exactly the kind of
// edit nothing else in the tree would catch.
TEST_CASE(no_two_controls_share_an_expander_pin)
{
  McpDecode d;
  mcp_decode_init(&d);

  // The encoder expander carries A and B for eight encoders: 16 of its 16 pins.
  uint8_t seen[16] = {0};
  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    CHECK(d.enc_pins_a[e] < 16);
    CHECK(d.enc_pins_b[e] < 16);
    CHECK(seen[d.enc_pins_a[e]] == 0);
    seen[d.enc_pins_a[e]] = 1;
    CHECK(seen[d.enc_pins_b[e]] == 0);
    seen[d.enc_pins_b[e]] = 1;
  }

  // The switch expander carries the eight encoder presses and the eight
  // bottom-row buttons.
  uint8_t seen_btn[16] = {0};
  for (uint8_t b = 0; b < N_ENCODERS; b++)
  {
    CHECK(d.enc_button_pins[b] < 16);
    CHECK(d.bottom_button_pins[b] < 16);
    CHECK(seen_btn[d.enc_button_pins[b]] == 0);
    seen_btn[d.enc_button_pins[b]] = 1;
    CHECK(seen_btn[d.bottom_button_pins[b]] == 0);
    seen_btn[d.bottom_button_pins[b]] = 1;
  }
}

TEST_CASE(each_button_reads_its_own_pin)
{
  McpDecode d;
  mcp_decode_init(&d);

  for (uint8_t b = 0; b < N_ENCODERS; b++)
  {
    uint8_t pin   = d.enc_button_pins[b];
    uint8_t gpioa = (pin < 8) ? (uint8_t) (1u << pin) : 0;
    uint8_t gpiob = (pin < 8) ? 0 : (uint8_t) (1u << (pin - 8));

    mcp_decode_buttons(&d, gpioa, gpiob);

    for (uint8_t i = 0; i < N_ENCODERS * 2; i++)
    {
      CHECK(d.button_state[i] == (i == b ? 1 : 0));
    }
  }

  for (uint8_t b = 0; b < N_ENCODERS; b++)
  {
    uint8_t pin   = d.bottom_button_pins[b];
    uint8_t gpioa = (pin < 8) ? (uint8_t) (1u << pin) : 0;
    uint8_t gpiob = (pin < 8) ? 0 : (uint8_t) (1u << (pin - 8));

    mcp_decode_buttons(&d, gpioa, gpiob);

    for (uint8_t i = 0; i < N_ENCODERS * 2; i++)
    {
      CHECK(d.button_state[i] == (i == N_ENCODERS + b ? 1 : 0));
    }
  }
}

// All ports low is all buttons up, and the decode must not leave anything
// latched from the previous sample.
TEST_CASE(buttons_release_when_their_pin_goes_low)
{
  McpDecode d;
  mcp_decode_init(&d);

  mcp_decode_buttons(&d, 0xFF, 0xFF);
  for (uint8_t i = 0; i < N_ENCODERS * 2; i++)
  {
    CHECK(d.button_state[i] == 1);
  }

  mcp_decode_buttons(&d, 0x00, 0x00);
  for (uint8_t i = 0; i < N_ENCODERS * 2; i++)
  {
    CHECK(d.button_state[i] == 0);
  }
}

int main(void)
{
  RUN_TEST(a_full_cycle_is_one_detent);
  RUN_TEST(ten_detents_count_ten);
  RUN_TEST(dither_at_the_detent_does_not_accumulate);
  RUN_TEST(a_partial_turn_reversed_nets_zero);
  RUN_TEST(the_first_sample_is_a_position_not_a_movement);
  RUN_TEST(encoders_are_independent);
  RUN_TEST(no_two_controls_share_an_expander_pin);
  RUN_TEST(each_button_reads_its_own_pin);
  RUN_TEST(buttons_release_when_their_pin_goes_low);
  return TESTKIT_SUMMARY();
}
