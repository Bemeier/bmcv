// The generated panel layout must agree with the firmware's index tables.
//
// Every WS2811 is mounted directly behind the control it lights, so for each
// pairing UxSetup declares - channel button/led/encoder, scene button/led,
// ctrl button/led, quantizer button/led - the two parts must sit at the same
// place on the panel. That makes this a real check on hw_setup.c: swap two
// entries in any of the index tables and a pairing lands on a control that is
// physically somewhere else, and this fails.
//
// Regenerate the layout with `just panel` after editing hw_setup.c.

#include "hw_setup.h"
#include "panel_layout.h"
#include "testkit.h"
#include "ux_setup.h"
#include <math.h>
#include <string.h>

#define TOL_MM 1.0f

static float dist(PanelPoint a, PanelPoint b)
{
  float dx = a.x - b.x, dy = a.y - b.y;
  return sqrtf(dx * dx + dy * dy);
}

static const UxSetup* ux(void) { return UxSetup_InitFromHw(HwSetup_Get()); }

TEST_CASE(channel_button_led_encoder_are_colocated)
{
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    const ChannelSetup* ch = &ux()->channels[c];
    CHECK(ch->button >= 0 && ch->button < N_BUTTONS);
    CHECK(ch->led >= 0 && ch->led < LED_COUNT);
    CHECK(ch->encoder >= 0 && ch->encoder < N_ENCODERS);

    PanelPoint btn = panel_button_pos[ch->button];
    PanelPoint led = panel_led_pos[ch->led];
    PanelPoint enc = panel_encoder_pos[ch->encoder];

    CHECK(dist(btn, led) < TOL_MM);
    CHECK(dist(btn, enc) < TOL_MM);
  }
}

TEST_CASE(scene_button_and_led_are_colocated)
{
  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    const SceneSetup* sc = &ux()->scenes[s];
    CHECK(sc->button >= 0 && sc->button < N_BUTTONS);
    CHECK(sc->led >= 0 && sc->led < LED_COUNT);
    CHECK(dist(panel_button_pos[sc->button], panel_led_pos[sc->led]) < TOL_MM);
  }
}

TEST_CASE(ctrl_button_and_led_are_colocated)
{
  for (uint8_t i = 0; i < N_CTRL_BUTTONS; i++)
  {
    const CtrlButtonSetup* cb = &ux()->ctrl_buttons[i];
    CHECK(cb->button >= 0 && cb->button < N_BUTTONS);
    if (cb->led < 0)
    {
      continue; // MUT/CPY/CLR are the unlit tactiles
    }
    CHECK(dist(panel_button_pos[cb->button], panel_led_pos[cb->led]) < TOL_MM);
  }
}

TEST_CASE(quantizer_button_and_led_are_colocated)
{
  for (uint8_t i = 0; i < N_SEMITONES; i++)
  {
    const QuantizerSetup* q = &ux()->quantizer_semitones[i];
    CHECK(q->button >= 0 && q->button < N_BUTTONS);
    CHECK(q->led >= 0 && q->led < LED_COUNT);
    CHECK(dist(panel_button_pos[q->button], panel_led_pos[q->led]) < TOL_MM);
  }
}

// The three unlit ctrl buttons must be the three parts with no LED behind
// them, and every other button must have exactly one LED on top of it.
TEST_CASE(only_the_tactiles_have_no_led)
{
  uint8_t has_led[N_BUTTONS] = {0};
  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    for (uint8_t l = 0; l < LED_COUNT; l++)
    {
      if (dist(panel_button_pos[b], panel_led_pos[l]) < TOL_MM)
      {
        has_led[b]++;
      }
    }
  }

  uint8_t unlit = 0;
  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    CHECK(has_led[b] <= 1);
    if (has_led[b] == 0)
    {
      unlit++;
    }
  }
  CHECK(unlit == 3);
}

TEST_CASE(no_two_controls_share_a_position)
{
  for (uint8_t a = 0; a < N_BUTTONS; a++)
  {
    for (uint8_t b = (uint8_t) (a + 1); b < N_BUTTONS; b++)
    {
      CHECK(dist(panel_button_pos[a], panel_button_pos[b]) >= TOL_MM);
    }
  }
  for (uint8_t a = 0; a < LED_COUNT; a++)
  {
    for (uint8_t b = (uint8_t) (a + 1); b < LED_COUNT; b++)
    {
      CHECK(dist(panel_led_pos[a], panel_led_pos[b]) >= TOL_MM);
    }
  }
}

// Jacks are on the panel, not stacked on top of a control.
TEST_CASE(jacks_are_clear_of_the_controls)
{
  for (uint8_t o = 0; o < N_CHANNELS; o++)
  {
    for (uint8_t b = 0; b < N_BUTTONS; b++)
    {
      CHECK(dist(panel_output_pos[o], panel_button_pos[b]) >= TOL_MM);
    }
  }
  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    for (uint8_t o = 0; o < N_CHANNELS; o++)
    {
      CHECK(dist(panel_input_pos[i], panel_output_pos[o]) >= TOL_MM);
    }
  }
}

// Everything must land inside the board outline.
TEST_CASE(everything_is_on_the_board)
{
  const PanelPoint* groups[] = {panel_button_pos, panel_led_pos, panel_encoder_pos, panel_output_pos, panel_input_pos};
  const uint8_t counts[]     = {N_BUTTONS, LED_COUNT, N_ENCODERS, N_CHANNELS, N_INPUTS};

  for (uint8_t g = 0; g < 5; g++)
  {
    for (uint8_t i = 0; i < counts[g]; i++)
    {
      CHECK(groups[g][i].x >= 0.0f && groups[g][i].x <= PANEL_BOARD_W_MM);
      CHECK(groups[g][i].y >= 0.0f && groups[g][i].y <= PANEL_BOARD_H_MM);
    }
  }
  CHECK(panel_slider_pos.x >= 0.0f && panel_slider_pos.x <= PANEL_BOARD_W_MM);
  CHECK(panel_slider_pos.y >= 0.0f && panel_slider_pos.y <= PANEL_BOARD_H_MM);
}

// Rack sizes a module from its panel SVG and only lands it on the rail grid
// at a whole number of HP. The board is 80mm and 16HP is 81.28, so the VCV
// panel is deliberately 0.28mm wider than the artwork the module is milled to
// and everything on it shifts by half of that.
#define HP_MM 5.08f

TEST_CASE(the_vcv_panel_is_a_whole_number_of_hp)
{
  CHECK(fabsf(PANEL_VCV_W_MM - PANEL_HP * HP_MM) < 0.001f);
  // Every Eurorack panel is this tall. Rack rejects anything else.
  CHECK(fabsf(PANEL_VCV_H_MM - 128.5f) < 0.001f);
}

TEST_CASE(the_board_is_centred_in_the_vcv_panel)
{
  CHECK(fabsf(PANEL_VCV_OFF_X_MM * 2.0f - (PANEL_VCV_W_MM - PANEL_BOARD_W_MM)) < 0.001f);
  CHECK(fabsf(PANEL_VCV_OFF_Y_MM * 2.0f - (PANEL_VCV_H_MM - PANEL_BOARD_H_MM)) < 0.001f);
}

// Same check as everything_is_on_the_board, in the coordinates the Rack widget
// actually places things at. The margin is what a control needs around it: the
// largest is the encoder, 12mm across.
TEST_CASE(everything_fits_inside_the_vcv_panel)
{
  const float margin              = 6.0f;
  const PanelPoint* groups[]      = {panel_button_pos, panel_led_pos, panel_encoder_pos, panel_output_pos, panel_input_pos};
  const uint8_t counts[]          = {N_BUTTONS, LED_COUNT, N_ENCODERS, N_CHANNELS, N_INPUTS};

  for (uint8_t g = 0; g < 5; g++)
  {
    for (uint8_t i = 0; i < counts[g]; i++)
    {
      float x = groups[g][i].x + PANEL_VCV_OFF_X_MM;
      float y = groups[g][i].y + PANEL_VCV_OFF_Y_MM;
      CHECK(x - margin >= 0.0f && x + margin <= PANEL_VCV_W_MM);
      CHECK(y - margin >= 0.0f && y + margin <= PANEL_VCV_H_MM);
    }
  }

  // The crossfader is the one control wider than the margin above.
  float sx = panel_slider_pos.x + PANEL_VCV_OFF_X_MM;
  float sy = panel_slider_pos.y + PANEL_VCV_OFF_Y_MM;
  CHECK(sx - PANEL_SLIDER_TRAVEL_MM / 2.0f >= 0.0f);
  CHECK(sx + PANEL_SLIDER_TRAVEL_MM / 2.0f <= PANEL_VCV_W_MM);
  CHECK(sy >= 0.0f && sy <= PANEL_VCV_H_MM);
}

// The tooltip text a host shows. Generated from the same roles the web
// frontend's hover hint uses, so an empty one means a button fell out of every
// index table.
TEST_CASE(every_button_has_a_label)
{
  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    CHECK(panel_button_label[b] != NULL);
    CHECK(panel_button_label[b][0] != '\0');
    // "SW1 - ..." : a designator, then what it does. A label that is only a
    // designator means no role was found for it.
    CHECK(strchr(panel_button_label[b], '-') != NULL);
  }
}

// The legends a host prints beside a switch. Six parameters and nine shift
// modes is what hw_setup.c declares, so a table that has drifted - or a
// role that stopped resolving - shows up as a different count here.
TEST_CASE(the_panel_legends_match_the_ctrl_tables)
{
  uint8_t params = 0, modes = 0;
  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    if (panel_button_param[b][0])
      params++;
    if (panel_button_mode[b][0])
      modes++;
  }
  CHECK(params == 6); // FRQ SHP MOD PHS AMP OFS
  CHECK(modes == N_CTRL_BUTTONS);

  // Every button with a parameter legend must also have a mode legend: the
  // parameter selector is the first six of the same nine ctrl buttons.
  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    if (panel_button_param[b][0])
      CHECK(panel_button_mode[b][0] != '\0');
  }
}

int main(void)
{
  RUN_TEST(the_panel_legends_match_the_ctrl_tables);
  RUN_TEST(the_vcv_panel_is_a_whole_number_of_hp);
  RUN_TEST(the_board_is_centred_in_the_vcv_panel);
  RUN_TEST(everything_fits_inside_the_vcv_panel);
  RUN_TEST(every_button_has_a_label);
  RUN_TEST(channel_button_led_encoder_are_colocated);
  RUN_TEST(scene_button_and_led_are_colocated);
  RUN_TEST(ctrl_button_and_led_are_colocated);
  RUN_TEST(quantizer_button_and_led_are_colocated);
  RUN_TEST(only_the_tactiles_have_no_led);
  RUN_TEST(no_two_controls_share_a_position);
  RUN_TEST(jacks_are_clear_of_the_controls);
  RUN_TEST(everything_is_on_the_board);
  return TESTKIT_SUMMARY();
}
