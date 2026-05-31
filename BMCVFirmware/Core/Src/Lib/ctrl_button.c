#include "ctrl_button.h"
#include "color_presets.h"
#include "helpers.h"
#include "state.h"
#include "ws2811.h"
#include <stdint.h>

void update_shift_mode(const CtrlButtonSetup* btn, UxState* state)
{
  uint32_t pressed_since  = state->hw_state->button_pressed_t[state->ux_setup->ctrl_buttons[btn->id].button];
  uint32_t released_after = state->hw_state->button_released_t[state->ux_setup->ctrl_buttons[btn->id].button];

  if (pressed_since > CTRL_SHIFT_ACTIVATION)
  {
    state->engine_state->shift_state = (ShiftStates) btn->id;
  }
  else if (released_after > 0 && released_after <= CTRL_SHIFT_ACTIVATION &&
           (state->engine_state->shift_state != SHIFT_STATE_QNT || btn->id == SHIFT_STATE_QNT))
  {
    state->engine_state->shift_state = SHIFT_STATE_NONE;
  }
}

void update_selected_param(const CtrlButtonSetup* btn, UxState* state)
{
  uint32_t released_after = state->hw_state->button_released_t[state->ux_setup->ctrl_buttons[btn->id].button];
  if (state->engine_state->shift_state == SHIFT_STATE_NONE && btn->id < CH_PARAM_COUNT && released_after > 0 && released_after < MS(200))
  {
    state->engine_state->selected_param = (ChannelParameters) btn->id;
    return;
  }
}

void write_ctrl_button_led(const CtrlButtonSetup* btn, UxState* state)
{
  if (btn->led < 0)
    return;

  if (state->engine_state->shift_state == btn->id)
  {
    ws2811_setled_hsv(btn->led, btn->color, SAT_MAX, state->engine_state->blink_slow ? VAL_MED : 0);
  }
  else if (state->engine_state->selected_param == btn->id && state->engine_state->shift_state == SHIFT_STATE_NONE)
  {
    ws2811_setled_hsv(btn->led, btn->color, SAT_MAX, VAL_MED);
  }
  else
  {
    ws2811_setled_hsv(btn->led, btn->color, SAT_MAX, VAL_OFF);
  }
}
