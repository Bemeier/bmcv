#include "ctrl_button.h"
#include "color_presets.h"
#include "helpers.h"
#include "state.h"
#include "ui_input.h"
#include "led_fb.h"
#include <stdint.h>

void update_shift_mode(const CtrlButtonSetup* btn, UxState* state)
{
  int8_t b = state->ux_setup->ctrl_buttons[btn->id].button;

  if (btn_ev(&state->in, b, BTN_EV_HOLD))
  {
    state->engine_state->shift_state = (ShiftStates) btn->id;
  }
  // QNT is the exception: there the ctrl and scene buttons are a piano-style
  // keyboard for semitone selection, so only QNT's own button may exit it.
  else if (btn_ev(&state->in, b, BTN_EV_TAP) &&
           (state->engine_state->shift_state != SHIFT_STATE_QNT || btn->id == SHIFT_STATE_QNT))
  {
    state->engine_state->shift_state = SHIFT_STATE_NONE;
  }
}

void update_selected_param(const CtrlButtonSetup* btn, UxState* state)
{
  int8_t b = state->ux_setup->ctrl_buttons[btn->id].button;
  if (state->engine_state->shift_state == SHIFT_STATE_NONE && btn->id < CH_PARAM_COUNT && btn_ev(&state->in, b, BTN_EV_TAP))
  {
    state->engine_state->selected_param = (ChannelParameters) btn->id;
  }
}

void write_ctrl_button_led(const CtrlButtonSetup* btn, UxState* state)
{
  if (btn->led < 0)
    return;

  if (state->engine_state->shift_state == btn->id)
  {
    led_set_hsv(state, btn->led, btn->color, SAT_MAX, state->engine_state->blink_slow ? VAL_MED : 0);
  }
  else if (state->engine_state->selected_param == btn->id && state->engine_state->shift_state == SHIFT_STATE_NONE)
  {
    led_set_hsv(state, btn->led, btn->color, SAT_MAX, VAL_MED);
  }
  else
  {
    led_set_hsv(state, btn->led, btn->color, SAT_MAX, VAL_OFF);
  }
}
