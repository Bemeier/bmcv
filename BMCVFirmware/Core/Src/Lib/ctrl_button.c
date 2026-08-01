#include "ctrl_button.h"
#include "color_presets.h"
#include "helpers.h"
#include "led_fb.h"
#include "state.h"
#include "ui_input.h"
#include "ui_mode.h"
#include <stdint.h>

void update_shift_mode(const CtrlButtonSetup* btn, UxState* state)
{
  int8_t b = state->ux_setup->ctrl_buttons[btn->id].button;

  if (btn_ev(&state->ui->in, b, BTN_EV_HOLD))
  {
    state->ui->shift_state = (ShiftStates) btn->id;
    return;
  }

  if (!btn_ev(&state->ui->in, b, BTN_EV_TAP) || state->ui->shift_state == SHIFT_STATE_NONE)
    return;

  // A mode's own button always leaves it. Any other ctrl button leaves it too,
  // except in QNT - there the ctrl and scene buttons are a piano-style
  // keyboard for semitone selection, so a tap is a note, not an exit.
  if (btn->id == state->ui->shift_state || ui_mode(state->ui->shift_state)->exits_on_other_ctrl)
  {
    state->ui->shift_state = SHIFT_STATE_NONE;
    // The tap is consumed by the exit. It used to also set selected_param in
    // the same tick, so leaving a mode via an arbitrary button silently
    // changed what the encoders edit.
    state->ui->exit_consumed_tap = 1;
  }
}

void update_selected_param(const CtrlButtonSetup* btn, UxState* state)
{
  int8_t b = state->ux_setup->ctrl_buttons[btn->id].button;
  if (state->ui->exit_consumed_tap)
    return;
  if (state->ui->shift_state == SHIFT_STATE_NONE && btn->id < CH_PARAM_COUNT && btn_ev(&state->ui->in, b, BTN_EV_TAP))
  {
    state->ui->selected_param = (ChannelParameters) btn->id;
  }
}
