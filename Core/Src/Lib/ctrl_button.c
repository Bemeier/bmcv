#include "ctrl_button.h"
#include "color_presets.h"
#include "config.h"
#include "helpers.h"
#include "led_fb.h"
#include "ui_input.h"
#include "ui_mode.h"
#include <stdint.h>

void ui_ctrl_shift_mode(const CtrlButtonSetup* btn, UxState* state)
{
  if (btn_ev(&state->ui->in, btn->button, BTN_EV_HOLD))
  {
    state->ui->shift_state = (ShiftStates) btn->id;
    return;
  }

  if (!btn_ev(&state->ui->in, btn->button, BTN_EV_TAP) || state->ui->shift_state == SHIFT_STATE_NONE)
    return;

  // A mode's own button always leaves it. Any other ctrl button leaves it too,
  // unless the mode has a keyboard overlay - in QNT the ctrl and scene buttons
  // are a piano for semitone selection, so a tap is a note, not an exit.
  if (btn->id == state->ui->shift_state || !ui_mode(state->ui->shift_state)->keyboard_overlay)
  {
    state->ui->shift_state = SHIFT_STATE_NONE;
    // The tap is consumed by the exit. It used to also set selected_param in
    // the same tick, so leaving a mode via an arbitrary button silently
    // changed what the encoders edit.
    state->ui->exit_consumed_tap = 1;
  }
}

void ui_ctrl_selected_param(const CtrlButtonSetup* btn, UxState* state)
{
  if (state->ui->exit_consumed_tap)
    return;
  if (state->ui->shift_state == SHIFT_STATE_NONE && btn->id < CH_PARAM_COUNT && btn_ev(&state->ui->in, btn->button, BTN_EV_TAP))
  {
    state->engine_config->selected_param = (uint8_t) btn->id;

    // Picking a parameter shows it across every encoder for a moment, the same
    // as touching one does - including when it is the parameter already
    // selected, which makes the button a "show me where these are set" key.
    //
    // The tap that *left* a shift mode is excluded above, deliberately: coming
    // out of a mode should land back on the output monitor rather than on a
    // parameter the user did not ask to see.
    ui_show_param_display(state->ui);
  }
}
