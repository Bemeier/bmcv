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
  }
}

void ui_ctrl_selected_param(const CtrlButtonSetup* btn, UxState* state)
{
  if (state->ui->shift_state == SHIFT_STATE_NONE && btn->id < CH_PARAM_COUNT && btn_ev(&state->ui->in, btn->button, BTN_EV_TAP))
  {
    state->engine_config->selected_param = (uint8_t) btn->id;

    // Picking a parameter shows it across every encoder for a moment, the same
    // as touching one does - including when it is the parameter already
    // selected, which makes the button a "show me where these are set" key.
    //
    // The tap that leaves a shift mode selects its parameter too, and reads the
    // same as any other press of that button. This used to be suppressed, on
    // the grounds that leaving a mode should not silently change what the
    // encoders edit - but the six page buttons *are* the six parameter buttons,
    // and the suppression meant leaving MIX by pressing AMP landed on whatever
    // was selected before, so the way to reach AMP was to press it twice. One
    // rule instead: a labelled button gives you the thing it is labelled with.
    // ux_update runs this pass after the shift-mode pass, so the exit and the
    // selection land in the same tick.
    ui_show_param_display(state->ui);
  }
}
