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
    state->ui->shift_state    = (ShiftStates) btn->id;
    state->ui->page_entry_btn = (int8_t) btn->id;
    state->ui->page_used      = 0;
    return;
  }

  if (!btn_ev(&state->ui->in, btn->button, BTN_EV_TAP) || state->ui->shift_state == SHIFT_STATE_NONE)
    return;

  // A page is left by its own button, or by one of the three unlit caps - MUT,
  // CPY and CLR, the ctrl buttons past the six that are also the parameter
  // row. Taps on the other five page buttons do nothing while a page is open.
  //
  // Any ctrl button used to exit, which made every page one stray press from
  // closing, and made the parameter row unusable as a row: the tap that was
  // meant to select SHP closed the page and selected SHP, so reaching a
  // parameter from a page took two presses of two different buttons. The three
  // unlit caps stay as a way out because they are the buttons that are not
  // labelled with something else.
  if (btn->id == state->ui->shift_state || btn->id >= CH_PARAM_COUNT)
  {
    state->ui->shift_state = SHIFT_STATE_NONE;
  }
}

void ui_ctrl_page_release(UxState* state)
{
  int8_t id = state->ui->page_entry_btn;
  if (id < 0 || id >= N_CTRL_BUTTONS)
    return;

  if (!btn_ev(&state->ui->in, state->ux_setup->ctrl_buttons[id].button, BTN_EV_UP))
    return;

  state->ui->page_entry_btn = -1;

  // Used while it was held, so the hold was the gesture: assign a scene with
  // STA down and letting go puts you back where you were, instead of leaving
  // the page open to be tapped shut afterwards. A hold that did nothing latches
  // - that is the two-handed way in, and it is why this is not simply
  // "momentary while held".
  if (state->ui->page_used && state->ui->shift_state == (uint8_t) id)
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
    // The tap that leaves a page selects its parameter too, and reads the same
    // as any other press of that button: a labelled button gives you the thing
    // it is labelled with. Only the page's own button can do both now, since it
    // is the only lit one whose tap still exits. ux_update runs this pass after
    // the shift-mode pass, so the exit and the selection land in the same tick.
    ui_show_param_display(state->ui);
  }
}
