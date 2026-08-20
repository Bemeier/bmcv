#ifndef INC_LIB_CTRL_BUTTON_H_
#define INC_LIB_CTRL_BUTTON_H_

#include "ux_setup.h"
#include "ux_state.h"

// Shift activation now comes from UI_T_HOLD in ui_input.h - the one place
// press durations are defined.

void ui_ctrl_shift_mode(const CtrlButtonSetup* btn, UxState* state);

// Releasing the button that opened a page leaves the page again, if anything
// was done with the page while it was held. Not per-button and not part of the
// pass above: it has to run *after* the handlers, or an action and the release
// landing in the same dispatch would be missed. See ux_update.
void ui_ctrl_page_release(UxState* state);

void ui_ctrl_selected_param(const CtrlButtonSetup* btn, UxState* state);

#endif /* INC_LIB_CTRL_BUTTON_H_ */
