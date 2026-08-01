#ifndef INC_LIB_CTRL_BUTTON_H_
#define INC_LIB_CTRL_BUTTON_H_

#include "ux_setup.h"
#include "ux_state.h"

// Shift activation now comes from UI_T_HOLD in ui_input.h - the one place
// press durations are defined.

void update_shift_mode(const CtrlButtonSetup* btn, UxState* state);

void update_selected_param(const CtrlButtonSetup* btn, UxState* state);

#endif /* INC_LIB_CTRL_BUTTON_H_ */
