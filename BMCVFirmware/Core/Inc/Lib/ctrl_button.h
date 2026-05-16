#ifndef INC_LIB_CTRL_BUTTON_H_
#define INC_LIB_CTRL_BUTTON_H_

#include "ux_setup.h"
#include "ux_state.h"

#define CTRL_SHIFT_ACTIVATION 100000

void update_shift_mode(const CtrlButtonSetup* btn, UxState* state);

void update_selected_param(const CtrlButtonSetup* btn, UxState* state);

void write_ctrl_button_led(const CtrlButtonSetup* btn, UxState* state);

#endif /* INC_LIB_CTRL_BUTTON_H_ */
