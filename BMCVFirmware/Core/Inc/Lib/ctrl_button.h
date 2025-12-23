#ifndef INC_LIB_CTRL_BUTTON_H_
#define INC_LIB_CTRL_BUTTON_H_

#include "state.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;
    int8_t button;
    int8_t led;
    uint8_t color;
} CtrlButton;

void init_ctrl_button(CtrlButton* btn);

// Updating => calculating output value
void update_ctrl_button(CtrlButton* btn, BaseState* state);

#endif /* INC_LIB_CTRL_BUTTON_H_ */
