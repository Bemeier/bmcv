#include "ctrl_button.h"
#include "ws2811.h"

void init_ctrl_button(CtrlButton* btn)
{
    // ...
}

void update_ctrl_button(CtrlButton* btn, State* state)
{
    if (btn->led < 0)
    {
        return;
    }

    uint8_t btn_state = state->button_state[btn->button];

    uint8_t flag_active = state->ctrl_flags & btn->ctrl_flags;

    uint8_t sat = flag_active ? 255 : 20;

    uint8_t val = 0;

    if (flag_active)
    {
        val = 20 * (btn_state == 0 ? 1 : state->blink_fast);
    }

    ws2811_setled_hsv(btn->led, btn->color, sat, val);
}
