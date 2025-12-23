#include "ctrl_button.h"
#include "clock_sync.h"
#include "state.h"
#include "uxstate.h"
#include "ws2811.h"
#include <math.h>
#include <stdint.h>

void init_ctrl_button(CtrlButton* btn)
{
    // ...
}

void update_ctrl_button(CtrlButton* btn, BaseState* state)
{
    if (btn->led < 0)
    {
        return;
    }

    uint8_t sub         = g_clk.have_beat && fmodf(g_clk.beat_phase, 1.0f / g_clk.PULSES_PER_BEAT) < 0.05f ? 5 : 0;
    uint8_t phase_color = g_clk.have_beat && g_clk.beat_phase < 0.05 ? 50 : 0 + sub;

    if (state->shift_state == btn->id)
    {
        if (btn->id == SHIFT_STATE_MON)
        {
            ws2811_setled_hsv(btn->led, btn->color, 255, phase_color);
        }
        else
        {
            ws2811_setled_hsv(btn->led, btn->color, 255, 20);
        }
        ws2811_setled_hsv(btn->led, btn->color, 255, 20);
    }
    else if (state->selected_param == btn->id)
    {
        ws2811_setled_hsv(btn->led, btn->color, 255, 20);
    }
    else
    {
        ws2811_setled_hsv(btn->led, btn->color, 255, 0);
    }
}
