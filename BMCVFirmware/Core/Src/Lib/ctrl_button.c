#include "ctrl_button.h"
#include "clock_sync.h"
#include "state.h"
#include "ws2811.h"
#include <math.h>
#include <stdint.h>

void init_ctrl_button(CtrlButton* btn)
{
    // ...
}

void update_ctrl_button(CtrlButton* btn, SystemState* state)
{
    if (btn->led < 0)
    {
        return;
    }

    uint8_t btn_state = state->button_state[btn->button];

    uint8_t flag_active = state->ctrl_flags & btn->ctrl_flags;

    uint8_t sat = flag_active ? 255 : 20;

    uint8_t val = 0;

    // uint8_t phase_color = (uint8_t) roundf(g_clk.beat_phase * 255.0f);
    uint8_t sub         = g_clk.have_beat && fmodf(g_clk.beat_phase, 1.0f / g_clk.PULSES_PER_BEAT) < 0.05f ? 5 : 0;
    uint8_t phase_color = g_clk.have_beat && g_clk.beat_phase < 0.05 ? 50 : 0 + sub;

    if (flag_active)
    {
        val = 20 * (btn_state == 0 ? 1 : state->blink_fast);
    }

    if (btn->ctrl_flags & CTRL_INP)
    {
        ws2811_setled_hsv(btn->led, btn->color, 255, phase_color);
    }
    else
    {
        ws2811_setled_hsv(btn->led, btn->color, sat, val);
    }
}
