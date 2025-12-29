#include "ctrl_button.h"
#include "assign.h"
#include "clock_sync.h"
#include "color_presets.h"
#include "state.h"
#include "ws2811.h"
#include <math.h>
#include <stdint.h>

void update_shift_mode(const CtrlButtonSetup* btn, UxState* state)
{
    uint16_t pressed_since = state->hw_state->button_pressed_t[state->ux_setup->ctrl_buttons[btn->id].button];
    if (pressed_since > 200 && pressed_since > state->engine_state->shift_active_for)
    {
        state->engine_state->shift_state = (ShiftStates) btn->id;
    }
}

void update_selected_param(const CtrlButtonSetup* btn, UxState* state)
{
    uint16_t released_after = state->hw_state->button_released_t[state->ux_setup->ctrl_buttons[btn->id].button];
    if (state->engine_state->shift_state == SHIFT_STATE_NONE && btn->id < CH_PARAM_COUNT && released_after > 0 && released_after < 200)
    {
        state->engine_state->selected_param = (ChannelParameters) btn->id;
        return;
    }
}

void write_ctrl_button_led(const CtrlButtonSetup* btn, UxState* state)
{
    if (btn->led < 0)
        return;

    if (state->engine_state->shift_state == btn->id)
    {
        switch (btn->id)
        {
        case SHIFT_STATE_MON:
            uint8_t sub         = g_clk.have_beat && fmodf(g_clk.beat_phase, 1.0f / g_clk.PULSES_PER_BEAT) < 0.05f ? 5 : 0;
            uint8_t phase_color = g_clk.have_beat && g_clk.beat_phase < 0.05 ? VAL_MED : 0 + sub;
            ws2811_setled_hsv(btn->led, btn->color, SAT_MAX, phase_color);
            break;
        default:
            ws2811_setled_hsv(btn->led, btn->color, SAT_MAX, state->engine_state->blink_slow ? VAL_MED : 0);
            break;
        }
    }
    else if (state->engine_state->selected_param == btn->id)
    {
        ws2811_setled_hsv(btn->led, btn->color, SAT_MAX, VAL_MED);
    }
    else
    {
        ws2811_setled_hsv(btn->led, btn->color, SAT_MAX, VAL_OFF);
    }
}
