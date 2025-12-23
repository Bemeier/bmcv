#include "scene.h"
#include "state.h"
#include "usbd_def.h"
#include "uxstate.h"
#include "ws2811.h"
#include <stdint.h>

static uint8_t input_mode_color[INPUT_MODE_COUNT] = {HUE_RED, HUE_MAGENTA, HUE_CYAN, HUE_GREEN};

static uint8_t min_value = 12;

void update_scene(Scene* scene, BaseState* state, ConfigState* system)
{
    if (state->shift_state == SHIFT_STATE_SYS)
    {
        if (scene->id < N_INPUTS)
        {
            ws2811_setled_hsv(scene->led, input_mode_color[system->input_mode[scene->id]], 255, 12);
            return;
        }
    }
    else if (state->shift_state != SHIFT_STATE_NONE && state->shift_state != SHIFT_STATE_STL && state->shift_state != SHIFT_STATE_STR)
    {
        return;
    }

    uint8_t val = scene->contribution / 8 + (state->active_scene == scene->id ? min_value : 0);

    if (system->scene_l == scene->id && state->shift_state == SHIFT_STATE_STL)
    {
        val = MAX(val, min_value) * state->blink_fast;
    }
    else if (system->scene_r == scene->id && state->shift_state == SHIFT_STATE_STR)
    {
        val = MAX(val, min_value) * state->blink_fast;
    }

    ws2811_setled_hsv(scene->led, 0, 0, val);
}

int8_t update_scene_button(Scene* scn, BaseState* state, ConfigState* system)
{
    if (state->system->button_released_t[scn->button] > 0)
    {
        if (state->shift_state == SHIFT_STATE_STL)
        {
            system->scene_l = scn->id;
        }

        if (state->shift_state == SHIFT_STATE_STR)
        {
            system->scene_r = scn->id;
        }

        if (state->shift_state == SHIFT_STATE_SYS)
        {
            if (scn->id < N_INPUTS)
            {
                system->input_mode[scn->id] = (system->input_mode[scn->id] + 1) % INPUT_MODE_COUNT;
            }
        }
    }
    else if (state->system->button_pressed_t[scn->button] > 0)
    {
        if (state->shift_state != SHIFT_STATE_NONE)
        {
            return -1;
        }

        return 1;
    }

    return -1;
}
