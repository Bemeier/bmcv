#include "scene.h"
#include "assign.h"
#include "state.h"
#include "usbd_def.h"
#include "ws2811.h"
#include <stdint.h>

static uint8_t input_mode_color[INPUT_MODE_COUNT] = {HUE_RED, HUE_MAGENTA, HUE_CYAN, HUE_GREEN};

static uint8_t min_value = 12;

void update_scene(Scene* scene, State* state, System* system)
{
    if (state->ctrl_flags & CTRL_SYS)
    {
        if (scene->id < N_INPUTS)
        {
            ws2811_setled_hsv(scene->led, input_mode_color[system->input_mode[scene->id]], 255, 12);
            return;
        }
    }
    else if (state->ctrl_flags & (CTRL_QNT | CTRL_MON | CTRL_SEQ))
    {
        return;
    }

    uint8_t val = scene->contribution / 8 + (state->active_scene_id == scene->id ? min_value : 0);

    if (state->scene_l == scene->id && state->ctrl_flags & CTRL_STL)
    {
        val = MAX(val, min_value) * state->blink_fast;
    }
    else if (state->scene_r == scene->id && state->ctrl_flags & CTRL_STR)
    {
        val = MAX(val, min_value) * state->blink_fast;
    }

    ws2811_setled_hsv(scene->led, 0, 0, val);
}

int8_t update_scene_button(Scene* scn, State* state, System* system)
{
    if (state->button_released_t[scn->button] > 0)
    {
        if (state->ctrl_flags & CTRL_STL)
        {
            state->scene_l = scn->id;
        }

        if (state->ctrl_flags & CTRL_STR)
        {
            state->scene_r = scn->id;
        }

        if (state->ctrl_flags & CTRL_SYS)
        {
            if (scn->id < N_INPUTS)
            {
                system->input_mode[scn->id] = (system->input_mode[scn->id] + 1) % INPUT_MODE_COUNT;
            }
        }
    }
    else if (state->button_pressed_t[scn->button] > 0)
    {
        if (state->ctrl_flags & (CTRL_STL | CTRL_QNT | CTRL_SYS | CTRL_MON | CTRL_SEQ | CTRL_STR))
        {
            return -1;
        }

        return 1;
    }

    return -1;
}
