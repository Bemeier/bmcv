#include "scene.h"
#include "state.h"
#include "ws2811.h"
#include <stdint.h>

void update_scene(Scene* scene, State* state)
{
    if (state->ctrl_flags & (CTRL_QNT | CTRL_SYS | CTRL_MON | CTRL_SEQ))
    {
        return;
    }

    uint8_t val = state->active_scene_id == scene->id ? 40 : 5;

    if (state->scene_latch == scene->id && state->ctrl_flags & CTRL_LAT)
    {
        val = val * state->blink_fast;
    }
    else if (state->scene_l == scene->id && state->ctrl_flags & CTRL_STL)
    {
        val = val * state->blink_fast;
    }
    else if (state->scene_r == scene->id && state->ctrl_flags & CTRL_STR)
    {
        val = val * state->blink_fast;
    }

    ws2811_setled_hsv(scene->led, scene->color, scene->contribution, val);
}

int8_t update_scene_button(Scene* scn, State* state)
{
    if (state->button_released_t[scn->button] > 0)
    {
        if (state->ctrl_flags & CTRL_LAT)
        {
            state->scene_latch          = scn->id;
            state->scene_latch_position = state->slider_position;

            if (state->ctrl_flags & CTRL_STL || state->ctrl_flags & CTRL_STL)
            {
                state->scene_latch = -1;
            }
        }

        if (state->ctrl_flags & CTRL_STL)
        {
            state->scene_l = scn->id;
        }

        if (state->ctrl_flags & CTRL_STR)
        {
            state->scene_r = scn->id;
        }
    }
    else if (state->button_pressed_t[scn->button] > 0)
    {
        if (state->ctrl_flags & (CTRL_STL | CTRL_LAT | CTRL_QNT | CTRL_SYS | CTRL_MON | CTRL_SEQ | CTRL_STR))
        {
            return -1;
        }

        return 1;
    }

    return -1;
}
