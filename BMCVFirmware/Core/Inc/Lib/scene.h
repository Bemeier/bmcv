#ifndef INC_LIB_SCENE_H_
#define INC_LIB_SCENE_H_

#include "state.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;
    int8_t button;
    int8_t led;
    uint8_t color;
    uint8_t alt_ctrl_flags;
    uint16_t position;
    uint8_t contribution; // pre morph evaluation (which is per channel)
} Scene;

void update_scene(Scene* scene, State* state);

int8_t update_scene_button(Scene* scn, State* state);

#endif /* INC_LIB_SCENE_H_ */
