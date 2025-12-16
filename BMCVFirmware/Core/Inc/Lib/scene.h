#ifndef INC_LIB_SCENE_H_
#define INC_LIB_SCENE_H_

#include "state.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;
    int8_t button;
    int8_t led;
    uint8_t contribution; // pre morph evaluation (which is per channel)
} Scene;

void update_scene(Scene* scene, SystemState* state, ConfigState* system);

int8_t update_scene_button(Scene* scn, SystemState* state, ConfigState* system);

#endif /* INC_LIB_SCENE_H_ */
