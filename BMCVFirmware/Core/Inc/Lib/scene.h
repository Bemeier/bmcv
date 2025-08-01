#ifndef INC_LIB_SCENE_H_
#define INC_LIB_SCENE_H_

#include <stdint.h>
#include "state.h"

typedef struct {
    uint8_t id;
    uint8_t button;
    uint8_t led;
    uint8_t color;
    uint16_t position;
} Scene;

void update_scene(Scene * scene, State * state);

#endif /* INC_LIB_SCENE_H_ */