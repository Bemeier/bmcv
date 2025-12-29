#ifndef INC_LIB_ASSIGN_H_
#define INC_LIB_ASSIGN_H_

#include "ux_state.h"
#include <stdint.h>

typedef enum
{
    ASSIGN_NONE,
    ASSIGN_CHANNEL,
    ASSIGN_SCENE,
    ASSIGN_INPUT,
    ASSIGN_PARAM
} AssignType;

/*

- Copy scene to scene
  - (any mode->copy->)
  - i.e. duplicate full scene, all channels
- Copy channel to scene
  - (any mode->copy->)
  - set current channel state to all scenes
- Copy channel to channel
  - (any mode->copy->)
  - copy channel to other channel in current scene
- Assign input to channel
  - (from monitor mode->input->)
  - ...
*/

int8_t assign_src();

AssignType assign_state();

void assign_event(AssignType sourceType, int8_t sourceId, UxState* state);

void assign_reset();

void assign_input_to_channel(int8_t i, int8_t c, UxState* state);

void assign_channel_to_channel(int8_t c_src, int8_t c_dst, UxState* state);

void assign_channel_to_scene(int8_t c_src, int8_t s_dst, UxState* state);

void assign_scene_to_scene(int8_t s_src, int8_t s_dst, UxState* state);

void clear_channel(int8_t c, int8_t all_scenes, UxState* state);

void clear_scene(int8_t s, UxState* state);

#endif /* INC_LIB_ASSIGN_H_ */
