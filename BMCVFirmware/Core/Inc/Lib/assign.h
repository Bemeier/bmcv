#ifndef INC_LIB_ASSIGN_H_
#define INC_LIB_ASSIGN_H_

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

void assign_reset();
void assign_event(AssignType sourceType, int8_t sourceId);
AssignType assign_state();
int8_t assign_src();

#endif /* INC_LIB_ASSIGN_H_ */
