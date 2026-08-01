#ifndef INC_LIB_SCENE_H_
#define INC_LIB_SCENE_H_

#include "ux_state.h"

void update_scene_button(const SceneSetup* scn, UxState* state);

void compute_scenes_contribution(EngineState* es, const EngineConfig* cfg, uint16_t slider, int8_t momentary_scene);

#endif /* INC_LIB_SCENE_H_ */
