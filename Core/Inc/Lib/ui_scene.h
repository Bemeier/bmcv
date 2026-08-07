#ifndef INC_LIB_UI_SCENE_H_
#define INC_LIB_UI_SCENE_H_

#include "ux_setup.h"
#include "ux_state.h"

// What a scene button does, which depends on the active mode: select a scene
// for a copy, wire one to an end of the crossfader, cycle an input's role,
// load or store a preset, or momentarily jump to it.
//
// Split from scene.c, which is now the crossfade maths only.
void ui_scene_button(const SceneSetup* scn, UxState* state);

#endif /* INC_LIB_UI_SCENE_H_ */
