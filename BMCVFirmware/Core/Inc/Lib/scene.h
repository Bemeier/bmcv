#ifndef INC_LIB_SCENE_H_
#define INC_LIB_SCENE_H_

#include "config.h"
#include "engine_state.h"
#include <stdint.h>

// The scene crossfade: which scenes contribute to the parameters the engine
// actually uses, and how much of each.
//
// Pure. The blend depends only on the slider, the configured A/B pair and
// whichever scene the user is holding. Taking momentary_scene as an argument
// rather than reading UiState keeps the one UI->DSP dependency explicit and
// lets this be exercised without an interaction layer at all.
//
// The scene *buttons* are ui_scene.h.
void scene_compute_contribution(EngineState* es, const EngineConfig* cfg, uint16_t slider, int8_t momentary_scene);

#endif /* INC_LIB_SCENE_H_ */
