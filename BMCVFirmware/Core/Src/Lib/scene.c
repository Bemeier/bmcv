#include "scene.h"
#include "config.h"
#include "engine_state.h"
#include "helpers.h"
#include "hw_setup.h"
#include <stdint.h>

void scene_compute_contribution(EngineState* es, const EngineConfig* cfg, uint16_t slider, int8_t momentary_scene)
{
  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    es->scenes_contribution[s] = 0;
  }

  if (momentary_scene >= 0)
  {
    es->scenes_contribution[momentary_scene] = 255;
    es->active_scene                         = momentary_scene;
  }
  else
  {
    uint8_t scene_a         = cfg->scene_a;
    uint8_t scene_b         = cfg->scene_b;
    uint16_t scene_a_anchor = SLIDER_MAX_VALUE;
    uint16_t scene_b_anchor = SLIDER_MIN_VALUE;

    if (scene_a == scene_b)
    {
      es->scenes_contribution[scene_a] = 255;
    }
    else
    {
      es->scenes_contribution[scene_a] = interpolate_clamped(scene_b_anchor, scene_a_anchor, slider);
      es->scenes_contribution[scene_b] = 255 - es->scenes_contribution[scene_a];
    }
    es->active_scene = es->scenes_contribution[scene_a] > es->scenes_contribution[scene_b] ? scene_a : scene_b;
  }
}
