#include "scene.h"
#include "assign.h"
#include "color_presets.h"
#include "dac_adc.h"
#include "error.h"
#include "helpers.h"
#include "hw_setup.h"
#include "led_fb.h"
#include "presets.h"
#include "state.h"
#include "ui_input.h"
#include "ux_state.h"
#include <stdint.h>

// Pure: the crossfade depends only on the slider, the configured A/B pair and
// whichever scene the user is holding. Taking momentary_scene as an argument
// rather than reading UiState keeps the one UI->DSP dependency explicit and
// lets this be exercised without an interaction layer at all.
void compute_scenes_contribution(EngineState* es, const EngineConfig* cfg, uint16_t slider, int8_t momentary_scene)
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

void update_scene_button(const SceneSetup* scene, UxState* state)
{
  int8_t pressed   = btn_ev(&state->ui->in, scene->button, BTN_EV_UP);
  int8_t momentary = btn_holding(&state->ui->in, scene->button, UI_T_DEBOUNCE);
  // Momentarty activation
  switch (state->ui->shift_state)
  {
  case SHIFT_STATE_NONE: // Normal mode, momentary scene activation
    if (momentary && state->ui->momentary_scene < 0)
    {
      state->ui->momentary_scene = scene->id;
    }

    if (!momentary && state->ui->momentary_scene == scene->id)
    {
      state->ui->momentary_scene = -1;
    }
    break;
  case SHIFT_STATE_STA:
    if (pressed)
      state->engine_config->scene_a = scene->id;
    break;
  case SHIFT_STATE_STB:
    if (pressed)
      state->engine_config->scene_b = scene->id;
    break;
  case SHIFT_STATE_SYS:
    if (pressed && scene->id < N_INPUTS)
      state->engine_config->input_mode[scene->id] = (state->engine_config->input_mode[scene->id] + 1) % INPUT_MODE_COUNT;
    break;
  case SHIFT_STATE_MON:
    if (pressed && scene->id < N_INPUTS)
      ui_sel_press(state, TGT_INPUT, scene->id, 0);
    break;
  case SHIFT_STATE_SAV:
    // Store fires the moment the hold crosses UI_T_VLONG, so the red LED and
    // the write happen together; the matching release must then not also load.
    if (btn_ev(&state->ui->in, scene->button, BTN_EV_VLONG))
    {
      preset_store(state->engine_config, scene->id);
    }
    else if (pressed && btn_held(&state->ui->in, scene->button) < UI_T_VLONG)
    {
      if (!preset_load(state->engine_config, scene->id))
      {
        error_set(5);
      }
    }
    break;
  case SHIFT_STATE_CPY:
    if (pressed)
      ui_sel_press(state, TGT_SCENE, scene->id, 0);
    break;
  case SHIFT_STATE_CLR:
    if (pressed)
      ui_sel_press(state, TGT_SCENE, scene->id, 0);
    break;
  case SHIFT_STATE_QNT:
    if (pressed && scene->id < N_INPUTS)
      ui_sel_press(state, TGT_INPUT, scene->id, 0);
    break;
  default:
    break;
  }
}
