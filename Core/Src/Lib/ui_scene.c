#include "ui_scene.h"
#include "config.h"
#include "error.h"
#include "hw_setup.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"
#include "ux_state.h"
#include <stdint.h>

void ui_scene_button(const SceneSetup* scene, UxState* state)
{
  const UiModeDesc* m = ui_mode(state->ui->shift_state);
  int8_t pressed      = btn_ev(&state->ui->in, scene->button, BTN_EV_UP);
  int8_t momentary    = btn_holding(&state->ui->in, scene->button, UI_T_DEBOUNCE);

  // Modes where the scene buttons address inputs only have four of them.
  if (m->scene_btn_kind == TGT_INPUT && scene->id >= N_INPUTS)
    return;

  switch (m->scene_btn_action)
  {
  case SCN_MOMENTARY:
    if (momentary && state->ui->momentary_scene < 0)
      state->ui->momentary_scene = scene->id;
    else if (!momentary && state->ui->momentary_scene == scene->id)
      state->ui->momentary_scene = -1;
    break;

  case SCN_SET_XFADE:
    // STA and STB differ only in which end they wire, so the mode table says
    // which and this is one case rather than two.
    if (pressed)
    {
      if (m->xfade_end == XFADE_A)
        state->engine_config->scene_a = scene->id;
      else if (m->xfade_end == XFADE_B)
        state->engine_config->scene_b = scene->id;
      else
        break;
      ui_feedback_emit(state->ui, FB_WRITE, TGT_SCENE, scene->id);
    }
    break;

  case SCN_INPUT_MODE:
    if (pressed)
      state->engine_config->input_mode[scene->id] =
          (int8_t) ((state->engine_config->input_mode[scene->id] + 1) % INPUT_MODE_COUNT);
    break;

  case SCN_PRESET:
    // Store fires the moment the hold crosses UI_T_VLONG, so the red LED and
    // the write happen together; the matching release must then not also load.
    if (btn_ev(&state->ui->in, scene->button, BTN_EV_VLONG))
    {
      // Symmetric with the load path below: a store that did not happen must
      // not flash the same green as one that did.
      if (ux_preset_store(state, scene->id))
        ui_feedback_emit(state->ui, FB_WRITE, TGT_SCENE, scene->id);
      else
        error_set(state->engine_state, ERR_PRESET_STORE);
    }
    else if (pressed && btn_held(&state->ui->in, scene->button) < UI_T_VLONG)
    {
      if (ux_preset_load(state, scene->id))
        ui_feedback_emit(state->ui, FB_LOAD, TGT_SCENE, -1);
      else
        error_set(state->engine_state, ERR_PRESET_LOAD);
    }
    break;

  case SCN_SELECT:
    if (pressed)
      ui_sel_press(state, (TargetKind) m->scene_btn_kind, scene->id, 0);
    break;

  default:
    break;
  }
}
