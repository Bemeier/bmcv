#include "ux_state.h"
#include "channel.h"
#include "ctrl_button.h"
#include "hw_setup.h"
#include "quantizer.h"
#include "scene.h"
#include "state.h"
#include "ui_feedback.h"
#include "ui_render.h"
#include "ui_select.h"
#include <stdint.h>

void ui_channel_note_edit(UxState* state, uint8_t channel)
{
  if (channel >= N_CHANNELS)
    return;
  state->engine_state->channels_last_delta[channel] = state->hw_state->time;
}

// Every UI timer ages here and nowhere else. They used to be decremented
// inside render functions and in the hardware poll, against whichever dt was
// nearest to hand.
static void age_timers(UxState* state)
{
  uint32_t dt = state->dt;

  ui_feedback_tick(state->ui, dt);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    uint32_t* hold = &state->ui->channels_edit_hold[c];
    *hold          = (*hold > dt) ? (*hold - dt) : 0;
  }
}

void update_ux_state(UxState* state)
{
  age_timers(state);
  state->ui->exit_consumed_tap = 0;

  for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
  {
    update_shift_mode(&state->ux_setup->ctrl_buttons[b], state);
  }

  // Entering a mode reveals every channel's state for that mode before
  // decaying back to the output level, so the mode is legible at a glance
  // without permanently replacing what the LEDs normally show.
  if (state->ui->shift_state != state->ui->prev_shift_state)
  {
    state->ui->prev_shift_state = state->ui->shift_state;
    ui_sel_reset(state->ui);
    ui_render_arm_all_edits(state);
  }

  for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
  {
    update_selected_param(&state->ux_setup->ctrl_buttons[b], state);
  }

  update_quantizer_buttons(state);

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    update_scene_button(&state->ux_setup->scenes[s], state);
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    update_channel(&state->ux_setup->channels[c], state);
  }

  ui_render(state);
}
