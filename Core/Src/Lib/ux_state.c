#include "ux_state.h"
#include "config.h"
#include "ctrl_button.h"
#include "helpers.h"
#include "hw_setup.h"
#include "quantizer.h"
#include "ui_channel.h"
#include "ui_feedback.h"
#include "ui_render.h"
#include "ui_scene.h"
#include "ui_select.h"
#include <stdint.h>

#define AUTOSAVE_INTERVAL MS(2000)

int8_t ux_preset_store(const UxState* state, int8_t slot)
{
  if (!state->presets || !state->presets->store)
    return 0;
  return state->presets->store(state->presets->user, state->engine_config, slot);
}

int8_t ux_preset_load(UxState* state, int8_t slot)
{
  if (!state->presets || !state->presets->load)
    return 0; // "nothing stored" - the first-boot path
  return state->presets->load(state->presets->user, state->engine_config, slot);
}

void ux_autosave_init(UxState* state, uint32_t now_us)
{
  state->last_autosave_us = now_us;
  state->last_crc         = crc32(state->engine_config, sizeof(EngineConfig));
}

// Config is written back to the autosave slot periodically, so a power cut
// does not lose the last few minutes of knob twiddling. Only on an actual
// change - the CRC is what decides, not a dirty flag, because config is
// mutated from a dozen places.
//
// Deliberately silent: this is background housekeeping on a timer, not
// something the user did, and flashing the whole scene row every time a knob
// settles is just noise. Confirmations are for committed *actions* - an
// explicit save still flashes, in scene.c.
static void autosave(UxState* state, uint32_t now_us)
{
  if (now_us - state->last_autosave_us <= AUTOSAVE_INTERVAL)
    return;

  state->last_autosave_us = now_us;

  uint32_t crc_now = crc32(state->engine_config, sizeof(EngineConfig));
  if (crc_now == state->last_crc)
    return;

  // Only record the config as saved if it actually was. Updating last_crc
  // regardless would mean a failed write is never retried - the next attempt
  // sees no change and does nothing until the user edits something else.
  if (ux_preset_store(state, CONFIG_AUTOSAVE_SLOT))
    state->last_crc = crc_now;
}

void ux_note_channel_edit(UxState* state, uint8_t channel)
{
  if (channel >= N_CHANNELS)
    return;
  state->engine_state->channels_last_delta[channel] = state->hw_state->time;
}

// Every UI timer ages here and nowhere else. They used to be decremented
// inside render functions and in the hardware poll, against whichever dt was
// nearest to hand.
static void age_timers(UxState* state, uint32_t dt)
{
  ui_feedback_tick(state->ui, dt);

  uint32_t* hold = &state->ui->param_display_hold;
  *hold          = (*hold > dt) ? (*hold - dt) : 0;
}

void ux_update(UxState* state, uint32_t now_us)
{
  age_timers(state, state->ui->in.dt);

  for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
  {
    ui_ctrl_shift_mode(&state->ux_setup->ctrl_buttons[b], state);
  }

  // A mode's channel LEDs show that mode's own setting for as long as it is
  // active, so entry needs no reveal - only the half-finished selection from
  // the mode being left has to go.
  if (state->ui->shift_state != state->ui->prev_shift_state)
  {
    state->ui->prev_shift_state = state->ui->shift_state;
    ui_sel_reset(state->ui);
  }

  // After the shift-mode pass, not before it. The six page buttons are the six
  // parameter buttons, so the tap that leaves a mode has to be able to select
  // the parameter it is labelled with in the same tick - which it can only do
  // once shift_state is back to NONE.
  for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
  {
    ui_ctrl_selected_param(&state->ux_setup->ctrl_buttons[b], state);
  }

  ui_quantizer_update(state);

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    ui_scene_button(&state->ux_setup->scenes[s], state);
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    ui_channel_update(&state->ux_setup->channels[c], state);
  }

  autosave(state, now_us);

  ui_render(state);
}
