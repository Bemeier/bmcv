#include "engine.h"
#include "channel.h"
#include "helpers.h"
#include "hw_setup.h"
#include "scene.h"
#include "state.h"
#include "ui_input.h"
#include "ux_state.h"

#define UX_UPDATE_INTERVAL MS(8)

void engine_tick(UxState* state, uint32_t now_us, uint8_t input_dirty)
{
  // Every tick, not just the ticks the UX layer runs on: a hold crossing its
  // threshold is not a level change, so nothing else would notice it.
  ui_input_update(&state->in, state->hw_state);

  state->engine_state->blink_fast = (now_us % FAST_BLINK_PERIOD) < (FAST_BLINK_PERIOD / 2);
  state->engine_state->blink_slow = (now_us % SLOW_BLINK_PERIOD) < (SLOW_BLINK_PERIOD / 2);

  compute_scenes_contribution(state);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    compute_channel(&state->ux_setup->channels[c], state);
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    detect_channel_trigger(&state->ux_setup->channels[c], state);
  }

  // UX runs slower than the signal path: on input change, or on a fixed tick.
  // state->dt is the accumulated input time, so a handler and a renderer in
  // the same pass always age their timers by the same amount.
  if (input_dirty || state->in.dt > UX_UPDATE_INTERVAL)
  {
    state->last_ux_update = now_us;
    state->dt             = state->in.dt;
    update_ux_state(state);
    ui_input_drain(&state->in);
  }
}
