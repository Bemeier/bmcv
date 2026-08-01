#include "engine.h"
#include "channel.h"
#include "helpers.h"
#include "hw_setup.h"
#include "scene.h"
#include "state.h"
#include "ux_state.h"

#define UX_UPDATE_INTERVAL MS(8)

void engine_tick(UxState* state, uint32_t now_us, uint8_t input_dirty)
{
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
  state->dt = now_us - state->last_ux_update;
  if (input_dirty || state->dt > UX_UPDATE_INTERVAL)
  {
    state->last_ux_update = now_us;
    update_ux_state(state);
  }
}
