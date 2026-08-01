#include "assign.h"
#include "channel.h"
#include "hw_setup.h"
#include "state.h"
#include <stdint.h>
#include <string.h>

void assign_input_to_channel(int8_t i, int8_t c, UxState* state) { state->engine_config->channel_state[c].src_input = i; }

void clear_channel(int8_t c, int8_t all_scenes, UxState* state)
{
  reset_channel(&state->ux_setup->channels[c], state, all_scenes ? -1 : state->engine_state->active_scene);
}

void clear_scene(int8_t s, UxState* state)
{
  for (int8_t c = 0; c < N_CHANNELS; c++)
  {
    reset_channel(&state->ux_setup->channels[c], state, s);
  }
}

void copy_scene_channel(int8_t c_src, int8_t s_src, int8_t c_dst, int8_t s_dst, UxState* state)
{
  if (c_dst >= N_ENCODERS || c_src >= N_ENCODERS || s_src >= N_SCENES || s_dst >= N_SCENES)
    return;
  memcpy(state->engine_config->channel_state[c_dst].params[s_dst], state->engine_config->channel_state[c_src].params[s_src],
         sizeof state->engine_config->channel_state[c_dst].params[s_dst]);
}

void assign_channel_to_channel(int8_t c_src, int8_t c_dst, UxState* state)
{
  copy_scene_channel(c_src, state->engine_state->active_scene, c_dst, state->engine_state->active_scene, state);
}

void assign_channel_to_scene(int8_t c_src, int8_t s_dst, UxState* state)
{
  copy_scene_channel(c_src, state->engine_state->active_scene, c_src, s_dst, state);
}

void assign_scene_to_scene(int8_t s_src, int8_t s_dst, UxState* state)
{
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    copy_scene_channel(c, s_src, c, s_dst, state);
  }
}

// Pointing a channel at itself means "stop triggering", which also switches
// quantizing off - otherwise the channel would sit in trig mode with no
// source and never update.
void assign_trig_src_use_channel(int8_t c_src, int8_t c_dst, UxState* state)
{
  if (c_src != c_dst)
  {
    state->engine_config->channel_state[c_src].src_trig = N_INPUTS + c_dst;
  }
  else
  {
    state->engine_config->channel_state[c_src].src_trig      = -1;
    state->engine_config->channel_state[c_src].quantize_mode = QUANTIZE_DISABLED;
  }
}

void assign_trig_src_use_input(int8_t c_src, int8_t i_dst, UxState* state) { state->engine_config->channel_state[c_src].src_trig = i_dst; }
