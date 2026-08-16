#include "assign.h"
#include "channel.h"
#include "config.h"
#include "hw_setup.h"
#include <stdint.h>
#include <string.h>

// Every mutation here is reachable from a button press, so each one validates
// its own indices rather than trusting the caller. ui_select.c does pass ids
// straight through from the setup tables today, but that is the wrong thing to
// rely on - and it was already inconsistent, with some of these checked and
// some not.

static int channel_ok(int8_t c) { return c >= 0 && c < N_CHANNELS; }
static int scene_ok(int8_t s) { return s >= 0 && s < N_SCENES; }
static int input_ok(int8_t i) { return i >= 0 && i < N_INPUTS; }

void assign_input_to_channel(int8_t i, int8_t c, UxState* state)
{
  if (!channel_ok(c) || !input_ok(i))
    return;
  state->engine_config->channel_state[c].src_input = i;
}

// MIX: pressing the held source channel again unroutes its input.
void assign_input_clear(int8_t c, UxState* state)
{
  if (!channel_ok(c))
    return;
  state->engine_config->channel_state[c].src_input = -1;
}

// QNT: choosing a channel as the thing to be triggered implies the mode it
// will be triggered in. Without this the assignment would have no audible
// effect until the encoder was also turned to reach QUANTIZE_TRIG_SRC.
void assign_trig_arm_channel(int8_t c, UxState* state)
{
  if (!channel_ok(c))
    return;
  state->engine_config->channel_state[c].quantize_mode = QUANTIZE_TRIG_SRC;
}

// A tap clears the scene in front of you; a hold clears the whole channel.
//
// "Whole" means the per-channel settings too - what MIX routed into it, what
// QNT samples it, its shape mode and pattern length - and not just its
// parameters in every scene. Those settings are as much part of a channel as
// its numbers are, and a channel that reads as cleared while still being
// modulated by an input is the confusing half-state this closes.
//
// The output clamp is the exception, deliberately: it describes what the module
// is patched into rather than what the patch is, and having it survive is the
// whole reason it is per channel and not per scene.
void clear_channel(int8_t c, int8_t all_scenes, UxState* state)
{
  if (!channel_ok(c))
    return;

  channel_reset(c, state->engine_state, state->engine_config, all_scenes ? -1 : state->engine_state->active_scene);

  if (!all_scenes)
    return;

  ChannelConfig* ch  = &state->engine_config->channel_state[c];
  ch->src_input      = -1;
  ch->src_trig       = -1;
  ch->input_amp_mode = INPUT_AMP_DISABLED;
  ch->quantize_mode  = QUANTIZE_DISABLED;
  ch->shape_mode     = SHAPE_LFO;
  ch->st_length_idx  = 0;
}

void clear_scene(int8_t s, UxState* state)
{
  if (!scene_ok(s))
    return;
  for (int8_t c = 0; c < N_CHANNELS; c++)
  {
    channel_reset((uint8_t) c, state->engine_state, state->engine_config, s);
  }
}

static void copy_scene_channel(int8_t c_src, int8_t s_src, int8_t c_dst, int8_t s_dst, UxState* state)
{
  if (!channel_ok(c_src) || !channel_ok(c_dst) || !scene_ok(s_src) || !scene_ok(s_dst))
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
  for (int8_t c = 0; c < N_CHANNELS; c++)
  {
    copy_scene_channel(c, s_src, c, s_dst, state);
  }
}

// Pointing a channel at itself means "stop triggering", which also switches
// quantizing off - otherwise the channel would sit in trig mode with no
// source and never update.
void assign_trig_src_use_channel(int8_t c_src, int8_t c_dst, UxState* state)
{
  if (!channel_ok(c_src) || !channel_ok(c_dst))
    return;

  if (c_src != c_dst)
  {
    state->engine_config->channel_state[c_src].src_trig = TRIG_SRC_CHANNEL(c_dst);
  }
  else
  {
    state->engine_config->channel_state[c_src].src_trig      = -1;
    state->engine_config->channel_state[c_src].quantize_mode = QUANTIZE_DISABLED;
  }
}

void assign_trig_src_use_input(int8_t c_src, int8_t i_dst, UxState* state)
{
  if (!channel_ok(c_src) || !input_ok(i_dst))
    return;
  state->engine_config->channel_state[c_src].src_trig = TRIG_SRC_INPUT(i_dst);
}
