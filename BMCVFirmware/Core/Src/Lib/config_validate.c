#include "config_validate.h"
#include "helpers.h"
#include "hw_setup.h"

// -1 is the established "unassigned" sentinel for routing fields, and the
// consumers in channel.c already guard on >= 0. So anything out of range
// becomes unassigned rather than being clamped onto a real source, which
// would silently route a channel somewhere the user never chose.
static inline int8_t valid_source_or_unassigned(int8_t value, int8_t count) { return (value >= 0 && value < count) ? value : -1; }

// What a module that has never been saved to comes up with. Here rather than
// in the composition root, next to the validation that has to accept it, and
// testable on its own.
void config_defaults(EngineConfig* cfg)
{
  cfg->input_mode[0] = INPUT_CLOCK;
  cfg->input_mode[1] = INPUT_RESET;
  cfg->input_mode[2] = INPUT_DEFAULT;
  cfg->input_mode[3] = INPUT_DEFAULT;

  cfg->scene_a       = 0;
  cfg->scene_b       = N_SCENES - 1;
  cfg->quantize_mask = 0x0FFFu; // every semitone enabled

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    cfg->channel_state[c].src_input     = -1;
    cfg->channel_state[c].src_trig      = -1;
    cfg->channel_state[c].quantize_mode = QUANTIZE_DISABLED;
  }
}

void config_validate(EngineConfig* cfg)
{
  cfg->scene_a = (uint8_t) iclamp(cfg->scene_a, 0, N_SCENES - 1);
  cfg->scene_b = (uint8_t) iclamp(cfg->scene_b, 0, N_SCENES - 1);

  // Only the 12 semitone bits carry meaning; stray high bits would show up as
  // enabled notes that no button can turn off.
  cfg->quantize_mask &= 0x0FFFu;

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    cfg->input_mode[i] = (InputMode) iclamp(cfg->input_mode[i], 0, INPUT_MODE_COUNT - 1);
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    ChannelConfig* ch = &cfg->channel_state[c];

    ch->shape_mode     = (int8_t) iclamp(ch->shape_mode, 0, SHAPE_MODE_COUNT - 1);
    ch->quantize_mode  = (ChannelQuantizeMode) iclamp(ch->quantize_mode, 0, QUANTIZE_MODE_COUNT - 1);
    ch->input_amp_mode = (ChannelInputAmpMode) iclamp(ch->input_amp_mode, 0, INPUT_AMP_MODE_COUNT - 1);

    ch->src_input = valid_source_or_unassigned(ch->src_input, N_INPUTS);
    ch->src_trig  = valid_source_or_unassigned(ch->src_trig, N_INPUTS + N_CHANNELS);
  }
}
