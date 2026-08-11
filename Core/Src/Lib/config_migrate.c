#include "config_migrate.h"
#include "config_validate.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Historical layouts
//
// Each is the EngineConfig as that version's config.h declared it - the same
// fields in the same order with the same types, enum-typed members included,
// because an enum's width is the compiler's business and has to be reproduced
// rather than assumed. (It is not the same everywhere: arm-none-eabi defaults
// to -fshort-enums and a host compiler does not, so ChannelConfig is 91 bytes
// on the module and 97 in a test. That is exactly why these are declared this
// way instead of by byte offset - each target reproduces its own history.)
// ---------------------------------------------------------------------------

// v2, up to and including "Review pass: enforce the layering". ChannelConfig
// had not yet gained sr_length_idx or clamp_mode, and CH_PARAM_AMP was half the
// swing it now is - channel.c multiplied it by 0.5.
typedef struct __attribute__((packed))
{
  int8_t src_input;
  int8_t src_trig;
  int8_t shape_mode;
  ChannelInputAmpMode input_amp_mode;
  ChannelQuantizeMode quantize_mode;
  int16_t params[N_SCENES][CH_PARAM_COUNT];
} ChannelConfigV2;

typedef struct __attribute__((packed))
{
  uint8_t clock_div;
  uint8_t scene_a;
  uint8_t scene_b;
  uint8_t current_preset;
  uint16_t quantize_mask;
  InputMode input_mode[N_INPUTS];
  ChannelConfigV2 channel_state[N_CHANNELS];
} EngineConfigV2;

// v3 and v4 share one layout: the current EngineConfig without its trailing
// selected_param. What changed at v4 is what shape_mode's numbers mean, which
// no amount of layout checking would catch; what changed at v5 is that the
// selected parameter became part of the patch. ChannelConfig itself has not
// moved since v3, so this reuses it rather than restating it.
typedef struct __attribute__((packed))
{
  uint8_t clock_div;
  uint8_t scene_a;
  uint8_t scene_b;
  uint8_t current_preset;
  uint16_t quantize_mask;
  InputMode input_mode[N_INPUTS];
  ChannelConfig channel_state[N_CHANNELS];
} EngineConfigV4;

// The one thing v5 added. A record written before it had no selected
// parameter at all, so it gets the same default a module with no stored
// config gets, rather than CH_PARAM_FRQ by way of a zeroed byte.
static void migrate_v4_to_v5(const EngineConfigV4* old, EngineConfig* cfg)
{
  cfg->clock_div      = old->clock_div;
  cfg->scene_a        = old->scene_a;
  cfg->scene_b        = old->scene_b;
  cfg->current_preset = old->current_preset;
  cfg->quantize_mask  = old->quantize_mask;

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    cfg->input_mode[i] = old->input_mode[i];
  }
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    cfg->channel_state[c] = old->channel_state[c];
  }

  cfg->selected_param = CH_PARAM_OFS;
}

// What changed at v4 is what shape_mode's numbers mean.
//
// v3 carried the stepped algorithm three times over, at three hold values:
//
//   0 LFO   1 STEPPED_SMOOTH   2 STEPPED_SEMI   3 STEPPED_HARD   4 PWM
//
// All three collapse onto SHAPE_STEPPED. Hold is not a per-channel setting yet,
// so a channel that was SEMI or HARD comes back as the one hold value that
// remains: the shape survives, its hold does not. Better than the alternatives,
// which are to lose the whole patch or to leave it pointing at PWM.
static const int8_t shape_v3_to_v4[] = {SHAPE_LFO, SHAPE_STEPPED, SHAPE_STEPPED, SHAPE_STEPPED, SHAPE_PWM};

static int8_t migrate_shape_mode(int8_t old_mode)
{
  if (old_mode < 0 || (size_t) old_mode >= sizeof(shape_v3_to_v4))
  {
    return SHAPE_LFO;
  }
  return shape_v3_to_v4[old_mode];
}

static void migrate_v3_to_v4(EngineConfigV4* cfg)
{
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    cfg->channel_state[c].shape_mode = migrate_shape_mode(cfg->channel_state[c].shape_mode);
  }
}

static void migrate_v2_to_v3(const EngineConfigV2* old, EngineConfigV4* cfg)
{
  memset(cfg, 0, sizeof(*cfg));

  cfg->clock_div      = old->clock_div;
  cfg->scene_a        = old->scene_a;
  cfg->scene_b        = old->scene_b;
  cfg->current_preset = old->current_preset;
  cfg->quantize_mask  = old->quantize_mask;

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    cfg->input_mode[i] = old->input_mode[i];
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    const ChannelConfigV2* och = &old->channel_state[c];
    ChannelConfig* ch          = &cfg->channel_state[c];

    ch->src_input      = och->src_input;
    ch->src_trig       = och->src_trig;
    ch->shape_mode     = och->shape_mode;
    ch->input_amp_mode = och->input_amp_mode;
    ch->quantize_mode  = och->quantize_mode;

    // The two fields v3 appended. Zero is the default for both - the shortest
    // pattern length and the widest output clamp - which is what a channel did
    // before either setting existed.
    ch->sr_length_idx = 0;
    ch->clamp_mode    = CLAMP_BI_10;

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
      for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
      {
        ch->params[s][p] = och->params[s][p];
      }

      // AMP is the peak swing now; it used to be half of it. Halving the stored
      // value is what makes an old patch come back at the level it was dialled
      // in at rather than twice it.
      ch->params[s][CH_PARAM_AMP] = (int16_t) (och->params[s][CH_PARAM_AMP] / 2);
    }
  }
}

// Deliberately in the way. Bumping the version without deciding what happens to
// the records already written is how eight slots got thrown away three times;
// this makes that a compile error rather than a discovery on someone's bench.
_Static_assert(CONFIG_STATE_VERSION == 5, "record format changed: add the migration below, then update this assert");

int8_t config_migrate(uint16_t version, uint16_t length, const void* data, EngineConfig* out)
{
  if (data == NULL || out == NULL)
  {
    return 0;
  }

  // Each arm brings the record up to the v4 layout and then hands it to
  // migrate_v4_to_v5, rather than each one knowing how to produce the current
  // struct. v3 and v4 are the same bytes, so the only difference between those
  // two arms is the shape-mode remap between them.
  switch (version)
  {
  case CONFIG_STATE_VERSION:
    if (length != sizeof(EngineConfig))
      return 0;
    memcpy(out, data, sizeof(EngineConfig));
    break;

  case 4:
  {
    if (length != sizeof(EngineConfigV4))
      return 0;
    EngineConfigV4 old;
    memcpy(&old, data, sizeof(old));
    migrate_v4_to_v5(&old, out);
    break;
  }

  case 3:
  {
    if (length != sizeof(EngineConfigV4))
      return 0;
    EngineConfigV4 old;
    memcpy(&old, data, sizeof(old));
    // Before config_validate, which would clamp SEMI and HARD onto PWM - the
    // right numeric range and the wrong shape.
    migrate_v3_to_v4(&old);
    migrate_v4_to_v5(&old, out);
    break;
  }

  case 2:
  {
    if (length != sizeof(EngineConfigV2))
      return 0;
    EngineConfigV2 old;
    EngineConfigV4 v4;
    memcpy(&old, data, sizeof(old));
    migrate_v2_to_v3(&old, &v4);
    migrate_v3_to_v4(&v4);
    migrate_v4_to_v5(&v4, out);
    break;
  }

  default:
    return 0;
  }

  // Whatever path it took, what comes out has to be safe to index with. An old
  // record can hold a value this build has no meaning for even when its layout
  // is understood perfectly.
  config_validate(out);
  return 1;
}
