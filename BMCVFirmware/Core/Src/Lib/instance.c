#include "instance.h"
#include "channel.h"
#include "clock_sync.h"
#include "config.h"
#include "config_validate.h"
#include "engine.h"
#include "ui_mode.h"
#include "ui_select.h"
#include <string.h>

void bmcv_instance_init(BmcvInstance* m, const PresetIo* io, uint32_t now_us)
{
  memset(m, 0, sizeof(*m));

  m->hw_setup = HwSetup_Get();
  m->ux_setup = UxSetup_InitFromHw(m->hw_setup);

  m->ux.hw_setup      = m->hw_setup;
  m->ux.ux_setup      = m->ux_setup;
  m->ux.engine_config = &m->engine_config;
  m->ux.engine_state  = &m->engine_state;
  m->ux.ui            = &m->ui_state;
  m->ux.presets       = io;

  Clock_Init(&m->engine_state.clock);

  m->ui_state.selected_param  = CH_PARAM_SHP;
  m->ui_state.shift_state     = SHIFT_STATE_NONE;
  m->ui_state.momentary_scene = -1; // 0 would read as "scene 0 held"
  ui_sel_reset(&m->ui_state);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    channel_init(c, &m->engine_state);
  }

  if (!ux_preset_load(&m->ux, CONFIG_AUTOSAVE_SLOT))
  {
    // No stored config is the normal state of a module that has not been used
    // yet, not a fault. This used to raise error bit 6, which render_error
    // draws by blanking every LED and blinking scene 6 until the next
    // interaction - a startup screen saying "nothing is wrong". A genuine
    // failure to read a slot the user explicitly asked for still reports, in
    // scene.c.
    config_defaults(&m->engine_config);

    // channel_reset also re-runs channel_init, which is harmless and keeps
    // "reset a channel" one operation rather than two the caller must pair.
    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
      channel_reset(c, &m->engine_state, &m->engine_config, -1);
    }
  }

  // Holds for the defaults above as well as a loaded preset, so the rest of
  // the firmware can index on these fields unconditionally.
  config_validate(&m->engine_config);

  input_frames_init(&m->input, &m->ux, now_us);

  // After the config is settled, so the freshly loaded one is not written
  // straight back out on the first autosave interval.
  ux_autosave_init(&m->ux, now_us);
}

uint8_t bmcv_instance_tick(BmcvInstance* m, const InputSample* sample, uint32_t now_us)
{
  uint8_t dirty = input_fold(&m->input, &m->ux, sample, now_us);
  engine_tick(&m->ux, now_us, dirty);
  return dirty;
}
