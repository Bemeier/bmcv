#include "instance.h"
#include "channel.h"
#include "clock_sync.h"
#include "config.h"
#include "config_validate.h"
#include "engine.h"
#include "ui_mode.h"
#include "ui_select.h"
#include <string.h>

void bmcv_instance_wire(BmcvInstance* m, const PresetIo* io)
{
  m->hw_setup = HwSetup_Get();
  m->ux_setup = UxSetup_InitFromHw(m->hw_setup);

  m->ux.hw_setup        = m->hw_setup;
  m->ux.ux_setup        = m->ux_setup;
  m->ux.engine_config   = &m->engine_config;
  m->ux.engine_state    = &m->engine_state;
  m->ux.channel_scratch = m->channel_scratch;
  m->ux.ui              = &m->ui_state;
  m->ux.presets         = io;

  // input_frames_init sets this too, as part of baselining the input layer.
  // Here as well because this is the list of every pointer in the instance, and
  // a list with a hole in it is worse than no list: an imported instance whose
  // hw_state still pointed into another module's RAM would read plausible
  // rubbish rather than fail.
  m->ux.hw_state = &m->input.curr;
}

void bmcv_instance_init(BmcvInstance* m, const PresetIo* io, uint32_t now_us)
{
  memset(m, 0, sizeof(*m));

  bmcv_instance_wire(m, io);

  Clock_Init(&m->engine_state.clock);
  midi_out_init(&m->midi_out);

  // No selected_param here: it comes out of the stored config below, or out of
  // config_defaults when there is none, so that a module comes back on the page
  // it was left on rather than on a fixed one.
  m->ui_state.shift_state     = SHIFT_STATE_NONE;
  m->ui_state.momentary_scene = -1; // 0 would read as "scene 0 held"
  m->ui_state.page_entry_btn  = -1; // 0 would read as "the STA button is down"
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

  // After engine_tick, so it reads the levels this tick produced and the clock
  // events it acted on rather than the previous tick's.
  midi_out_publish(&m->midi_out, &m->engine_state, &m->input.curr, now_us);

  return dirty;
}

uint8_t bmcv_instance_take_command(BmcvInstance* m, const PresetIo* io, uint32_t now_us)
{
  if (m->command.seq == m->command_seq)
    return 0;

  // Read out before anything happens: bmcv_instance_init zeroes the instance,
  // the mailbox included, and the acknowledgement has to survive that or the
  // same command reads as new on the next pass and the module resets for ever.
  const uint32_t seq = m->command.seq;
  const uint8_t op   = m->command.op;

  switch (op)
  {
  case REMOTE_OP_RESET_WIPE:
    // Before the re-init, so the load it performs finds nothing and the module
    // comes back on its first-boot defaults rather than reloading what was just
    // meant to be forgotten.
    if (io && io->clear)
      io->clear(io->user);
    bmcv_instance_init(m, io, now_us);
    break;

  case REMOTE_OP_RESET:
    bmcv_instance_init(m, io, now_us);
    break;

  default:
    break; // an op this build does not know is acknowledged and ignored
  }

  m->command.seq = seq;
  m->command.op  = op;
  m->command_seq = seq;
  return 1;
}
