#include "fixture.h"
#include "assign.h"
#include "channel.h"
#include "engine.h"
#include "clock_sync.h"
#include "scene.h"
#include <string.h>

void fixture_init(Fixture* f)
{
  memset(f, 0, sizeof(*f));

  f->hw_setup = HwSetup_Get();
  f->ux_setup = UxSetup_InitFromHw(f->hw_setup);

  f->ux.hw_setup      = f->hw_setup;
  f->ux.ux_setup      = f->ux_setup;
  f->ux.hw_state      = &f->hw_state;
  f->ux.engine_config = &f->engine_config;
  f->ux.engine_state  = &f->engine_state;

  f->engine_config.scene_a       = 0;
  f->engine_config.scene_b       = 0;
  f->engine_config.quantize_mask = 0b111111111111;
  f->engine_state.momentary_scene = -1; // 0 would be misread as "scene 0 held" (see compute_scenes_contribution)
  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    f->engine_config.channel_state[c].src_input = -1;
    f->engine_config.channel_state[c].src_trig  = -1;
  }

  f->hw_state.slider_state = SLIDER_MIN_VALUE;

  assign_reset(&f->ux); // assign_src_id must start at -1, not 0
  Clock_Init();         // clears the global clock, including its smoothing estimate

  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    init_channel(&f->ux_setup->channels[c], &f->ux);
    reset_channel(&f->ux_setup->channels[c], &f->ux, -1);
  }
}

void fixture_tick(Fixture* f, uint32_t dt_us)
{
  f->hw_state.dt = dt_us;
  f->hw_state.time += dt_us;

  // The real thing - same function bmcv_main() calls on hardware.
  engine_tick(&f->ux, f->hw_state.time, /*input_dirty=*/0);
}

void fixture_set_param(Fixture* f, uint8_t ch, uint8_t scene, ChannelParameters param, int16_t value)
{
  f->engine_config.channel_state[ch].params[scene][param] = value;
}
