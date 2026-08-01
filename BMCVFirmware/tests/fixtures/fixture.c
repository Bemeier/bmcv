#include "fixture.h"
#include "assign.h"
#include "channel.h"
#include "clock_sync.h"
#include "engine.h"
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
  f->ux.ui            = &f->ui_state;

  f->engine_config.scene_a       = 0;
  f->engine_config.scene_b       = 0;
  f->engine_config.quantize_mask = 0b111111111111;
  f->ui_state.momentary_scene    = -1; // 0 would be misread as "scene 0 held" (see compute_scenes_contribution)
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

  // Mirror bmcv_state_update's input_dirty: a button level change or any
  // encoder movement forces the UX pass, otherwise it waits for the interval.
  uint8_t dirty = 0;
  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    dirty += f->hw_state.button_state[b] != f->prev_button_state[b];
    f->prev_button_state[b] = f->hw_state.button_state[b];
  }
  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    dirty += f->hw_state.encoder_delta[e] != 0;
  }

  // The real thing - same function bmcv_main() calls on hardware.
  engine_tick(&f->ux, f->hw_state.time, dirty);

  // encoder_delta is a per-tick quantity on hardware, recomputed from the
  // encoder position each time round; it does not persist.
  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    f->hw_state.encoder_delta[e] = 0;
  }
}

void fixture_hold(Fixture* f, int8_t button, uint32_t hold_us)
{
  f->hw_state.button_state[button] = 1;
  fixture_tick(f, MS(1)); // the DOWN edge

  for (uint32_t elapsed = 0; elapsed < hold_us; elapsed += MS(1))
  {
    fixture_tick(f, MS(1));
  }
}

void fixture_release(Fixture* f, int8_t button)
{
  f->hw_state.button_state[button] = 0;
  fixture_tick(f, MS(1));
}

void fixture_press(Fixture* f, int8_t button, uint32_t hold_us)
{
  fixture_hold(f, button, hold_us);
  fixture_release(f, button);
}

void fixture_encoder(Fixture* f, int8_t encoder, int16_t delta)
{
  f->hw_state.encoder_delta[encoder] = delta;
  fixture_tick(f, MS(1));
}

void fixture_set_param(Fixture* f, uint8_t ch, uint8_t scene, ChannelParameters param, int16_t value)
{
  f->engine_config.channel_state[ch].params[scene][param] = value;
}
