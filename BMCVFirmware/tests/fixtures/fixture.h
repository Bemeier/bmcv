// Builds a fully-wired UxState (HwSetup + UxSetup + zeroed HwState/
// EngineConfig/EngineState) so tests and tools can call compute_channel(),
// compute_scenes_contribution(), etc. exactly as bmcv_main() does, without
// any hardware behind it.
#ifndef BMCV_TEST_FIXTURE_H_
#define BMCV_TEST_FIXTURE_H_

#include "ux_state.h"

typedef struct
{
  const HwSetup* hw_setup;
  const UxSetup* ux_setup;
  HwState hw_state;
  EngineConfig engine_config;
  EngineState engine_state;
  UxState ux;

  // So fixture_tick can derive input_dirty the same way bmcv_state_update
  // does - the UX layer only runs on a level change or the 8ms interval, and
  // tests need that timing to be realistic.
  uint8_t prev_button_state[N_BUTTONS];
} Fixture;

// Zeroes state, wires up HwSetup/UxSetup, and resets every channel to its
// default params. scene_a == scene_b == 0, so fixture_tick's scene-blend
// step puts scene 0 fully active until a test changes scene_a/scene_b or
// the slider position.
void fixture_init(Fixture* f);

// Advances hw_state time by dt_us, recomputes the scene crossfade, then
// ticks compute_channel() for every channel (mirrors bmcv_main()'s DSP
// block). Channel outputs land in f->engine_state.channels_output_level[].
void fixture_tick(Fixture* f, uint32_t dt_us);

// Convenience: set one parameter of one channel in the given scene.
void fixture_set_param(Fixture* f, uint8_t ch, uint8_t scene, ChannelParameters param, int16_t value);

// Button gestures, driven at the hardware level (button_state) so they go
// through ui_input_update and the UX rate limit exactly as on the module.

// Press and keep holding for hold_us, ticking at a realistic rate throughout.
void fixture_hold(Fixture* f, int8_t button, uint32_t hold_us);

// Release a held button. The level change makes the UX layer run immediately.
void fixture_release(Fixture* f, int8_t button);

// hold + release.
void fixture_press(Fixture* f, int8_t button, uint32_t hold_us);

// Feed encoder movement for one tick.
void fixture_encoder(Fixture* f, int8_t encoder, int16_t delta);

#endif /* BMCV_TEST_FIXTURE_H_ */
