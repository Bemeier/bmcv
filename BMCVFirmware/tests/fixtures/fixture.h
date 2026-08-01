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

#endif /* BMCV_TEST_FIXTURE_H_ */
