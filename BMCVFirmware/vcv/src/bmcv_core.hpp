#pragma once

// One place where the firmware's C headers cross into C++.
//
// The core is plain C and knows nothing about Rack, so every header it needs
// is pulled in here rather than in thirty places with thirty extern "C"
// blocks. Nothing in this file is BMCV logic - it is a language boundary.

extern "C"
{
#include "config.h"       // EngineConfig, FRAM_CONFIG_SLOTS, CONFIG_AUTOSAVE_SLOT
#include "engine_state.h" // what the engine publishes: levels, LEDs
#include "hw_setup.h"     // the index tables and the converter ranges
#include "input_fold.h"   // InputSample
#include "instance.h"     // BmcvInstance
#include "led_color.h"    // framebuffer bytes -> what a screen should show
#include "panel_layout.h" // generated geometry and lettering
#include "sim_rt.h"       // tick decimation, gate latching, volt conversion
#include "slot_store.h"   // the preset slots, in memory
}
