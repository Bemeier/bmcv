#ifndef INC_LIB_UXSTATE_H_
#define INC_LIB_UXSTATE_H_

#include "config.h"
#include "engine_state.h"
#include "hw_state.h"
#include "ui_input.h"
#include "ui_state.h"
#include "ux_setup.h"
#include <stdint.h>

// Where persisted presets go. The one place the core still needs a callback,
// because the backing store genuinely differs per host - FRAM on the module,
// patch JSON in a VCV Rack instance, browser storage in the web sim - and
// because it is a cold path (the SAV button and a 2s autosave), so the
// indirection costs nothing.
//
// A NULL PresetIo is valid and is what a plain unit test wants: store becomes
// a no-op and load reports "nothing stored", which is the first-boot path.
typedef struct
{
  int8_t (*store)(void* user, const EngineConfig* cfg, int8_t slot);
  int8_t (*load)(void* user, EngineConfig* cfg, int8_t slot);

  // Forget every stored preset. Optional: a NULL clear is storage that cannot
  // be wiped, which is what a test with no backing store has, and what a host
  // that does not want its slots reachable from outside can offer.
  int8_t (*clear)(void* user);

  void* user;
} PresetIo;

// Composition root: the wiring that lets any layer reach the pieces it needs.
// The layers themselves are separate - hardware (HwState), persisted config
// (EngineConfig), signal path (EngineState) and interaction (UiState).
typedef struct UxState
{
  const UxSetup* ux_setup;
  const HwSetup* hw_setup;
  HwState* hw_state;
  EngineConfig* engine_config;
  EngineState* engine_state;
  UiState* ui;
  const PresetIo* presets;

  // Autosave bookkeeping. Here because this struct owns the preset io, and the
  // write is a cold path on a 2s timer. It used to live in the input layer,
  // which had nothing to do with it beyond running often enough.
  uint32_t last_autosave_us;
  uint32_t last_crc;
} UxState;

// Preset access that tolerates a NULL PresetIo, so no call site has to check.
// store returns non-zero when the config actually reached the backing store.
int8_t ux_preset_store(const UxState* state, int8_t slot);
int8_t ux_preset_load(UxState* state, int8_t slot);

// Capture the config CRC the autosave compares against, so a freshly loaded
// config is not written straight back out. Call once the pointers are wired
// and the stored config has been loaded.
void ux_autosave_init(UxState* state, uint32_t now_us);

// One UX pass: age the timers, run every handler, render. now_us is the tick
// timestamp; the elapsed time the timers use is ui->in.dt, accumulated since
// the last pass.
void ux_update(UxState* state, uint32_t now_us);

// The only writer of channels_last_delta: records that the user is actively
// turning this channel's encoder, which channel_compute uses to decide
// whether a stepped-pattern length change may apply mid-cycle.
void ux_note_channel_edit(UxState* state, uint8_t channel);

#endif /* INC_LIB_UXSTATE_H_ */
