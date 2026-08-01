#ifndef INC_LIB_UXSTATE_H_
#define INC_LIB_UXSTATE_H_

#include "state.h"
#include "ui_input.h"
#include "ui_state.h"
#include "ux_setup.h"
#include <stdint.h>

// Composition root: the wiring that lets any layer reach the pieces it needs.
// The layers themselves are separate - hardware (HwState), persisted config
// (EngineConfig), signal path (EngineState) and interaction (UiState).
typedef struct UxState
{
  const UxSetup* ux_setup;
  const HwSetup* hw_setup;
  uint32_t last_ux_update;
  uint32_t dt;
  HwState* hw_state;
  EngineConfig* engine_config;
  EngineState* engine_state;
  UiState* ui;
} UxState;

void update_ux_state(UxState* state);

// The only writer of channels_last_delta: records that the user is actively
// turning this channel's encoder, which compute_channel uses to decide
// whether a stepped-pattern length change may apply mid-cycle.
void ui_channel_note_edit(UxState* state, uint8_t channel);

#endif /* INC_LIB_UXSTATE_H_ */
