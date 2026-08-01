#ifndef INC_LIB_UI_STATE_H_
#define INC_LIB_UI_STATE_H_

#include "hw_setup.h"
#include "state.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_select.h"
#include <stdint.h>

// Everything the interaction layer owns: what mode the user is in, what they
// have selected, what is being shown back to them.
//
// Deliberately separate from EngineState, which is now only signal path -
// phases, output levels, trigger edges, scene blend. The two were one struct,
// so nothing stopped a DSP function reaching for shift_state or a renderer
// mutating a phase accumulator. The split is what lets the UI be reasoned
// about and tested without the engine, and vice versa.
//
// None of this is persisted: EngineConfig is the FRAM record and is untouched
// by the UI layer except through the explicit mutation functions.
typedef struct UiState
{
  uint8_t shift_state;      // ShiftStates
  uint8_t prev_shift_state; // to spot mode entry
  uint8_t selected_param;   // ChannelParameters
  int8_t momentary_scene;   // -1 when no scene button is held

  // Pending two-step action (copy, routing). See ui_select.h.
  Selection sel;

  // Confirmation flashes for committed actions. See ui_feedback.h.
  Feedback fb;

  // Output mute, per channel. Not in EngineConfig on purpose: adding a field
  // to ChannelConfig would move the FRAM record layout and invalidate saved
  // presets, and booting into a muted channel is not wanted behaviour.
  uint8_t muted[N_CHANNELS];

  // Transient value display: how much longer this element should show the
  // value being edited instead of its base state.
#define UI_EDIT_DISPLAY MS(1000)
  uint32_t channels_edit_hold[N_CHANNELS];
  uint8_t channels_edit_hue[N_CHANNELS];

  uint8_t blink_slow;
  uint8_t blink_fast;

  UiInput in;
} UiState;

#endif /* INC_LIB_UI_STATE_H_ */
