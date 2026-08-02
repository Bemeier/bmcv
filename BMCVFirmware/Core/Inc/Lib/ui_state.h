#ifndef INC_LIB_UI_STATE_H_
#define INC_LIB_UI_STATE_H_

#include "config.h"
#include "engine_state.h"
#include "helpers.h"
#include "hw_setup.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"
#include <stdint.h>

// Blink rates for anything that pulses, in microseconds. Presentation, so they
// live with the rest of the interaction layer rather than with the engine.
#define FAST_BLINK_PERIOD 300000
#define SLOW_BLINK_PERIOD 800000

// How long an element keeps showing the value being edited before decaying
// back to its base state.
#define UI_EDIT_DISPLAY MS(1000)

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
  uint8_t shift_state;       // ShiftStates
  uint8_t prev_shift_state;  // to spot mode entry
  uint8_t exit_consumed_tap; // this tick's tap left a mode; it selects no param
  uint8_t selected_param;    // ChannelParameters
  int8_t momentary_scene;    // -1 when no scene button is held

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
  uint32_t channels_edit_hold[N_CHANNELS];
  uint8_t channels_edit_hue[N_CHANNELS];

  uint8_t blink_slow;
  uint8_t blink_fast;

  UiInput in;
} UiState;

// Arm the transient value display on one channel: for UI_EDIT_DISPLAY it shows
// whatever the current mode's encoder edits instead of its base state. Handlers
// write it, the renderer reads it. A helper rather than the bare assignment it
// replaces, which appeared at five call sites with no bounds check.
static inline void ui_show_channel_edit(UiState* ui, uint8_t channel)
{
  if (channel < N_CHANNELS)
    ui->channels_edit_hold[channel] = UI_EDIT_DISPLAY;
}

// Every channel at once. Used on shift-mode entry, so the mode's per-channel
// state is legible at a glance before decaying back to the output level.
static inline void ui_show_all_channel_edits(UiState* ui)
{
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    ui->channels_edit_hold[c] = UI_EDIT_DISPLAY;
  }
}

#endif /* INC_LIB_UI_STATE_H_ */
