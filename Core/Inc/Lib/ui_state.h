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

// Blink rate for anything that pulses, in microseconds. Presentation, so it
// lives with the rest of the interaction layer rather than with the engine.
#define SLOW_BLINK_PERIOD 800000

// How long a held press drops out for as it crosses one of its thresholds.
// A dip, not a blink: what is wanted is "that registered", once, at the moment
// it becomes true - a pulse that keeps going says something is still happening.
#define UI_HELD_DIP MS(90)

// The assignment marker is not a blink but a mark: mostly off, with a short
// flash on a long period. That is what lets an element keep showing its own
// state and still say "you can pick me" - a 50% duty blink cannot, because for
// half the time the element is showing the wrong thing.
#define MARK_BLINK_PERIOD 1600000
#define MARK_BLINK_ON 200000

// How long the encoders keep showing the selected parameter before decaying
// back to the output level.
#define UI_EDIT_DISPLAY MS(2000)

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
  int8_t momentary_scene;    // -1 when no scene button is held

  // The selected parameter is deliberately NOT here: it is saved with the
  // patch, so it lives in EngineConfig. See the note beside it there.

  // Pending two-step action (copy, routing). See ui_select.h.
  Selection sel;

  // Confirmation flashes for committed actions. See ui_feedback.h.
  Feedback fb;

  // Output mute, per channel. Not in EngineConfig on purpose: adding a field
  // to ChannelConfig would move the FRAM record layout and invalidate saved
  // presets, and booting into a muted channel is not wanted behaviour.
  uint8_t muted[N_CHANNELS];

  // Transient value display: how much longer the encoders should show the
  // selected parameter instead of the output level.
  //
  // One timer for all eight, not one each: the point of showing it is to
  // compare the channels against each other, which needs them all lit at once.
  uint32_t param_display_hold;

  uint8_t blink_slow;
  uint8_t blink_mark; // short, on a long period; see MARK_BLINK_ON

  UiInput in;
} UiState;

// Show the selected parameter on every encoder for UI_EDIT_DISPLAY. Armed by
// turning any channel's encoder and by picking a parameter; read by the
// renderer.
//
// Only no-mode needs it. A shift mode's channel LEDs show that mode's setting
// for as long as the mode is active (ChannelBaseLayer), so there is nothing to
// reveal and decay back from - and arming this on mode *entry* is what used to
// flash every encoder red on the way back out.
static inline void ui_show_param_display(UiState* ui) { ui->param_display_hold = UI_EDIT_DISPLAY; }

#endif /* INC_LIB_UI_STATE_H_ */
