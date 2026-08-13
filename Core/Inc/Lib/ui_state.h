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

// How long the encoders keep showing the selected parameter before decaying
// back to the output level.
#define UI_EDIT_DISPLAY MS(2000)

// A ring that shows a division pulses at the rate that division produces, so
// the row says which channels are fast as well as which division each one is
// on. Two pages do it: FRQ while it is being shown, and the stepped-random
// pattern length, where the rate is the channel's own multiplied by the number
// of steps.
//
// The pulse dims and never brightens: VAL_BASE is the peak, so the FRQ ring
// keeps the same ceiling as every other base layer. It goes most of the way
// down from there - a shallow breath was not readable on the hardware, and the
// rate is the whole reason the pulse is there.
//
// Not to zero, though: the trough still has to show the hue and the saturation,
// which are the other two facts the ring is carrying.
#define RING_PULSE_V_MIN 6
#define RING_PULSE_V_MAX VAL_BASE

// The peak also runs slightly warm, which is what a filament does and reads as
// more contrast than brightness alone gives.
//
// Small on purpose, and this is the number that cannot be turned up freely:
// quintuplet (orange, 30) and triplet (yellow, 43) are only 13 apart, so a
// swing approaching that makes a quintuplet at its peak the same colour as a
// triplet at its trough - and the hue is carrying which division this is.
// Below half that gap, the classes stay apart at every phase.
#define RING_PULSE_HUE_SWING 4

// Above this the pulse stops following the oscillator and free-runs here
// instead. Sampling a faster phase than the panel is drawn at aliases it into a
// slow phantom pulse - which would draw the fastest channel on the row as one
// of the slowest. Everything past the ceiling shimmers together instead, which
// at least reads as "all of these are off the top of the scale".
//
// The rate that bounds this is the *render* rate, not engine_state.led_fps.
// The flush measures ~300Hz, but ui_render only rebuilds the framebuffer once
// per UX_UPDATE_INTERVAL - 125Hz - so most of those flushes re-send a frame
// that has not changed. Sampling is what aliases, and the sampling happens in
// ui_render.
//
// 25Hz is five samples per cycle at that rate, and is also about where the eye
// stops resolving a pulse this shallow and starts fusing it into a steady glow
// - so it is the perceptual limit as much as the sampling one. Raising it
// further means shortening UX_UPDATE_INTERVAL first; led_fps has headroom to
// spare and is not the constraint.
//
// Divides 1000000 exactly, so the free-running fallback wraps cleanly.
#define RING_PULSE_MAX_HZ 25u

// Fine adjust on the frequency grid, in detents per gap between two ratios.
#define FREQ_FINE_STEPS_PER_GAP 8

// How far off a grid entry a value may sit and still read as fully saturated,
// as a percentage of the distance to the neighbour. Absorbs rounding so a
// snapped ratio always shows a pure hue.
#define FREQ_OFFGRID_DEADZONE 5

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
