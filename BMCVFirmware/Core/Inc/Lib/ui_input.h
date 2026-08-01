#ifndef INC_LIB_UI_INPUT_H_
#define INC_LIB_UI_INPUT_H_

#include "helpers.h"
#include "hw_setup.h"
#include "state.h"
#include <stdint.h>

// Derived button gestures. Every consumer reads these instead of re-deriving
// press/hold semantics from HwState's raw button_pressed_t/button_released_t,
// which is how five different thresholds ended up scattered across the UX
// layer - and how the render path and the input path came to disagree about
// when a long press had happened.
//
// Events are edges: each fires exactly once per gesture. They are OR-ed into
// a bitmask because the UX layer runs slower than the engine (see
// ui_input_update below), so several may land in the same dispatch.
#define BTN_EV_NONE 0u
#define BTN_EV_DOWN (1u << 0)  // pressed this tick
#define BTN_EV_UP (1u << 1)    // released this tick, whatever the duration
#define BTN_EV_TAP (1u << 2)   // released before UI_T_HOLD
#define BTN_EV_HOLD (1u << 3)  // held past UI_T_HOLD, fires while still down
#define BTN_EV_LONG (1u << 4)  // held past UI_T_LONG, fires while still down
#define BTN_EV_VLONG (1u << 5) // held past UI_T_VLONG, fires while still down

// The only press-duration thresholds in the firmware.
//
// Presses shorter than UI_T_DEBOUNCE emit no UP/TAP at all - that is what the
// scattered `released_t > MS(10)` checks were really asking for, so the test
// belongs here rather than at every call site.
#define UI_T_DEBOUNCE MS(10)
#define UI_T_HOLD MS(100)
#define UI_T_LONG MS(500)
#define UI_T_VLONG MS(1000)

typedef struct
{
  uint8_t ev[N_BUTTONS];   // accumulated BTN_EV_* bitmask, cleared on drain
  uint8_t down[N_BUTTONS]; // level, not an edge

  // How long the button has been down, or - once released - how long the last
  // press lasted. Cleared on the next press, not on release, so a handler
  // reacting to BTN_EV_TAP/LONG can still ask how long the press was.
  uint32_t held_us[N_BUTTONS];

  int16_t enc_delta[N_ENCODERS]; // accumulated, cleared on drain

  uint32_t dt; // accumulated since the last drain - the UI's only time base

  // Internal: which thresholds a given press has already reported, so HOLD
  // and LONG fire once each rather than every tick while the button is down.
  uint8_t crossed[N_BUTTONS];
} UiInput;

// Fold one engine tick of raw hardware state into the event set.
//
// MUST run every engine tick, not only on the ticks where the UX layer
// updates: engine.c only calls the UX layer when a button *level* changed or
// the 8ms interval elapsed, and a HOLD/LONG threshold crossing is neither. So
// events and encoder deltas accumulate here and are drained by the dispatcher.
void ui_input_update(UiInput* in, const HwState* curr);

// Consume the accumulated events. Call once per UX dispatch, after reading.
void ui_input_drain(UiInput* in);

static inline uint8_t btn_ev(const UiInput* in, int8_t btn, uint8_t mask)
{
  if (btn < 0 || btn >= N_BUTTONS)
    return 0;
  return (in->ev[btn] & mask) != 0;
}

static inline uint8_t btn_down(const UiInput* in, int8_t btn)
{
  if (btn < 0 || btn >= N_BUTTONS)
    return 0;
  return in->down[btn];
}

// Duration of the current press, or of the last one once released.
static inline uint32_t btn_held(const UiInput* in, int8_t btn)
{
  if (btn < 0 || btn >= N_BUTTONS)
    return 0;
  return in->held_us[btn];
}

// Live level: "is being held, and has been for at least t". For indicators
// that should light up while the user keeps holding.
static inline uint8_t btn_holding(const UiInput* in, int8_t btn, uint32_t t) { return btn_down(in, btn) && btn_held(in, btn) >= t; }

// Edge: "was just released, after being held at least t".
static inline uint8_t btn_released_after(const UiInput* in, int8_t btn, uint32_t t)
{
  return btn_ev(in, btn, BTN_EV_UP) && btn_held(in, btn) >= t;
}

static inline int16_t enc_delta(const UiInput* in, int8_t enc)
{
  if (enc < 0 || enc >= N_ENCODERS)
    return 0;
  return in->enc_delta[enc];
}

#endif /* INC_LIB_UI_INPUT_H_ */
