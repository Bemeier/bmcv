#ifndef INC_LIB_UI_SELECT_H_
#define INC_LIB_UI_SELECT_H_

#include <stdint.h>

// What kind of thing a press addressed. The same physical button is a
// different kind depending on the mode - a scene button is a scene in CPY and
// an input in MON - so the mode descriptor says which, and callers do not
// guess. See ui_mode.h.
typedef enum
{
  TGT_NONE = 0,
  TGT_CHANNEL,
  TGT_SCENE,
  TGT_INPUT,
  TGT_KIND_COUNT
} TargetKind;

#define TGT_BIT(k) ((uint8_t) (1u << (k)))

// What the active mode does with a press. Previously this was implied by
// which shift mode happened to call assign_event() with which AssignType,
// with the mode-specific exceptions living in the callers.
typedef enum
{
  ACT_NONE = 0,
  ACT_COPY,        // pick a source, then a destination, then copy
  ACT_CLEAR,       // the press is the action; no source is held
  ACT_ROUTE_INPUT, // pick a channel, then the input that modulates it
  ACT_ROUTE_TRIG,  // pick a channel, then what triggers its sample & hold
} UiAction;

// The pending half of a two-step action. src_id < 0 means nothing is picked.
typedef struct
{
  uint8_t action;
  uint8_t src_kind;
  int8_t src_id;
} Selection;

// Declared rather than included: ui_state.h needs Selection above, so
// including it here would be circular.
typedef struct UiState UiState;
typedef struct UxState UxState;

void ui_sel_reset(UiState* ui);

// Is a source waiting for its destination?
int ui_sel_pending(const UiState* ui);
int8_t ui_sel_src(const UiState* ui);
int ui_sel_is_src(const UiState* ui, TargetKind kind, int8_t id);

// Would pressing this element right now do something? Asked by the renderer
// to decide what to highlight and by the dispatcher to decide what to act on,
// so the two cannot disagree.
int ui_sel_is_candidate(const UxState* state, TargetKind kind, int8_t id);

// The single entry point for "the user pressed this element". Picks a source,
// commits against a held source, or performs an immediate action, according
// to the active mode's descriptor.
void ui_sel_press(UxState* state, TargetKind kind, int8_t id, uint8_t is_long);

#endif /* INC_LIB_UI_SELECT_H_ */
