#ifndef INC_LIB_UI_FEEDBACK_H_
#define INC_LIB_UI_FEEDBACK_H_

#include "color_presets.h"
#include "helpers.h"
#include "ui_select.h"
#include <stdint.h>

// "That worked." One mechanism for every committed action, with the same
// timing everywhere, so copy, clear, save, load and assign all report back
// the same way. Before this there were three unrelated one-off timers - a
// per-channel mark decremented inside a render function, a preset-write flag
// painted outside the render pass, and an error blit that returned early from
// the input update - and most actions gave no feedback at all.
typedef enum
{
  FB_WRITE, // copy, save, assign
  FB_CLEAR,
  FB_LOAD,
  FB_ERROR,
} FeedbackKind;

#define UI_FB_SLOTS 4
#define UI_FB_DURATION MS(400)

typedef struct
{
  uint8_t kind;        // FeedbackKind
  uint8_t target_kind; // TargetKind
  int8_t id;           // < 0 means every element of that kind
  uint32_t remaining;  // 0 when the slot is free
} FeedbackSlot;

typedef struct
{
  FeedbackSlot slot[UI_FB_SLOTS];
} Feedback;

typedef struct UiState UiState;

// id < 0 flashes every element of that kind. Oldest slot is recycled when
// full, so a burst of actions cannot lose the most recent feedback.
void ui_feedback_emit(UiState* ui, FeedbackKind kind, TargetKind target_kind, int8_t id);

// Age every slot. Called once per UX pass, alongside the other UI timers -
// never from a render function.
void ui_feedback_tick(UiState* ui, uint32_t dt);

// Is this element currently flashing, and with what? out may be NULL.
int ui_feedback_active(const UiState* ui, TargetKind target_kind, int8_t id, FeedbackKind* out);

// The colour a given kind flashes in.
UiColor ui_feedback_color(FeedbackKind kind);

#endif /* INC_LIB_UI_FEEDBACK_H_ */
