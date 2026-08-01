#include "ui_feedback.h"
#include "color_presets.h"
#include "ui_state.h"
#include <stdint.h>

UiColor ui_feedback_color(FeedbackKind kind)
{
  switch (kind)
  {
  case FB_CLEAR:
    return UI_COL_CONFIRM_CLEAR;
  case FB_LOAD:
    return UI_COL_CONFIRM_LOAD;
  case FB_ERROR:
    return UI_COL_ERROR;
  case FB_WRITE:
  default:
    return UI_COL_CONFIRM_WRITE;
  }
}

void ui_feedback_emit(UiState* ui, FeedbackKind kind, TargetKind target_kind, int8_t id)
{
  FeedbackSlot* victim = NULL;

  for (uint8_t i = 0; i < UI_FB_SLOTS; i++)
  {
    FeedbackSlot* s = &ui->fb.slot[i];

    // Re-flashing the same element restarts it rather than filling a slot.
    if (s->remaining > 0 && s->kind == kind && s->target_kind == target_kind && s->id == id)
    {
      s->remaining = UI_FB_DURATION;
      return;
    }

    if (s->remaining == 0)
    {
      victim = s;
      break;
    }

    // Otherwise evict whichever has least time left - it is the one already
    // closest to disappearing.
    if (victim == NULL || s->remaining < victim->remaining)
      victim = s;
  }

  victim->kind        = kind;
  victim->target_kind = target_kind;
  victim->id          = id;
  victim->remaining   = UI_FB_DURATION;
}

void ui_feedback_tick(UiState* ui, uint32_t dt)
{
  for (uint8_t i = 0; i < UI_FB_SLOTS; i++)
  {
    FeedbackSlot* s = &ui->fb.slot[i];
    if (s->remaining > dt)
      s->remaining -= dt;
    else
      s->remaining = 0;
  }
}

int ui_feedback_active(const UiState* ui, TargetKind target_kind, int8_t id, FeedbackKind* out)
{
  for (uint8_t i = 0; i < UI_FB_SLOTS; i++)
  {
    const FeedbackSlot* s = &ui->fb.slot[i];
    if (s->remaining == 0 || s->target_kind != target_kind)
      continue;
    if (s->id >= 0 && s->id != id)
      continue;

    if (out)
      *out = (FeedbackKind) s->kind;
    return 1;
  }
  return 0;
}
