#include "ui_select.h"
#include "assign.h"
#include "hw_setup.h"
#include "ui_feedback.h"
#include "ui_mode.h"
#include "ui_state.h"
#include "ux_state.h"
#include <stdint.h>

// Which (source kind -> destination kind) pairs an action accepts. The single
// place that knows; both the dispatcher and the renderer go through it, so a
// button cannot blink as a valid target and then do nothing when pressed.
static int can_commit(uint8_t action, uint8_t src_kind, uint8_t dst_kind)
{
  switch (action)
  {
  case ACT_COPY:
    if (src_kind == TGT_CHANNEL)
      return dst_kind == TGT_CHANNEL || dst_kind == TGT_SCENE;
    if (src_kind == TGT_SCENE)
      return dst_kind == TGT_SCENE; // a whole scene has nowhere to go in one channel
    return 0;
  case ACT_ROUTE_INPUT:
    return src_kind == TGT_CHANNEL && dst_kind == TGT_INPUT;
  case ACT_ROUTE_TRIG:
    return src_kind == TGT_CHANNEL && (dst_kind == TGT_CHANNEL || dst_kind == TGT_INPUT);
  default:
    return 0;
  }
}

// Some actions have an effect the moment a source is picked, before any
// destination exists. Keyed on the action rather than on the shift mode: this
// file's whole point is that it does not know which mode it is running under,
// and ACT_* is the vocabulary it does own.
static void on_pick_src(UxState* state, uint8_t action, uint8_t kind, int8_t id)
{
  if (action == ACT_ROUTE_TRIG && kind == TGT_CHANNEL)
    assign_trig_arm_channel(id, state);
}

// ... and the inverse, when the source is pressed again to undo it.
static void on_deselect(UxState* state, uint8_t action, uint8_t kind, int8_t id)
{
  if (action == ACT_ROUTE_INPUT && kind == TGT_CHANNEL)
    assign_input_clear(id, state);
}

static void commit(UxState* state, uint8_t action, uint8_t src_kind, int8_t src_id, uint8_t dst_kind, int8_t dst_id)
{
  switch (action)
  {
  case ACT_COPY:
    if (src_kind == TGT_CHANNEL && dst_kind == TGT_CHANNEL)
      assign_channel_to_channel(src_id, dst_id, state);
    else if (src_kind == TGT_CHANNEL && dst_kind == TGT_SCENE)
      assign_channel_to_scene(src_id, dst_id, state);
    else if (src_kind == TGT_SCENE && dst_kind == TGT_SCENE)
      assign_scene_to_scene(src_id, dst_id, state);
    break;
  case ACT_ROUTE_INPUT:
    assign_input_to_channel(dst_id, src_id, state);
    break;
  case ACT_ROUTE_TRIG:
    // Same channel as source and destination means "stop triggering", which
    // assign_trig_src_use_channel already encodes.
    if (dst_kind == TGT_CHANNEL)
      assign_trig_src_use_channel(src_id, dst_id, state);
    else
      assign_trig_src_use_input(src_id, dst_id, state);
    break;
  default:
    break;
  }
}

void ui_sel_reset(UiState* ui)
{
  ui->sel.action   = ACT_NONE;
  ui->sel.src_kind = TGT_NONE;
  ui->sel.src_id   = -1; // must not be 0, which is a valid element id
}

int ui_sel_pending(const UiState* ui) { return ui->sel.src_id >= 0; }

int8_t ui_sel_src(const UiState* ui) { return ui->sel.src_id; }

int ui_sel_is_src(const UiState* ui, TargetKind kind, int8_t id)
{
  return ui_sel_pending(ui) && ui->sel.src_kind == kind && ui->sel.src_id == id;
}

int ui_sel_is_candidate(const UxState* state, TargetKind kind, int8_t id)
{
  const UiModeDesc* m = ui_mode(state->ui->shift_state);

  if (m->action == ACT_NONE)
    return 0;

  // Nothing picked yet (or nothing ever gets picked): anything this mode
  // accepts as a source is a candidate.
  if (m->immediate || !ui_sel_pending(state->ui))
    return (m->src_kinds & TGT_BIT(kind)) != 0;

  // The source itself is only pressable again where that means "undo".
  if (ui_sel_is_src(state->ui, kind, id))
    return m->allow_deselect;

  return (m->dst_kinds & TGT_BIT(kind)) && can_commit(state->ui->sel.action, state->ui->sel.src_kind, kind);
}

uint8_t ui_sel_press_stages(const UxState* state, TargetKind kind, int8_t id)
{
  const UiModeDesc* m = ui_mode(state->ui->shift_state);

  // Only the immediate actions have anything to show while held. Everywhere
  // else a press either picks a source or commits one, and neither has a
  // second stage or a threshold to cross - the press is the whole gesture.
  if (!m->immediate || !ui_sel_is_candidate(state, kind, id))
    return 0;

  // CLR on a channel: a tap clears it in the active scene, a hold clears the
  // whole channel. A scene has only the one thing it can mean.
  if (m->action == ACT_CLEAR && kind == TGT_CHANNEL)
    return 2;

  return 1;
}

void ui_sel_press(UxState* state, TargetKind kind, int8_t id, uint8_t is_long)
{
  const UiModeDesc* m = ui_mode(state->ui->shift_state);
  Selection* sel      = &state->ui->sel;

  if (m->action == ACT_NONE || id < 0)
    return;

  if (m->immediate)
  {
    if (!(m->src_kinds & TGT_BIT(kind)))
      return;
    if (m->action == ACT_CLEAR)
    {
      // On a channel, a long press clears every scene rather than just the
      // active one.
      if (kind == TGT_CHANNEL)
        clear_channel(id, is_long, state);
      else if (kind == TGT_SCENE)
        clear_scene(id, state);
      ui_feedback_emit(state->ui, FB_CLEAR, kind, id);
      ui_page_note_action(state->ui);
    }
    return;
  }

  if (!ui_sel_pending(state->ui))
  {
    if (!(m->src_kinds & TGT_BIT(kind)))
      return;

    sel->action   = m->action;
    sel->src_kind = kind;
    sel->src_id   = id;

    on_pick_src(state, sel->action, kind, id);

    // Deliberately not ui_page_note_action: picking a source is half a
    // gesture, and a page that closed on the release of its own button here
    // would take the other half with it.
    return;
  }

  if (m->allow_deselect && ui_sel_is_src(state->ui, kind, id))
  {
    on_deselect(state, sel->action, kind, id);
    ui_feedback_emit(state->ui, FB_CLEAR, kind, id);
    ui_page_note_action(state->ui);
    ui_sel_reset(state->ui);
    return;
  }

  if (can_commit(sel->action, sel->src_kind, kind))
  {
    commit(state, sel->action, sel->src_kind, sel->src_id, kind, id);
    // Flash the destination - the thing that changed - not the source.
    ui_feedback_emit(state->ui, FB_WRITE, kind, id);
    ui_page_note_action(state->ui);
    ui_sel_reset(state->ui);
  }
}
