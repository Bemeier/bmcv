#ifndef INC_LIB_UI_RENDER_H_
#define INC_LIB_UI_RENDER_H_

#include "ui_select.h"
#include "ux_state.h"
#include <stdint.h>

// Every element renders as the same strict stack of layers, highest wins:
//
//   3  confirmation flash   an action just committed here
//   2  edit value           this element was touched recently
//   1  context              candidate / source in the active mode
//   0  base                 output level, or muted, or the mode's own state
//
// Layer 0 always writes, so a stale pixel is structurally impossible - the
// previous per-mode switch statements had arms that wrote nothing and left
// the framebuffer holding the last frame.
//
// Layers 1 and 3 are entirely mode-independent: they read ui_sel_is_candidate
// and ui_feedback_active, so a new mode gets consistent highlighting and
// confirmation without writing any render code.

void ui_render(UxState* state);

// Individual layers, exposed so tests can assert precedence directly.
void ui_render_context(UxState* state, int16_t led, TargetKind kind, int8_t id);
void ui_render_feedback(UxState* state, int16_t led, TargetKind kind, int8_t id);

#endif /* INC_LIB_UI_RENDER_H_ */
