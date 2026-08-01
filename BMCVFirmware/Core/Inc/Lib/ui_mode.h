#ifndef INC_LIB_UI_MODE_H_
#define INC_LIB_UI_MODE_H_

#include "state.h"
#include "ui_select.h"
#include <stdint.h>

// What each shift mode does, as data rather than as a switch arm repeated in
// every handler and every renderer. The dispatcher and the renderer read the
// same descriptor, which is what stops "what blinks" and "what responds" from
// drifting apart - they had already drifted in CPY, QNT and MON.
typedef struct
{
  const char* name;

  // QNT overlays the ctrl and scene buttons with a piano-style semitone
  // keyboard, so a tap on another ctrl button is a note, not "leave this
  // mode". Every other mode exits on any ctrl tap.
  uint8_t exits_on_other_ctrl;

  uint8_t action;    // UiAction this mode's presses perform
  uint8_t src_kinds; // TGT_BIT mask: what may be picked as a source
  uint8_t dst_kinds; // TGT_BIT mask: what may be picked as a destination

  // The press commits immediately instead of holding a source (CLR).
  uint8_t immediate;

  // Pressing the source again undoes the assignment rather than being
  // ignored (MON: press the channel again to unroute its input).
  uint8_t allow_deselect;

  // Which TargetKind a press on each button group addresses in this mode.
  uint8_t channel_btn_kind;
  uint8_t scene_btn_kind;
} UiModeDesc;

const UiModeDesc* ui_mode(uint8_t shift_state);

#endif /* INC_LIB_UI_MODE_H_ */
