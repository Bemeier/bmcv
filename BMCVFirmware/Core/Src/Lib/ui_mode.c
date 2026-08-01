#include "ui_mode.h"
#include "state.h"
#include "ui_select.h"

// Indexed by ShiftStates. Modes that do not take part in the selection model
// yet (STA, STB, SYS, SAV, MUT) still declare their exit behaviour here and
// keep their direct config writes in the handlers.
static const UiModeDesc modes[SHIFT_STATE_COUNT] = {
    [SHIFT_STATE_STA] = {.name = "STA", .exits_on_other_ctrl = 1, .action = ACT_NONE},
    [SHIFT_STATE_SYS] = {.name = "SYS", .exits_on_other_ctrl = 1, .action = ACT_NONE, .channel_enc_target = ENC_SHAPE},

    [SHIFT_STATE_QNT] = {.name                = "QNT",
                         .exits_on_other_ctrl = 0, // the keyboard overlay owns the other buttons
                         .action              = ACT_ROUTE_TRIG,
                         .src_kinds           = TGT_BIT(TGT_CHANNEL),
                         .dst_kinds           = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_INPUT),
                         .channel_btn_kind    = TGT_CHANNEL,
                         .scene_btn_kind      = TGT_INPUT,
                         .channel_enc_target  = ENC_QUANT},

    [SHIFT_STATE_MON] = {.name                = "MON",
                         .exits_on_other_ctrl = 1,
                         .action              = ACT_ROUTE_INPUT,
                         .src_kinds           = TGT_BIT(TGT_CHANNEL),
                         .dst_kinds           = TGT_BIT(TGT_INPUT),
                         .allow_deselect      = 1,
                         .channel_btn_kind    = TGT_CHANNEL,
                         .scene_btn_kind      = TGT_INPUT,
                         .channel_enc_target  = ENC_AMPMODE},

    [SHIFT_STATE_SAV] = {.name = "SAV", .exits_on_other_ctrl = 1, .action = ACT_NONE},
    [SHIFT_STATE_STB] = {.name = "STB", .exits_on_other_ctrl = 1, .action = ACT_NONE},
    [SHIFT_STATE_MUT] = {.name = "MUT", .exits_on_other_ctrl = 1, .action = ACT_NONE},

    [SHIFT_STATE_CPY] = {.name                = "CPY",
                         .exits_on_other_ctrl = 1,
                         .action              = ACT_COPY,
                         .src_kinds           = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_SCENE),
                         .dst_kinds           = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_SCENE),
                         .channel_btn_kind    = TGT_CHANNEL,
                         .scene_btn_kind      = TGT_SCENE},

    [SHIFT_STATE_CLR] = {.name                = "CLR",
                         .exits_on_other_ctrl = 1,
                         .action              = ACT_CLEAR,
                         .src_kinds           = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_SCENE),
                         .immediate           = 1,
                         .channel_btn_kind    = TGT_CHANNEL,
                         .scene_btn_kind      = TGT_SCENE},

    [SHIFT_STATE_NONE] = {.name = "---", .exits_on_other_ctrl = 1, .action = ACT_NONE, .channel_enc_target = ENC_PARAM},
};

const UiModeDesc* ui_mode(uint8_t shift_state)
{
  if (shift_state >= SHIFT_STATE_COUNT)
    return &modes[SHIFT_STATE_NONE];
  return &modes[shift_state];
}
