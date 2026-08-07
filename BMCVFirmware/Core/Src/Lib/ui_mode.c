#include "ui_mode.h"
#include "color_presets.h"
#include "config.h"
#include "hw_setup.h"
#include "ui_select.h"

// A ctrl button's index is three things at once: a ShiftStates (hold it to
// enter that mode), a ChannelParameters for the first six (tap it with no mode
// active to choose what the encoders edit), and an index into
// HwSetup.ctrl_button_color[]. Reordering ctrl_button_idx in hw_setup.c would
// silently remap every shift mode, so state the coupling here, next to the
// table that depends on it.
_Static_assert(N_CTRL_BUTTONS == SHIFT_STATE_COUNT - 1, "ctrl button id is a ShiftStates; SHIFT_STATE_NONE has no button");
_Static_assert(CH_PARAM_COUNT <= N_CTRL_BUTTONS, "the first CH_PARAM_COUNT ctrl buttons select a parameter");

// Every mode, as one table. What each button group does, what the encoder
// drives, what the LEDs show underneath, and how the mode is left - all in one
// place, read by both the dispatcher and the renderer.
static const UiModeDesc modes[SHIFT_STATE_COUNT] = {
    // The encoders set the stepped-random pattern length here: it is a division
    // of the beat, and this is the page the FRQ button opens.
    [SHIFT_STATE_STA] = {.name               = "STA",
                         .scene_btn_kind     = TGT_SCENE,
                         .channel_enc_target = ENC_SR_LENGTH,
                         .scene_btn_action   = SCN_SET_XFADE,
                         .channel_base       = CHBASE_SR_LENGTH,
                         .scene_btn_base     = SCB_CROSSFADE,
                         .xfade_end          = XFADE_A},

    [SHIFT_STATE_STB] = {.name             = "STB",
                         .scene_btn_kind   = TGT_SCENE,
                         .scene_btn_action = SCN_SET_XFADE,
                         .scene_btn_base   = SCB_CROSSFADE,
                         .xfade_end        = XFADE_B},

    [SHIFT_STATE_SYS] = {.name               = "SYS",
                         .scene_btn_kind     = TGT_INPUT,
                         .channel_enc_target = ENC_SHAPE,
                         .scene_btn_action   = SCN_INPUT_MODE,
                         .channel_base       = CHBASE_SHAPE,
                         .scene_btn_base     = SCB_INPUT_MODE},

    [SHIFT_STATE_QNT] = {.name               = "QNT",
                         .keyboard_overlay   = 1,
                         .action             = ACT_ROUTE_TRIG,
                         .src_kinds          = TGT_BIT(TGT_CHANNEL),
                         .dst_kinds          = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_INPUT),
                         .channel_btn_kind   = TGT_CHANNEL,
                         .scene_btn_kind     = TGT_INPUT,
                         .channel_enc_target = ENC_QUANT,
                         .channel_btn_action = CHB_SELECT,
                         .scene_btn_action   = SCN_SELECT,
                         .channel_base       = CHBASE_QUANTIZE,
                         .scene_btn_base     = SCB_INPUT_LEVEL},

    [SHIFT_STATE_MON] = {.name               = "MON",
                         .action             = ACT_ROUTE_INPUT,
                         .src_kinds          = TGT_BIT(TGT_CHANNEL),
                         .dst_kinds          = TGT_BIT(TGT_INPUT),
                         .allow_deselect     = 1,
                         .channel_btn_kind   = TGT_CHANNEL,
                         .scene_btn_kind     = TGT_INPUT,
                         .channel_enc_target = ENC_AMPMODE,
                         .channel_btn_action = CHB_SELECT,
                         .scene_btn_action   = SCN_SELECT,
                         .channel_base       = CHBASE_AMPMODE,
                         .scene_btn_base     = SCB_INPUT_LEVEL},

    // The encoders set each channel's output range, which is the AMP button's
    // page - it is what bounds the amplitude the module actually puts out.
    [SHIFT_STATE_SAV] = {.name               = "SAV",
                         .scene_btn_kind     = TGT_SCENE,
                         .channel_enc_target = ENC_CLAMP,
                         .scene_btn_action   = SCN_PRESET,
                         .channel_base       = CHBASE_CLAMP,
                         .scene_btn_base     = SCB_PRESET},

    // Scene buttons have no use here yet, so they keep showing the blend.
    [SHIFT_STATE_MUT] = {.name               = "MUT",
                         .channel_btn_kind   = TGT_CHANNEL,
                         .channel_enc_target = ENC_MUTE,
                         .channel_btn_action = CHB_MUTE_TOGGLE,
                         .scene_btn_kind     = TGT_SCENE,
                         .channel_base       = CHBASE_MUTE,
                         .scene_btn_base     = SCB_CROSSFADE},

    [SHIFT_STATE_CPY] = {.name               = "CPY",
                         .action             = ACT_COPY,
                         .src_kinds          = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_SCENE),
                         .dst_kinds          = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_SCENE),
                         .channel_btn_kind   = TGT_CHANNEL,
                         .scene_btn_kind     = TGT_SCENE,
                         .channel_btn_action = CHB_SELECT,
                         .scene_btn_action   = SCN_SELECT,
                         .channel_base       = CHBASE_TINT,
                         .scene_btn_base     = SCB_MODE_TINT,
                         .tint_hue           = HUE_BLUE},

    [SHIFT_STATE_CLR] = {.name               = "CLR",
                         .action             = ACT_CLEAR,
                         .src_kinds          = TGT_BIT(TGT_CHANNEL) | TGT_BIT(TGT_SCENE),
                         .immediate          = 1,
                         .channel_btn_kind   = TGT_CHANNEL,
                         .scene_btn_kind     = TGT_SCENE,
                         .channel_btn_action = CHB_SELECT,
                         .scene_btn_action   = SCN_SELECT,
                         .channel_base       = CHBASE_TINT,
                         .scene_btn_base     = SCB_MODE_TINT,
                         .tint_hue           = HUE_PINK},

    [SHIFT_STATE_NONE] = {.name               = "---",
                          .scene_btn_kind     = TGT_SCENE,
                          .channel_enc_target = ENC_PARAM,
                          .channel_btn_action = CHB_RESET_PARAM,
                          .scene_btn_action   = SCN_MOMENTARY,
                          .channel_base       = CHBASE_OUTPUT,
                          .scene_btn_base     = SCB_CROSSFADE},
};

const UiModeDesc* ui_mode(uint8_t shift_state)
{
  if (shift_state >= SHIFT_STATE_COUNT)
    return &modes[SHIFT_STATE_NONE];
  return &modes[shift_state];
}
