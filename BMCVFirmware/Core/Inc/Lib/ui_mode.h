#ifndef INC_LIB_UI_MODE_H_
#define INC_LIB_UI_MODE_H_

#include "config.h"
#include "ui_select.h"
#include <stdint.h>

// The shift modes themselves. Lives here rather than with the persisted
// config it used to sit next to: it is not persisted, and this is the file
// that enumerates what each one does.
typedef enum
{
  SHIFT_STATE_STA,
  SHIFT_STATE_SYS,
  SHIFT_STATE_QNT,
  SHIFT_STATE_MON,
  SHIFT_STATE_SAV,
  SHIFT_STATE_STB,
  SHIFT_STATE_MUT,
  SHIFT_STATE_CPY,
  SHIFT_STATE_CLR,
  SHIFT_STATE_NONE,
  SHIFT_STATE_COUNT
} ShiftStates;

// What each shift mode does, as data rather than as a switch arm repeated in
// every handler and every renderer. The dispatcher and the renderer read the
// same descriptor, which is what stops "what blinks" and "what responds" from
// drifting apart - they had already drifted in CPY, QNT and MON.
//
// A new mode should be one row in ui_mode.c. If adding one needs an edit to
// ui_render.c, scene.c or channel.c, then whatever it needed is missing from
// this descriptor and belongs here instead.

// What a channel encoder drives in a given mode. Also names the value the
// transient display shows when that channel is touched, so one renderer
// serves every mode instead of each mode drawing its own.
typedef enum
{
  ENC_NONE = 0,
  ENC_PARAM,   // the selected scene parameter
  ENC_SHAPE,   // waveshape mode
  ENC_QUANT,   // quantize mode
  ENC_AMPMODE, // input cross-modulation mode
} EncoderTarget;

// What a channel button does. CHB_SELECT hands the press to the selection
// model; the others are direct.
typedef enum
{
  CHB_NONE = 0,
  CHB_SELECT,
  CHB_RESET_PARAM, // long press resets the selected param
  CHB_MUTE_TOGGLE,
} ChannelButtonAction;

typedef enum
{
  SCN_NONE = 0,
  SCN_SELECT,     // hand to the selection model
  SCN_MOMENTARY,  // hold to make this the active scene
  SCN_SET_XFADE,  // wire to one end of the crossfader; xfade_end says which
  SCN_INPUT_MODE, // cycle this input's role
  SCN_PRESET,     // tap loads, long hold stores
} SceneButtonAction;

// What a scene button's LED shows before the context and confirmation layers
// go over the top - layer 0 of the renderer, as data. See ui_render.h.
typedef enum
{
  SCB_CROSSFADE = 0, // this scene's weight in the blend
  SCB_INPUT_LEVEL,   // live CV on the matching input jack
  SCB_INPUT_MODE,    // that input's configured role, as a hue
  SCB_PRESET,        // armed slot, turning red once the hold will store
} SceneBaseLayer;

// Which end of the crossfader a mode's scene buttons wire, and which end the
// renderer marks as the current source. STA and STB differ in nothing else.
typedef enum
{
  XFADE_NONE = 0,
  XFADE_A,
  XFADE_B,
} CrossfadeEnd;

typedef struct
{
  const char* name;

  // QNT overlays the ctrl and scene buttons with a piano-style semitone
  // keyboard. That single fact decides three things, which is why it is one
  // flag and not three: a tap on another ctrl button is a note rather than
  // "leave this mode", the semitone handler runs, and the keyboard is drawn
  // over the scene row.
  uint8_t keyboard_overlay;

  UiAction action;   // what this mode's presses perform
  uint8_t src_kinds; // TGT_BIT mask: what may be picked as a source
  uint8_t dst_kinds; // TGT_BIT mask: what may be picked as a destination

  // The press commits immediately instead of holding a source (CLR).
  uint8_t immediate;

  // Pressing the source again undoes the assignment rather than being
  // ignored (MON: press the channel again to unroute its input).
  uint8_t allow_deselect;

  // Which TargetKind a press on each button group addresses in this mode.
  TargetKind channel_btn_kind;
  TargetKind scene_btn_kind;

  EncoderTarget channel_enc_target;
  ChannelButtonAction channel_btn_action;
  SceneButtonAction scene_btn_action;

  SceneBaseLayer scene_btn_base;
  CrossfadeEnd xfade_end;
} UiModeDesc;

const UiModeDesc* ui_mode(uint8_t shift_state);

#endif /* INC_LIB_UI_MODE_H_ */
