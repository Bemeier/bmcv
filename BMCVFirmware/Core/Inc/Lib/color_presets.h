#ifndef INC_LIB_COLOR_PRESETS_H_
#define INC_LIB_COLOR_PRESETS_H_

#include <stdint.h>

#define HUE_RED 252
#define HUE_ORANGE 30
#define HUE_YELLOW 65
#define HUE_GREEN 80
#define HUE_CYAN 120
#define HUE_BLUE 160
// Between blue and magenta, and deliberately off the red/green/blue axis that
// led_set_dac uses for voltage - nothing wearing purple can be misread as a
// level. The selection/transfer family and mute all sit here.
#define HUE_PURPLE 180
#define HUE_MAGENTA 200

#define VAL_OFF 0

#define VAL_LOW 8

#define VAL_MED 32

#define VAL_HIG 64

#define VAL_MAX 128

#define SAT_OFF 0

#define SAT_LOW 64

#define SAT_MED 128

#define SAT_HIG 230

#define SAT_MAX 255

// Setting-state hues. A shift mode's channel and scene LEDs show which value a
// per-channel or per-input setting is on, and the same concept gets the same
// hue on every page - so "purple means it is switched off" is learned once and
// then holds everywhere, rather than each mode picking its own four colours.
//
// The rule that matters: index 0 of every settings enum is the off / default /
// neutral value, and it is always purple.
#define HUE_STATE_DEFAULT HUE_PURPLE // off, disabled, neutral
#define HUE_STATE_LEVEL HUE_CYAN     // continuous, follows a level
#define HUE_STATE_MIX HUE_GREEN      // additive, or half-way along a ramp
#define HUE_STATE_EVENT HUE_YELLOW   // triggered, clocked, discrete steps
// Magenta is a hair from purple at this saturation and brightness, and purple
// already means "off" on the same page, so multiply borrows cyan.
#define HUE_STATE_MULT HUE_CYAN // multiplicative
#define HUE_STATE_RESET HUE_RED // resets something

// Semantic palette. The renderer names meanings, not colours, so "what a
// blinking button means" is defined once instead of being re-picked in every
// mode's switch arm.
//
// Brightness discipline: LEDs read as far too bright above VAL_MED on this
// hardware, so base and context layers stay at VAL_LOW and confirmations stop
// at VAL_MED. VAL_HIG and VAL_MAX are not used by the layered renderer.
//
// White is the whole vocabulary of assignment, and nothing else uses it: a
// short white flash over an element's own colour means "this can be picked",
// steady white means "the thing you are holding can go here". Purple is left
// to mean off / default / cleared, which is what it means everywhere else.
typedef struct
{
  uint8_t h, s, v;
} UiColor;

#define UI_COL_MARK ((UiColor){0, SAT_OFF, VAL_LOW})                   // brief flash: "you can pick this"
#define UI_COL_TARGET ((UiColor){0, SAT_OFF, VAL_LOW})                 // steady: "assign to this"
#define UI_COL_DARK ((UiColor){0, SAT_OFF, VAL_OFF})                   // nothing here
#define UI_COL_SOURCE ((UiColor){HUE_PURPLE, SAT_MAX, VAL_LOW})        // steady: already picked
#define UI_COL_MUTED ((UiColor){HUE_PURPLE, SAT_HIG, VAL_LOW})         // steady: output gated to 0V
#define UI_COL_UNMUTED ((UiColor){HUE_GREEN, SAT_HIG, VAL_LOW})        // steady: output passing, on the mute page
#define UI_COL_CONFIRM_WRITE ((UiColor){HUE_GREEN, SAT_MAX, VAL_MED})  // copy / save / assign committed
#define UI_COL_CONFIRM_CLEAR ((UiColor){HUE_PURPLE, SAT_MAX, VAL_MED}) // clear committed
#define UI_COL_CONFIRM_LOAD ((UiColor){HUE_CYAN, SAT_MAX, VAL_MED})    // preset loaded
#define UI_COL_ERROR ((UiColor){HUE_RED, SAT_MAX, VAL_MED})            // slow blink; red is errors only

#endif /* INC_LIB_COLOR_PRESETS_H_ */
