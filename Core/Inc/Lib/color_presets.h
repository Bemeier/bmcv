#ifndef INC_LIB_COLOR_PRESETS_H_
#define INC_LIB_COLOR_PRESETS_H_

#include <stdint.h>

#define HUE_RED 252
#define HUE_ORANGE 30
// 43, not 65. led_set_hsv() splits the wheel into six regions of 43, so a hue
// is pure at a multiple of 43 and a blend in between: 65 sits two thirds of the
// way from yellow to green and comes out (0.48, 1.00, 0.00) - chartreuse. It
// was also only 15 from HUE_GREEN, so the SHP and AMP buttons were nearly the
// same colour.
#define HUE_YELLOW 43
#define HUE_GREEN 80
#define HUE_CYAN 120
#define HUE_BLUE 160
// Between blue and magenta, and deliberately off the red/green/blue axis that
// led_set_dac uses for voltage - nothing wearing purple can be misread as a
// level. The selection/transfer family and mute all sit here.
#define HUE_PURPLE 180
#define HUE_MAGENTA 200
// Between magenta and red. Clearing wears this rather than purple: purple is
// what "off" or "default" looks like on half the pages, and the one page whose
// whole job is destructive should not be the same colour as a neutral setting.
#define HUE_PINK 225

#define VAL_OFF 0

// Below the base layer's brightness, for something that should be legible
// without competing with anything: the unselected parameter buttons.
#define VAL_DIM 2

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

// Frequency ratios. FRQ is a rational multiple of the beat, and the one thing
// about a ratio worth coding as colour is its prime limit - whether it is a
// straight division, a triplet or a quintuplet. Octaves are free, so 1/8 and 16
// are the same green and 1/3, 3/2 and 24 are the same yellow.
//
// Ordered along the wheel so it reads as familiar -> exotic. Red is not in this
// scale on purpose: the old table used it for every odd division, and red means
// an error and nothing else.
#define HUE_FREQ_STRAIGHT HUE_GREEN    // 2-limit: halves, quarters, octaves
#define HUE_FREQ_TRIPLET HUE_YELLOW    // 3-limit
#define HUE_FREQ_QUINTUPLET HUE_ORANGE // 5-limit

// Semantic palette. The renderer names meanings, not colours, so "what a
// blinking button means" is defined once instead of being re-picked in every
// mode's switch arm.
//
// Brightness discipline: LEDs read as far too bright above VAL_MED on this
// hardware, so base and context layers stay at VAL_LOW and confirmations stop
// at VAL_MED. VAL_HIG is for the two places where the point is being
// unmistakably brighter than something else of the same colour: the second
// stage of a held press, and the selected parameter button against the dim
// row it sits in. VAL_MAX is not used by the layered renderer.
//
// White is the whole vocabulary of assignment, and nothing else uses it: a
// short white flash over an element's own colour means "this can be picked",
// steady white means "the thing you are holding can go here".
//
// Purple is what a selection looks like - the source you are holding, and the
// flash confirming where it landed - and what "off" or "default" looks like in
// a settings list. Pink is destructive. Red is errors, and nothing else.
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
#define UI_COL_CONFIRM_WRITE ((UiColor){HUE_PURPLE, SAT_MAX, VAL_MED}) // copy / save / assign committed
#define UI_COL_CONFIRM_CLEAR ((UiColor){HUE_PINK, SAT_MAX, VAL_MED})   // clear committed
#define UI_COL_CONFIRM_LOAD ((UiColor){HUE_CYAN, SAT_MAX, VAL_MED})    // preset loaded
#define UI_COL_ERROR ((UiColor){HUE_RED, SAT_MAX, VAL_MED})            // slow blink; red is errors only

#endif /* INC_LIB_COLOR_PRESETS_H_ */
