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

// Semantic palette. The renderer names meanings, not colours, so "what a
// blinking button means" is defined once instead of being re-picked in every
// mode's switch arm.
//
// Brightness discipline: LEDs read as far too bright above VAL_MED on this
// hardware, so base and context layers stay at VAL_LOW and confirmations stop
// at VAL_MED. VAL_HIG and VAL_MAX are not used by the layered renderer.
//
// Within purple, motion is the discriminator rather than hue: a candidate
// blinks, a source and a muted channel are steady, and saturation separates
// them further.
typedef struct
{
  uint8_t h, s, v;
} UiColor;

#define UI_COL_CANDIDATE ((UiColor){HUE_PURPLE, SAT_MED, VAL_LOW})    // blinks: "you can pick this"
#define UI_COL_SOURCE ((UiColor){HUE_PURPLE, SAT_MAX, VAL_LOW})       // steady: already picked
#define UI_COL_MUTED ((UiColor){HUE_PURPLE, SAT_HIG, VAL_LOW})        // steady: output gated to 0V
#define UI_COL_CONFIRM_WRITE ((UiColor){HUE_GREEN, SAT_MAX, VAL_MED}) // copy / save / assign committed
#define UI_COL_CONFIRM_CLEAR ((UiColor){HUE_RED, SAT_MAX, VAL_MED})   // clear committed
#define UI_COL_CONFIRM_LOAD ((UiColor){HUE_PURPLE, SAT_MED, VAL_MED}) // preset loaded
#define UI_COL_ERROR ((UiColor){HUE_RED, SAT_MAX, VAL_MED})           // slow blink

#endif /* INC_LIB_COLOR_PRESETS_H_ */
