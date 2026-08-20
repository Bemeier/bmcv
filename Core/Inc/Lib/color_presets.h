#ifndef INC_LIB_COLOR_PRESETS_H_
#define INC_LIB_COLOR_PRESETS_H_

#include <stdint.h>

/* ---- hues -------------------------------------------------------------- */

// Positions on led_set_hsv's wheel, which is six regions of 43: a hue is at its
// purest on a multiple of 43 and a blend in between. Off-vertex is often what
// is wanted - teal and purple only exist between two - but it is worth knowing
// which of these are vertices (red, yellow) and which are mixtures, because a
// mixture's minor primary lands on a die of its own efficiency and the result
// is rarely halfway.
//
// The simulator draws all of this faithfully now, so it is the place to judge
// them rather than a build-flash-squint cycle.
//
// Which half of the wheel a hue is on is itself a meaning - see the arc below.
#define HUE_RED 0
// 43, not 65. led_set_hsv() splits the wheel into six regions of 43, so a hue
// is pure at a multiple of 43 and a blend in between: 65 sits two thirds of the
// way from yellow to green and comes out (0.48, 1.00, 0.00) - chartreuse. It
// was also only 15 from HUE_GREEN, so the SHP and AMP buttons were nearly the
// same colour.
#define HUE_YELLOW 43
#define HUE_GREEN 80

// The cool half, 30 apart, and every colour an encoder ring wears that is not a
// voltage comes from here. 30 because the evidence on the wheel is that around
// 25 is where two hues stop reading as one: orange sat 13 from yellow and the
// two were hard to tell apart, and moving it to 25 away made it its own colour.
#define HUE_TEAL 110
#define HUE_CYAN 140
#define HUE_BLUE 170
// Off the red/green axis led_set_dac uses for voltage, and far enough off it
// that nothing wearing purple can be misread as a level. The selection/transfer
// family, mute and every settings default sit here.
#define HUE_PURPLE 200
// Between purple and red. Clearing wears this rather than purple: purple is
// what "off" or "default" looks like on half the pages, and the one page whose
// whole job is destructive should not be the same colour as a neutral setting.
#define HUE_PINK 230

/* ---- the voltage arc --------------------------------------------------- */

// led_set_bipolar owns the warm half of the wheel. On an encoder ring, one of
// its colours means a voltage reading and nothing else.
//
// Which colours those are, measured rather than assumed - its die ratios put
// back through set_hsv_fine:
//
//     |V|        positive   negative
//     0V         19 (dark)  19 (dark)
//     0.3V       55          6
//     0.6 - 5V   85          0
//     7.5V       78          2
//     10V        69          3
//
// So the readable arc is h in [0, 6] and [69, 85]. The stretch between is
// crossed only inside +/-0.6V, where LED_CV_FLOOR has the ring nearly dark.
// HUE_GREEN is inside the positive arm - it *is* the +3V ring - and hue 18 is
// the zero crossing exactly, being the hue where the two dies emit equal light
// (204 : 189 in weighted units). Neither is available to anything that is not
// a level, which is why the division scale below had to leave them.
//
// What keeps the two vocabularies apart is not hue distance, though every cool
// hue above is 25 or more clear of both arms. It is that the arc **never lights
// the blue die** - led->b = 0 in led_set_bipolar, asserted in test_led_render.c
// - and every control hue does: 56% of the light at HUE_TEAL, 65% at HUE_PINK.
// That is the property to test when adding a colour, and the reason a hue with
// no blue in it (yellow, orange) is not a free choice here.

/* ---- levels ------------------------------------------------------------ */

// Brightness. A value is light, not duty: led_set_hsv scales each hue until it
// puts out the value it was asked for, so these mean the same thing whatever
// colour wears them. See LED_PALETTE_REF in led_curve.h.
#define VAL_OFF 0
#define VAL_DIM 2 // legible without competing: the unselected parameter buttons

// The lowest duty a single die is asked for before it is lifted to it.
//
// The balancing above equalises *light*, and does it correctly: every control
// hue at VAL_DIM comes out at the same 3.5 units. What it cannot know is that a
// die does not behave down there. Below about two duty steps a WS2812 is on the
// bend of its own curve and emits less than the linear model says - and a hue
// mixed from two dies reaches that bend twice as fast, because the same light
// is split between them.
//
// Measured at VAL_DIM: red asks one die for 4.34 and looks right; yellow asks
// two for 1.37 each and looks half-lit, though the arithmetic says they match.
// Green, being the efficient die, gets away with 1.89. So the mixtures are the
// ones that suffer, and yellow - two dies, near enough evenly split - suffers
// most.
//
// Lifting rather than scaling, and only the primaries that are already on: what
// is wrong is the bottom of one die's curve, not the ratio between them. It
// costs a little hue accuracy at the very bottom of the range, where there is
// no hue accuracy to speak of anyway.
#define LED_MIN_ON_DUTY 2.0f
#define VAL_LOW 8
#define VAL_MED 32
#define VAL_HIG 64
#define VAL_MAX 128 // the top of the scale; the layered renderer stays under it

// What a base layer is drawn at, and the peg every other level is set against.
// Named rather than spelled VAL_MED at each call site: half the palette below
// is defined as "a step above the base" or "the same as the page underneath",
// and those relationships are the point.
#define VAL_BASE VAL_MED

// Saturation. Worth knowing what SAT_HIG costs, now that the simulator shows it
// honestly: it leaves about a tenth of the value on each of the other two
// primaries, and a tenth is not a tenth once it reaches the eye - a little blue
// shifts a hue further than a little green does, so a SAT_HIG red reads faintly
// pink and a SAT_HIG green faintly pale. SAT_MAX is a bare die and has none of
// that. Which is wanted is taste; the state colours are on SAT_HIG.
#define SAT_OFF 0
#define SAT_LOW 64
#define SAT_MED 128
#define SAT_HIG 230
#define SAT_MAX 255

/* ---- what a colour means ----------------------------------------------- */

// Setting-state hues. A shift mode's channel and scene LEDs show which value a
// per-channel or per-input setting is on, and the same concept gets the same
// hue on every page - so "purple means it is switched off" is learned once and
// then holds everywhere, rather than each mode picking its own four colours.
//
// The rule that matters: index 0 of every settings enum is the off / default /
// neutral value, and it is always purple.
#define HUE_STATE_DEFAULT HUE_PURPLE // off, disabled, neutral
#define HUE_STATE_LEVEL HUE_CYAN     // continuous, follows a level
#define HUE_STATE_MIX HUE_TEAL       // additive, or half-way along a ramp
#define HUE_STATE_EVENT HUE_BLUE     // triggered, clocked, discrete steps
// Cyan again: multiply and level-following are near enough the same idea that
// sharing reads as a family rather than as a collision, and the cool half does
// not have a sixth slot to spend on the difference.
#define HUE_STATE_MULT HUE_CYAN // multiplicative

// The one state that keeps a warm hue, and the only one that can: it reaches an
// LED through input_mode_color alone, which renders on the SYS scene row, and
// the SYS channel row shows shape mode - so RESET never shares a panel with a
// voltage. Yellow is the narrowest clearance in the palette, 24 from the zero
// crossing and 26 from the +10V hue, and this is the one place on the module
// where there is nothing for it to be confused with.
#define HUE_STATE_RESET HUE_YELLOW // resets something

// Divisions. Anything that divides the beat - the FRQ ratio, and the number of
// steps a stepped pattern loops over - is coded by its prime limit:
// whether it is a straight division, a triplet or a quintuplet. Octaves are
// free, so 1/8 and 16 are the same teal, 1/3, 3/2 and 24 are the same blue,
// and a 12-step pattern is the same blue as a 3-step one.
//
// Three classes is the whole of it, and the pattern lengths are chosen to stay
// inside them - a 7-step pattern would have needed a fourth colour squeezed
// into the scale, and was dropped rather than crowd it.
//
// One scale for both pages, on purpose. A triplet feels like a triplet whether
// it is what the channel runs at or how its pattern subdivides, and the point
// of a coded colour is that it is learned once.
//
// Ordered along the wheel so it still reads as familiar -> exotic, but walking
// the cool half rather than the warm one. It used to be green -> yellow ->
// orange, which is the voltage arc end to end: green is the +3V ring, orange is
// the zero crossing, and yellow is the gap between the two arms. Three ratios
// were being coded in the colours of the level the ring shows the rest of the
// time.
//
// 60 apart, against the 25 to 37 the old scale had, so RING_PULSE_HUE_SWING has
// more room than it ever did - and the swing now runs inward, toward the middle
// of this scale, so a pulse cannot brighten a class toward the arc.
#define HUE_FREQ_STRAIGHT HUE_TEAL   // 2-limit: halves, quarters, octaves
#define HUE_FREQ_TRIPLET HUE_BLUE    // 3-limit
#define HUE_FREQ_QUINTUPLET HUE_PINK // 5-limit

/* ---- the signed parameter ramp ----------------------------------------- */

// SHP, MOD and PHS are numbers, not voltages, so their ring reads the way the
// voltage ramp does - dark at zero, brightening toward either end, hue for the
// sign - in colours that cannot be taken for one. See led_set_signed.
//
// 120 units apart, wider than the arc's own 85, and it needs to be: hue is
// harder to resolve across the blue region than across red and green. Purple
// and cyan were the first pairing and were dropped, because purple is what a
// muted channel shows on the same row and both are confirmation-flash colours -
// a negative value would have read as a muted channel.
#define HUE_SIGNED_POS HUE_TEAL
#define HUE_SIGNED_NEG HUE_PINK

/* ---- the semantic palette ---------------------------------------------- */

// The renderer names meanings, not colours, so "what a blinking button means"
// is defined once instead of being re-picked in every mode's switch arm.
//
// Brightness: a base or context layer is VAL_BASE, and anything whose job is to
// read as brighter than the page underneath it - a confirmation, the second
// stage of a held press, the selected parameter button - goes one step above at
// VAL_HIG. Nothing in the layered renderer goes higher.
//
// Colour: purple is selection - the source you are holding, and the flash
// confirming where it landed - and also what "off" or "default" looks like in a
// settings list. Pink is destructive. Red and green are a voltage; the error
// blink is the one thing that borrows red, and it blanks the panel to say so.
//
// White is the whole vocabulary of assignment and nothing else uses it: white
// light swelling over an element's own colour means "this can be picked", and
// the same light over a cleared one means "the thing you are holding can go
// here". Both come from the field in ui_sparkle.h rather than from a colour
// here, which is why neither has an entry below.
typedef struct
{
  uint8_t h, s, v;
} UiColor;

#define UI_COL_DARK ((UiColor){0, SAT_OFF, VAL_OFF})                   // nothing here
#define UI_COL_SOURCE ((UiColor){HUE_PURPLE, SAT_MAX, VAL_BASE})       // steady: already picked
#define UI_COL_MUTED ((UiColor){HUE_PURPLE, SAT_HIG, VAL_BASE})        // steady: output gated to 0V
#define UI_COL_UNMUTED ((UiColor){HUE_CYAN, SAT_HIG, VAL_BASE})        // steady: output passing, on the mute page
#define UI_COL_CONFIRM_WRITE ((UiColor){HUE_PURPLE, SAT_MAX, VAL_HIG}) // copy / save / assign committed
#define UI_COL_CONFIRM_CLEAR ((UiColor){HUE_PINK, SAT_MAX, VAL_HIG})   // clear committed
#define UI_COL_CONFIRM_LOAD ((UiColor){HUE_CYAN, SAT_MAX, VAL_HIG})    // preset loaded
#define UI_COL_ERROR ((UiColor){HUE_RED, SAT_MAX, VAL_HIG})            // slow blink; red is errors only

#endif /* INC_LIB_COLOR_PRESETS_H_ */
