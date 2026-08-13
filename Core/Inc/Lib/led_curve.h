#ifndef INC_LIB_LED_CURVE_H_
#define INC_LIB_LED_CURVE_H_

#include "color_presets.h" // IWYU pragma: keep - VAL_MED

// Everything about how a number becomes light, in one file.
//
// Two halves, and the difference matters:
//
//   DEVICE   what the panel does. The firmware renders through these, so
//            changing one changes the module. The simulator reads the same
//            values because its job is to predict the module.
//
//   DISPLAY  how a screen should draw what the panel does. Read only by the
//            simulator and by web/leds.js; the firmware never sees them, so
//            these can be moved freely to make the sim easier to look at
//            without touching what gets flashed.
//
// Every constant is #ifndef-guarded, so a build that wants to differ overrides
// it without editing this file - which is how the two halves are tuned apart
// when a screen and a diffused 5050 in a dark rack disagree.

/* ==== DEVICE ============================================================ */

// --- framebuffer ---------------------------------------------------------

// EngineState.leds[] is 8.8 fixed point: LED_UNIT is one step of WS2812 duty.
//
// The fraction is not decoration. The ramp below spends most of its length at
// duty under 4 - that is what a comfortable ceiling costs - and at those levels
// integer rounding does not just band, it swings the hue, because red and green
// round to different fractions of what they were asked for. The framebuffer
// carries the intended colour and the flush quantises it, which is also what
// keeps the sim showing the ramp rather than the dither.
#define LED_FRAC_BITS 8
#define LED_UNIT (1 << LED_FRAC_BITS)

// --- die efficiency ------------------------------------------------------

// Light out per unit of duty, per primary. Only the ratios matter; they are
// written to average about 1.0 so VAL_MED keeps meaning "the comfortable
// brightness" everywhere else in the palette.
//
// This is the fix for the oldest defect in the bipolar ring: the green die is
// roughly twice the red one, so at equal duty +3V was visibly brighter than
// -3V and neither end of the scale could be trusted against the other. Nominal
// figures for a WS2812B - measure the real panel and correct them here.
#ifndef LED_W_RED
#define LED_W_RED 0.80f
#endif
#ifndef LED_W_GREEN
#define LED_W_GREEN 1.75f
#endif
#ifndef LED_W_BLUE
#define LED_W_BLUE 0.45f
#endif

// --- the panel's own response --------------------------------------------

// Perceived lightness -> luminance. Duty is linear in light output and the eye
// is not: L* goes as roughly Y^(1/3), and sRGB's 2.2 is the usual working
// approximation of that under normal viewing. Lower makes the bottom of a ramp
// brighter and the steps there coarser.
//
// Not specific to the CV ring: anything that fades or pulses goes through this
// on its way to a duty, or it spends its first half looking like nothing and
// then arrives all at once.
#ifndef LED_GAMMA
#define LED_GAMMA 2.20f
#endif

// --- the palette's brightness ------------------------------------------

// What a VAL_* means, in light rather than in duty.
//
// The palette names a value and expects a brightness, and until this existed it
// got a duty instead: at VAL_LOW a purple put out 6.7 units of light and a
// yellow 20.7, three times as much, because purple lights two of the weakest
// dies and yellow two of the strongest. Every page mixing hues was unevenly
// lit, and the brighter hues read as more important than the dimmer ones for no
// reason anyone chose.
//
// So led_set_hsv scales each colour until its luminance is the value it was
// asked for times this. Anchored on green because green is close to the mean of
// the hues the palette actually uses - so the panel keeps roughly the brightness
// it had, with the dim hues coming up and the bright ones coming down, rather
// than the whole thing moving.
//
// The cost is that a duty is no longer predictable from a VAL_*: a red at
// VAL_LOW now runs at 17 to match a green at 8. That is the point, and it is
// what the framebuffer's headroom is for.
#ifndef LED_PALETTE_REF
#define LED_PALETTE_REF LED_W_GREEN
#endif

// --- the bipolar CV ramp -------------------------------------------------

// Luminance at full scale, in duty on a nominal weight-1.0 die. This is the one
// number to reach for when the ring is too dim or too fierce as a whole -
// everything else here is shape.
//
// VAL_HIG rather than VAL_MED. The old ramp ran both dies to VAL_MAX, which put
// the top of the scale at four times the luminance of this and well into the
// range that is painful to look at; the first pass at replacing it went to
// VAL_MED and overcorrected, because weighting the dies cost the green end a
// further factor of two that the old unweighted numbers had been hiding.
//
// Weighted, so the actual duty is not this: green peaks near 30 and red near
// 70, which is what equal light on unequal dies costs. Red exceeding VAL_HIG on
// paper is not a brightness at all - it is the duty red needs to match what
// green does with less than half of it.
#ifndef LED_CV_CEIL
#define LED_CV_CEIL ((float) VAL_HIG)
#endif

// Share of the perceived range spent below 5V. The ramp is two straight lines
// in perceived-lightness space meeting here at half scale, so this one number
// says how much of the ring is spent on the range most patches live in. 0.5 is
// a straight perceptual ramp over the whole +/-10V.
#ifndef LED_CV_KNEE
#define LED_CV_KNEE 0.70f
#endif

// Where the ramp starts, in perceived lightness at 0V. The ramp spans from here
// to 1.0 rather than clamping across the bottom, so raising it lifts the whole
// scale instead of flattening the quiet end of it.
//
// Zero by default: off is off. The same ramp draws the input jacks and the
// parameter row, not only the channel rings, so anything above zero here lights
// an unpatched jack and a parameter sitting at its default - and a panel with
// nothing plugged into it should be dark. Raise it for a resting ember on every
// ring, which does give the eye a baseline to compare the lit ones against, at
// the cost of that.
//
// If it is raised, keep it clear of about 0.15: a *static* level below a
// quarter of a duty step makes the dither emit one step every four frames or
// fewer, and a pattern that slow reads as a flicker rather than as a colour.
// That only bites on a level that sits still, which at zero is every idle
// channel at once. It does not apply at 0.0f, where the duty is exactly zero
// and the dither has nothing to do, nor to signals passing through the same
// region on their way somewhere - they are moving, and movement dithers itself.
#ifndef LED_CV_FLOOR
#define LED_CV_FLOOR 0.0f
#endif

// How far either side of 0V the ember stays neutral, as a fraction of full
// scale. Inside it the two primaries are mixed toward equal - zero reads amber
// rather than as a very dim red or green - which makes the zero crossing itself
// visible. Polarity is unreadable in here, which is the trade: at 0.3V there is
// nothing to read.
#ifndef LED_CV_ZERO_SPAN
#define LED_CV_ZERO_SPAN 0.06f
#endif

// Luminance share handed to the other primary at full scale, ramping in from
// half scale. Past 5V the colour warms - green toward yellow-green, red toward
// orange - which is the only cue left that the signal is outside the usual
// range now that the blue axis is gone. The mix is luminance-preserving, so
// warming does not brighten. Set to 0 for pure primaries throughout.
#ifndef LED_CV_WARM
#define LED_CV_WARM 0.15f
#endif

/* ==== DISPLAY =========================================================== */

// Relative luminance of a screen's own primaries - Rec. 709, the sRGB ones.
//
// Needed because "how much light a die puts out" and "how blue something looks"
// are not the same question, and a display model that only knows the first gets
// the second badly wrong. A blue at SAT_HIG runs three times more green duty
// than it looks like it should, and green is nearly four times the blue die, so
// by luminance that colour is green-dominant - while to an eye it is plainly
// blue. Dividing each die's light by what the matching screen primary
// contributes turns emitted light back into a colour a screen can show.
//
// Without this the panel drew every hue pulled toward green and washed out: a
// blue came out pale cyan, white came out pale green.
#ifndef LED_Y_RED
#define LED_Y_RED 0.2126f
#endif
#ifndef LED_Y_GREEN
#define LED_Y_GREEN 0.7152f
#endif
#ifndef LED_Y_BLUE
#define LED_Y_BLUE 0.0722f
#endif

// Luminance that a screen should draw as full brightness, in the same units as
// LED_CV_CEIL. Handing a screen the raw duty bytes makes everything the
// renderer draws invisible, since the panel's own ceiling is a small fraction
// of 255.
//
// It has to be at least the brightest thing the renderer produces, or the sim
// clips and everything above some point on screen looks the same - which is
// what the old VAL_MED cap did to the CV ramp, and that was the half of the
// range most in need of judging.
//
// That brightest thing is a palette colour at VAL_HIG, which balancing puts at
// VAL_HIG * LED_PALETTE_REF of light - above the ramp's own ceiling, since
// VAL_HIG is what the palette reserves for being unmistakably brighter than
// everything around it.
#ifndef LED_DISPLAY_FULL
#define LED_DISPLAY_FULL ((float) VAL_HIG * LED_PALETTE_REF)
#endif

// Luminance -> sRGB, for the same reason LED_GAMMA exists, in the other
// direction.
#ifndef LED_DISPLAY_GAMMA
#define LED_DISPLAY_GAMMA 2.20f
#endif

#endif /* INC_LIB_LED_CURVE_H_ */
