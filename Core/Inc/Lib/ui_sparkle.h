#ifndef INC_LIB_UI_SPARKLE_H_
#define INC_LIB_UI_SPARKLE_H_

#include "color_presets.h" // IWYU pragma: keep - VAL_MED
#include <stdint.h>

// The assignment marker's light, as a field over the panel rather than a timer.
//
// Everything pickable used to flash white together on one gate: correct, and it
// read as a row of indicator lamps rather than as one surface. This replaces
// the gate with a value noise field in x, y and time, sampled at each LED's
// real position on the board - so the brightness travels across the panel and
// no two lights are quite in step.
//
// Two states wear it, and they are not the same errand:
//
//   SPARKLE_MARK    nothing is held. "You could pick these." An advertisement,
//                   so it runs for part of the period and rests for the rest,
//                   and an element it is not currently lighting still shows its
//                   own colour underneath.
//
//   SPARKLE_TARGET  a source is held and these are where it can go. Mid-gesture
//                   and no longer an invitation, so it never rests and never
//                   drops a light: the floor is the brightness the target used
//                   to sit at steadily, and the sparkle only adds above it.
//
// One field either way, so the panel has one idiom for "this can be pressed"
// and the urgency is what differs. Nothing here changes which elements are
// candidates.

// Lattice resolution of the field, over the box the LEDs occupy. Roughly one
// cell per light across, four rows down - fine enough that neighbours differ,
// coarse enough that the panel reads as one moving surface rather than as
// twenty-one independent flickers.
#ifndef MARK_FIELD_COLS
#define MARK_FIELD_COLS 8
#endif
#ifndef MARK_FIELD_ROWS
#define MARK_FIELD_ROWS 4
#endif

// The box the lights occupy, in board mm, taken from panel_led_pos. Not the
// board: the margins carry no lights, and spreading the field over them would
// spend most of it where nothing can show it. Over this box the two button rows
// - 11mm apart, against 65mm of panel - land two thirds of a cell apart and
// sparkle differently, which over the whole board they would not.
//
// Written out rather than scanned for, because a bound derived at every sample
// is a loop per LED per frame to recompute four numbers that only change when
// the board does. The tests hold them to the generated table instead.
#define MARK_FIELD_X0 5.5f
#define MARK_FIELD_X1 74.5f
#define MARK_FIELD_Y0 38.0f
#define MARK_FIELD_Y1 103.5f

// The marker's own period, and the share of it the sparkle runs for. Anything
// left over is a flash: every candidate full on together, once per period.
//
// 1.0 - no flash, sparkle the whole way through. The period then only matters
// to whatever the flash would have been, which is nothing, so at this setting
// MARK_SPARKLE_PERIOD does not appear on the panel at all.
//
// The flash is kept because it is the only thing that ever says "all of these"
// at once. Without it the guarantee is statistical - each light is lit often
// rather than lit for certain - and the gap a candidate can sit dark for grows
// from about two seconds to over three. Lower this below 1.0 to bring it back;
// 0.9 gives a 160ms flash on a 1600ms period.
#ifndef MARK_SPARKLE_PERIOD
#define MARK_SPARKLE_PERIOD 1600000
#endif
#ifndef MARK_SPARKLE_DUTY
#define MARK_SPARKLE_DUTY 1.0f
#endif

// Lattice cells the field travels per second: the rate an individual light
// pulses at. Fast enough to read as a sparkle rather than as a drift.
// How much of the flash is spent arriving, and the same again leaving. The rest
// holds at full on - which is the point of it, so the edges are a fraction and
// not the whole shape.
//
// A trapezoid with smoothstepped edges rather than a raised sine to a power.
// The power was flat-topped in the right way and had an infinite slope at the
// instant it began: at 0.25 the first millisecond of the flash covered a third
// of the range, which is a step wearing the shape of a curve. Smoothstep leaves
// and arrives at zero slope, so this joins the sparkle underneath it without a
// seam however short the edge is made.
#ifndef MARK_SPARKLE_FLASH_EDGE
#define MARK_SPARKLE_FLASH_EDGE 0.35f
#endif

// The sparkle itself has no envelope at all. It used to swell across its
// window, and that was the one input every light shared - the whole panel came
// up and down together with the field only deciding by how much, and at a
// pronounced swell there were moments with all twenty-one lit at once. With the
// flash carrying the "all of them" job, the field runs flat underneath it and
// stays the only thing deciding who is lit.

// How fast the field churns in place, in lattice cells per second: peaks fading
// up and down where they stand. On its own this is a twinkle with no direction
// to it.
//
// Read it as a lifetime. A feature lasts about 1 / this seconds before the
// noise has moved a whole cell along its own axis and replaced it - so 0.25 is
// a feature that lives four seconds.
#ifndef MARK_SPARKLE_HZ
#define MARK_SPARKLE_HZ 0.25f
#endif

// How fast the field slides across the panel, in lattice cells per second, left
// to right. Read it as a crossing time: MARK_FIELD_COLS / this is how long a
// feature takes to travel the whole row, so 5.0 crosses in about 1.6 seconds.
//
// The two numbers together are what decides whether any of this reads as
// direction, and the rule is not the ratio between them - it is that a feature
// has to *outlive its own crossing*. At 4.0 against a churn of 1.0 the crossing
// took two seconds and the feature lasted one: it was always replaced before it
// got across, and what reached the eye was indistinguishable from noise. Nobody
// could have found that by looking at the constants; it measured as a
// correlation of 0.02 between one light and the light downstream of it.
//
// A lifetime of two to three crossings is the useful range. Below one there is
// no travel to see. Far above it the same pattern is towed past over and over,
// which reads as mechanical.
#ifndef MARK_SPARKLE_DRIFT
#define MARK_SPARKLE_DRIFT 5.0f
#endif

// Brightness at the bottom of the field, as a share of the peak.
//
// Effectively a switch rather than a dial, and it is worth knowing why. The
// level is a *perceived* brightness laid over a base layer drawn at VAL_BASE,
// so nothing under (VAL_BASE / MARK_SPARKLE_V) ^ (1 / LED_GAMMA) - about 0.53 -
// changes what is on the panel at all. A floor above that lights every
// candidate for the whole window, which is legible and is the unison this was
// meant to get away from; anything below it, including 0, is the same as 0.
//
// Zero, then, and the guarantee moves: a candidate is no longer always lit, it
// is lit often. Each one crosses the threshold every half second or so, and
// only about once in four hundred windows does one sit out a whole window -
// which the tests hold to a bound, because that is the number that decides
// whether an element still reads as pickable.
#ifndef MARK_SPARKLE_FLOOR
#define MARK_SPARKLE_FLOOR 0.0f
#endif

// How peaked the field is. Value noise clusters around its middle; above 1 this
// pushes it toward the dark end so the bright points are points.
#ifndef MARK_SPARKLE_CONTRAST
#define MARK_SPARKLE_CONTRAST 2.5f
#endif

// Where the field starts registering, on the raw noise's own 0..1 scale.
// Everything below is dark and the rest is stretched over the full range.
//
// This is the knob for *how many* sparkles there are, and it is the only one
// that is. Contrast only trims their edges, and the rate trades count against
// length one for one - a field travelling twice as fast crosses twice as many
// peaks and spends half as long on each. Raising the gate cuts the count
// without touching how fast a peak is crossed, so it is what pairs with
// MARK_SPARKLE_HZ to set the two independently: gate for how often, rate for
// how long.
//
// Value noise almost never reaches its own extremes, so this is deep into the
// thin end of the distribution and small changes move the count a lot: 0.50
// sparkled each light about 1.6 times a second, 0.68 does it about 0.4. What
// that costs is at the bottom of this file.
#ifndef MARK_SPARKLE_GATE
#define MARK_SPARKLE_GATE 0.68f
#endif

// Where it reaches full. Together with the gate this is the window of raw noise
// the whole visible range is spent on, and it has to move with the gate: left
// behind, the window narrows to nothing and the remap becomes a hard threshold
// - which shortens every sparkle instead of thinning them out.
#ifndef MARK_SPARKLE_GATE_TOP
#define MARK_SPARKLE_GATE_TOP 0.90f
#endif

// The brightest a marked element gets, in whole duty steps, against the
// VAL_BASE its base layer is drawn at.
//
// Near VAL_BASE on purpose. The wash has two axes and they do not want the same
// amount: pulling the colour pale is the part that reads as a sparkle, and
// brightness on top of it mostly makes the panel restless.
//
// Twice the base is twice the light and, through LED_GAMMA, about a third more
// perceived brightness. Four times it was nearly double the brightness, which
// is what "it goes up a lot" was. The other half of it is that a sparkle has to
// be a strong one to brighten anything at all: below a level of about
// (VAL_BASE / this) ^ (1 / LED_GAMMA) the duty asked for is less than the
// element already has and led_wash leaves the brightness alone, so the quieter
// two thirds of the field are pure saturation.
#ifndef MARK_SPARKLE_V
#define MARK_SPARKLE_V (VAL_BASE * 2)
#endif

// The same for a destination under a held source, and set for a different
// reason: that one is drawn over a cleared LED, so it is not a lift over
// anything, it is the whole of the light. Nothing competes with it and it has
// to carry the page on its own, so it takes the top of the palette.
//
// The two land on the same number at the current levels - twice VAL_BASE is
// VAL_HIG - which is a coincidence of where VAL_BASE sits and not a shared
// setting. Moving VAL_BASE moves the mark and leaves this where it is.
#ifndef TARGET_SPARKLE_V
#define TARGET_SPARKLE_V VAL_HIG
#endif

// How much of an element's own colour survives at the top of the marker, on the
// SAT_* scale. The marker washes what it is over rather than whitening it: at
// SAT_MAX the colour is untouched and only the brightness moves, at 0 the
// element goes white and loses the one thing saying what it is.
//
// Read it as the amplitude of the wash: 255 minus this, over 255, is how far
// the colour is pulled toward neutral at the peak. 215 is about a sixth of the
// way, which on a SAT_HIG base takes the minor primaries from a tenth of the
// peak to a quarter. SAT_LOW here was three quarters of the way, and at that
// depth the wash was the loudest thing on the panel.
//
// Not much room below: past about a tenth the saturation stops carrying
// anything and the sparkle is back to being brightness on its own, which is the
// axis MARK_SPARKLE_V deliberately keeps small.
//
// White is still what a *destination* wears - those are drawn over a cleared
// LED, so there is no colour for this to keep and it has no effect there.
#ifndef MARK_SPARKLE_KEEP
#define MARK_SPARKLE_KEEP 215
#endif

// Brightness under a held source, as a share of the peak. The target is drawn
// over nothing - everything that is not a destination goes dark - so unlike the
// mark this has no colour beneath it to fall back to, and a light the field
// happened to leave out would read as "not a destination". The floor is what
// stops that, and it is the level the target used to hold steadily: at the
// defaults, VAL_BASE against a VAL_HIG peak.
#ifndef TARGET_SPARKLE_FLOOR
#define TARGET_SPARKLE_FLOOR 0.55f
#endif

// Which of the two is being drawn.
typedef enum
{
  SPARKLE_MARK = 0,
  SPARKLE_TARGET,
} SparkleKind;

// What the sparsity costs, measured over forty periods across all twenty-one
// lights at the settings above: each light sparkles about 0.41 times a second
// for about 150ms, a little over one light lit at any instant, and the longest
// a given light sits unlit is about nineteen seconds.
//
// That last number is the one to watch. It used to be under two, held there by
// the flash; without it the marker is a texture that happens to be legible
// rather than a guarantee, and on a page with one or two valid destinations
// there is no guarantee left at all. MARK_SPARKLE_DUTY at 0.9 buys it back for
// 160ms of every 1600.

// The marker's brightness for one LED, 0..1, at a moment. Zero for an LED index
// off the panel, and - for SPARKLE_MARK - for the part of the period the marker
// is not running.
//
// Pure: the same arguments always give the same level, which is what lets the
// whole effect be asserted without a framebuffer.
float ui_sparkle_level(uint32_t now_us, int16_t led, SparkleKind kind);

#endif /* INC_LIB_UI_SPARKLE_H_ */
