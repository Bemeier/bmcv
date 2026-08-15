// Constants more than one module needs. Anything only one module cares about
// stays in that module - this is not a dumping ground for every number on the
// page, it is the short list of facts two files have to agree on.

/* ---- timing ------------------------------------------------------------- */

// The rate the frontend ticks the engine at. The engine is dt-driven so any
// value is correct; 4kHz is roughly what the hardware achieves.
export const TICK_US = 250;

// A backgrounded tab can come back with seconds of elapsed time. Catching up on
// all of it in one frame would freeze the page, so drop the excess instead.
export const MAX_CATCHUP_TICKS = 400;

/* ---- signal levels ------------------------------------------------------ */

export const GATE_V = 5.0;   // what a eurorack gate actually swings to
export const PULSE_MS = 10;  // comfortably above one engine tick
export const IN_V = 10;      // the input faders' range, and the input scope's
export const SCOPE_V = 10;   // an output scope cell spans +/- this many volts

/* ---- scope -------------------------------------------------------------- */

// How much *time* a cell shows. Shared by both scopes so a trace means the same
// span in each.
//
// A duration rather than a sample count, which is what this was. The count only
// looked like a reasonable way to say it while one thing filled the buffer: at
// the engine's tick rate 1500 frames is 375ms, and at the ~30Hz a debug probe
// manages the same 1500 frames is fifty seconds of history drawn in a cell that
// claims to be the same width as the simulator's.
//
// One number for all three sources. It was two - a short window for the
// simulation and a longer one for a module - which meant the same cell, the
// same width, showed five times as much time depending on what was driving it,
// and a trace that looked slow in one read as fast in the other. Switching
// source is meant to change where the picture comes from and nothing else
// about it.
//
// Two seconds, which is the longer of the two it replaces: enough to see a
// slow channel come round, and enough that a module sampled at ninety a second
// is a curve rather than a row of points. The simulator's ring is decimated to
// make it fit - see BMCV_SIM_SCOPE_DIV.
export const SCOPE_SECONDS = 2;

// The ground the traces are drawn on, read out of the stylesheet at load rather
// than named a second time here - two places holding the same colour is one
// place to forget.
//
// The same surface the panel sits on, so the two read as one material.
// Barely a fill - enough to settle the cells into a grid and take the edge off
// whatever is behind a trace; not enough to make the scopes a black rectangle
// laid over the page.
export const CELL_FILL = (() => {
  const fallback = 'rgba(0, 0, 0, .15)';
  if (typeof getComputedStyle !== 'function' || typeof document === 'undefined') return fallback;
  const v = getComputedStyle(document.documentElement).getPropertyValue('--surface').trim();
  return v || fallback;
})();

// The space between one cell and the next. With no outlines, the gap is what
// separates them - the page shows through it, exactly as it does between the
// readout cells.
//
// Four rather than three. At the narrower page width the cells sit closer
// together, and a gap that reads as a seam at 1600px reads as a shared edge at
// 1200 - two traces looking like one wide one is the thing this exists to
// prevent.
export const CELL_GAP = 4;

// One colour for all eight traces: the position already says which channel it
// is, so a palette would only add noise.
export const TRACE = '#e6eaf0';

// The inputs are context for the outputs - what was fed in, against what came
// out - so they are drawn back from them rather than competing. Dim enough to
// read as the second thing on the page, bright enough to follow.
export const TRACE_IN = '#7d8794';

/* ---- what a thing is set to ---------------------------------------------- */

// The module says what a mode *is* with a hue, and it uses four of them for
// everything - see HUE_STATE_* in Core/Inc/Lib/color_presets.h. The point of
// them is that they mean the same thing wherever they appear: a jack set to
// CLOCK and a channel set to PWM are both yellow because both are about
// discrete events, and that is worth more than either being memorable on its
// own.
//
// Approximations, deliberately. The module's are hues run through the LED
// curve, which is a different thing from a colour on a screen; what matters is
// that a channel reading PWM here is the colour the panel glows for PWM, not
// that the two match to a hex digit.
const STATE_DEFAULT = '#b39ddb'; // purple - off, disabled, neutral
const STATE_LEVEL = '#6fd8e0';   // cyan   - continuous, follows a level
const STATE_EVENT = '#e3c341';   // yellow - triggered, clocked, discrete steps
const STATE_RESET = '#e8705f';   // red    - resets something

// Both of these are the same array ui_render.c holds, in the order config.h
// declares the modes: input_mode_color[] and shape_mode_color[]. Written as the
// same four names rather than as hex so the two stay a mapping from mode to
// meaning, which is what they are on the module, rather than a list of colours
// that happen to agree.
export const INPUT_MODE_COLORS = [
  STATE_DEFAULT, // —
  STATE_EVENT,   // CLOCK
  STATE_RESET,   // RESET
  STATE_LEVEL,   // SLIDER
];

export const SHAPE_MODE_COLORS = [
  STATE_DEFAULT, // LFO     - a plain wavetable, the neutral case
  STATE_LEVEL,   // STEPPED - holds a level between steps
  STATE_EVENT,   // PWM     - an edge, and where it falls
];
