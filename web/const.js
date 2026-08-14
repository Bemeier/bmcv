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

// The input that boots as INPUT_CLOCK, and so carries the clock generator.
export const CLOCK_INPUT = 0;

/* ---- scope -------------------------------------------------------------- */

// How much *time* a cell shows. Shared by both scopes so a trace means the same
// span in each.
//
// A duration rather than a sample count, which is what this was. The count only
// looked like a reasonable way to say it while one thing filled the buffer: at
// the engine's tick rate 1500 frames is 375ms, and at the ~30Hz a debug probe
// manages the same 1500 frames is fifty seconds of history drawn in a cell that
// claims to be the same width as the simulator's.
export const SCOPE_SECONDS = 0.375;

// And what it shows of a physical module. Longer deliberately: a probe samples
// far slower than the engine ticks, so 375ms of hardware is a handful of points
// with straight lines between them.
export const SCOPE_SECONDS_LIVE = 2;

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
export const CELL_GAP = 3;

// One colour for all eight traces: the position already says which channel it
// is, so a palette would only add noise.
export const TRACE = '#e6eaf0';

// The inputs are context for the outputs - what was fed in, against what came
// out - so they are drawn back from them rather than competing. Dim enough to
// read as the second thing on the page, bright enough to follow.
export const TRACE_IN = '#7d8794';
