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

// Frames of history per cell. Shared by both scopes so a trace means the same
// span of time in each.
export const SCOPE_SPAN = 1500;

// One colour for all eight traces: the position already says which channel it
// is, so a palette would only add noise.
export const TRACE = '#e6eaf0';
