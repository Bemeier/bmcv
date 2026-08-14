// What is driving the page: the simulation, or a physical module over a probe.
//
// Its own module, tiny, because the things that need to ask are the panel's
// input handlers, the frame loop and the scopes, and none of them should have
// to import the whole probe stack - nor the probe import them - to find out.

import { TICK_US } from './const.js';

// The simulator captures one scope sample per engine tick.
const LOCAL_CAPTURE_HZ = 1e6 / TICK_US;

let live = false;
let captureHz = LOCAL_CAPTURE_HZ;
let contiguous = Infinity;

// Told only when `live` flips, not on every snapshot. Which half of the module
// box is shown is CSS off a body class, so this is for the things CSS cannot
// do - disabling a button that would otherwise look ready and do nothing.
const listeners = [];

export const mode = {
  // True while a physical module is driving the page. The panel is then a
  // display: its state arrives from hardware, and anything typed into it here
  // would be overwritten before it was drawn.
  get live() {
    return live;
  },

  // Scope samples per second. Not a constant, because it is a property of
  // whatever is filling the buffer: an engine tick in the simulator, a probe
  // read on hardware, and those differ by more than two orders of magnitude.
  get captureHz() {
    return captureHz;
  },

  // How many of the newest samples were taken at that rate, back to back.
  //
  // The buffer survives things the sampling does not. A tab in the background
  // has its timers throttled to about one a second and its animation frames
  // stopped altogether, so what sits in the ring after a few minutes elsewhere
  // is a handful of samples a second apart, immediately followed by samples ten
  // milliseconds apart - and a scope draws them evenly spaced, because it has
  // no way to know. Everything before a gap is discarded rather than drawn as
  // though the time axis meant anything across it.
  get contiguous() {
    return contiguous;
  },

  // One call rather than two settable properties, so the two facts can never
  // disagree - a page that thought it was live at 4kHz would draw fifty seconds
  // of scope and call it a third of one.
  //
  // `hz` is null to hand the page back to the simulator.
  // Call with what to do about it now, and again whenever it changes.
  onChange(fn) {
    listeners.push(fn);
    fn(live);
  },

  drivenBy(hz, samplesSinceGap = Infinity) {
    const wasLive = live;
    live = hz !== null;
    captureHz = hz ?? LOCAL_CAPTURE_HZ;

    // Forced, not taken on trust. The simulator fills the buffer every tick and
    // cannot have a gap in it, so handing the page back has to clear whatever
    // count the probe left behind - and the probe hands its own count over on
    // the way out. It did: disconnecting after ~900 snapshots left the
    // simulator drawing 900 of its 1500 samples, a trace that stopped 60% of
    // the way across the cell and stayed there.
    contiguous = hz === null ? Infinity : samplesSinceGap;

    // Only when it changes. The probe calls this on every snapshot to publish
    // its measured rate, which is a hundred times a second; touching the class
    // list that often is a style recalculation the browser did not need.
    if (live !== wasLive) {
      document.body.classList.toggle('live', live);
      for (const fn of listeners) fn(live);
    }
  },
};
