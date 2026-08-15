// What is driving the page, and at what rate.
//
// Three sources, and the page is always in exactly one of them:
//
//   SIM    the simulation in this tab. Its MIDI output can be pointed at a real
//          port, which is the only thing optional about this mode.
//   USB    a physical module, over its own USB cable.
//   PROBE  a physical module, over an ST-Link on the programming header.
//
// The last two are the same picture by different means, so most of the page
// only wants to know `live`. What the two differ in is worth saying out loud
// where it matters - see web/probe/ui.js - and nowhere else.
//
// Its own module, tiny and dependency-free, because the things that need to ask
// are the panel's input handlers, the frame loop, the scopes and the readouts,
// and none of them should have to import a transport to find out.

import { sim } from './sim.js';
import { TICK_US } from './const.js';

export const SIM = 'sim';
export const USB = 'usb';
export const PROBE = 'probe';

// What each source is called where a person reads it.
export const SOURCE_NAME = {
  [SIM]: 'simulation',
  [USB]: 'module over USB',
  [PROBE]: 'module over probe',
};

// What rate the simulation fills the scope ring at: one sample per engine tick,
// divided by whatever the wasm decimates by. Asked rather than assumed - the
// divisor is the wasm's, and a copy of it here is a second definition free to
// drift from the one that actually writes the ring.
const LOCAL_CAPTURE_HZ = 1e6 / TICK_US / sim.scopeDiv;

let current = SIM;
let captureHz = LOCAL_CAPTURE_HZ;
let contiguous = Infinity;

// Told only when the source changes, not on every snapshot. Which parts of the
// page are shown is CSS off a body class, so this is for what CSS cannot do -
// relabelling a button, or saying what a reset is about to affect.
const listeners = [];

export const mode = {
  // Which of the three. Prefer `live` unless the difference actually matters.
  get current() {
    return current;
  },

  // True while a physical module is driving the page, by either route. What the
  // panel shows then arrives from hardware, and what is done to it goes back
  // out - see web/input.js.
  get live() {
    return current !== SIM;
  },

  // Scope samples per second. Not a constant, because it is a property of
  // whatever is filling the buffer: an engine tick in the simulator, a snapshot
  // over a link on hardware, and those differ by more than two orders of
  // magnitude.
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

  // Call with what to do about the current source now, and again whenever it
  // changes.
  onChange(fn) {
    listeners.push(fn);
    fn(current);
  },

  // One call rather than separate settable properties, so the facts can never
  // disagree - a page that thought it was live at 4kHz would draw fifty seconds
  // of scope and call it a third of one.
  //
  // Pass SIM to hand the page back to the simulation.
  drivenBy(source, hz = null, samplesSinceGap = Infinity) {
    const was = current;
    current = source;
    captureHz = source === SIM ? LOCAL_CAPTURE_HZ : (hz || 1);

    // Forced, not taken on trust. The simulator fills the buffer every tick and
    // cannot have a gap in it, so handing the page back has to clear whatever
    // count a link left behind - and a link hands its own count over on the way
    // out. It did: disconnecting after ~900 snapshots left the simulator
    // drawing 900 of its 1500 samples, a trace that stopped 60% of the way
    // across the cell and stayed there.
    contiguous = source === SIM ? Infinity : samplesSinceGap;

    // Only when it changes. A link calls this on every snapshot to publish its
    // measured rate, which is ninety times a second; touching the class list
    // that often is a style recalculation the browser did not need.
    if (current === was) return;

    // The history belonged to the source being left. Both write into one ring -
    // the simulation every other tick, a link on every snapshot - so without
    // this the scopes carry the old source's trace until the new one has filled
    // two seconds over it, and the join reads as something the module did.
    sim.scopeClear();

    document.body.classList.toggle('live', current !== SIM);
    for (const name of [SIM, USB, PROBE]) {
      document.body.classList.toggle(`mode-${name}`, current === name);
    }
    for (const fn of listeners) fn(current);
  },
};
