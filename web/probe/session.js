// The half of a live link that has nothing to do with how the bytes arrive.
//
// An ST-Link is polled and a module over MIDI pushes, so the two transports
// differ in almost everything about connecting and staying connected. What they
// do not differ in is what a *session* is: the simulation is put aside, the
// outgoing mailboxes are emptied, snapshots are counted, a rate is published,
// and on the way out the simulation comes back. That was written twice, and the
// two copies had already begun to disagree about how the rate was measured.
//
// Composition rather than a base class: neither transport is a kind of session,
// each one has one.

import { sim } from '../sim.js';
import { mode, SIM } from '../mode.js';

// Snapshots are counted over a window rather than smoothed from the gap between
// them. The gap is only the truth when arrivals are evenly spaced, and a pushed
// stream's are not: a burst of ten with no gap between them reads as a thousand
// a second, and an average of that against long silences once reported sixty
// while the panel was visibly updating about once. Counting cannot say that.
const RATE_WINDOW_MS = 500;

// Whichever session is currently driving the page, or null. Module-level
// because the page has exactly one - the two transports cannot both be live,
// since each holds the only handle its device offers - and because the reset
// buttons need to reach it without importing a transport.
let active = null;

export const activeSession = () => active;

export class Session {
  // `kind` is a mode.js source. `sendCommand` is how this transport delivers a
  // RemoteCommand - the one thing a session needs from its transport, and the
  // reason the reset buttons can be transport-agnostic.
  constructor(kind, { sendCommand }) {
    this.kind = kind;
    this.sendCommand = sendCommand;

    this.state = 'idle'; // idle | connecting | live | error
    this.error = null;
    this.stage = ''; // what connecting is doing right now
    this.snapshots = 0;

    this.hz = 0;
    this.contiguous = 0;
    this.onchange = () => {};

    // The simulation as it was before a module took the page over. Watching
    // hardware must not cost you the patch you were working on: the first
    // import overwrites the whole instance, and without this the simulator
    // would come back holding the module's state and autosave it over the
    // browser's copy a couple of seconds later.
    this.saved = null;

    this.windowCount = 0;
    this.windowAt = 0;
  }

  set(state, error = null) {
    this.state = state;
    this.error = error;
    mode.drivenBy(state === 'live' ? this.kind : SIM, this.hz, this.contiguous);
    this.onchange(this);
  }

  setStage(stage) {
    this.stage = stage;
    this.onchange(this);
  }

  // A module is about to take the page over.
  begin() {
    this.saved = sim.exportInstance();

    // Start with empty hands. A button still down in a mailbox from a previous
    // session would otherwise be pressed on this module the moment the first
    // write lands.
    sim.remoteClear();

    active = this;
    this.snapshots = 0;
    this.restartSampling();
  }

  // Begin a fresh run of samples: forget the measured rate, since the next
  // interval would otherwise be however long the page spent elsewhere, and
  // forget the history, since the scopes must not draw across the join.
  restartSampling() {
    this.contiguous = 0;
    this.windowCount = 0;
    this.windowAt = 0;
  }

  // One snapshot has landed and been adopted.
  adopted() {
    this.snapshots++;
    this.contiguous++;

    const now = performance.now();
    if (!this.windowAt) this.windowAt = now;
    this.windowCount++;

    const span = now - this.windowAt;
    if (span >= RATE_WINDOW_MS) {
      this.hz = (this.windowCount * 1000) / span;
      this.windowCount = 0;
      this.windowAt = now;
    }

    if (this.state !== 'live') this.set('live');
    else mode.drivenBy(this.kind, this.hz, this.contiguous);
  }

  // However the session ended - a click, an unplugged cable, a bus fault. Here
  // rather than in each disconnect path so all three leave the page in the same
  // condition.
  end() {
    if (active === this) active = null;
    if (this.saved) {
      sim.importInstance(this.saved);
      this.saved = null;
    }
  }
}
