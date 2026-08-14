// A physical BMCV on the page, over its own USB port.
//
// The same two directions `probe.js` gets through an ST-Link, without the
// ST-Link: the module streams its whole state as SysEx on the MIDI endpoint it
// already enumerates, and takes the remote input mailbox back the same way.
//
// Nothing here decodes anything, exactly as in probe.js. The wasm is handed the
// bytes and asked what they mean, and the seven-bit codec that gets them there
// is the module's own sysex.c compiled to wasm rather than a copy written out
// in JS. See docs/plans/midi-transport.md.
//
// The shape differs from probe.js in one way that matters: an ST-Link is polled
// and a module *pushes*. There is no read loop here - snapshots arrive, and the
// only thing on a timer is the request that keeps them coming.

import { sim } from '../sim.js';
import { mode } from '../mode.js';

// Mirrors of Core/Inc/Lib/sysex.h.
const SYSEX_ID = [0x7d, 0x42, 0x4d];
const CMD_REMOTE_INPUT = 0x20;
const CMD_STREAM_REQ = 0x21;
const CMD_SNAPSHOT = 0x22;

// The module stops streaming SYSEX_STREAM_TIMEOUT_US (2s) after the last
// request, so this has to be comfortably under that. It is also what stops a
// module talking to a tab that has gone away.
const KEEPALIVE_MS = 500;

// The mailbox goes out on a timer rather than on every gesture: it carries a
// sequence number the module reads as a heartbeat, so it has to keep arriving
// whether or not anything was touched. Same bargain probe.js makes by writing
// it on every poll.
const MAILBOX_MS = 40;

const RATE_SMOOTHING = 0.1;

const isModule = port => /bmcv/i.test(port.name ?? '');

class MidiLink {
  constructor() {
    this.state = 'idle'; // idle | connecting | live | error
    this.error = null;
    this.snapshots = 0;
    this.onchange = () => {};

    this.access = null;
    this.input = null;
    this.output = null;
    this.timers = [];

    this.hz = 0;
    this.lastAt = 0;
    this.contiguous = 0;

    // The simulation as it was before a module took the page over, exactly as
    // probe.js keeps it: watching hardware must not cost you the patch you were
    // working on.
    this.saved = null;

    if (typeof window !== 'undefined') {
      window.addEventListener('pagehide', () => this.#teardown());
    }
  }

  #set(state, error = null) {
    this.state = state;
    this.error = error;
    mode.drivenBy(state === 'live' ? this.hz : null, this.contiguous);
    this.onchange(this);
  }

  /* ---- connecting -------------------------------------------------------- */

  async connect() {
    if (this.state === 'connecting' || this.state === 'live') return;
    this.#set('connecting');

    try {
      // sysex: true is the whole point, and it is a separate permission from
      // plain MIDI - a browser that grants one may still refuse the other.
      this.access = await navigator.requestMIDIAccess({ sysex: true });

      this.input = [...this.access.inputs.values()].find(isModule);
      this.output = [...this.access.outputs.values()].find(isModule);

      if (!this.input || !this.output) {
        const seen = [...this.access.inputs.values()].map(p => p.name).join(', ');
        throw new Error(`no BMCV on the MIDI bus - this browser can see: ${seen || 'nothing'}`);
      }

      this.saved = sim.exportInstance();
      sim.remoteClear();

      this.lastAt = 0;
      this.contiguous = 0;
      this.input.onmidimessage = ev => this.#onMessage(ev);

      this.#request();
      this.timers.push(setInterval(() => this.#request(), KEEPALIVE_MS));
      this.timers.push(setInterval(() => this.#sendMailbox(), MAILBOX_MS));

      // Nothing is live until a snapshot actually arrives - a module that is
      // plugged in but running firmware without the stream command answers
      // nothing, and saying "live" before then would be a lie the panel tells.
      this.watchdog = setTimeout(() => {
        if (this.state === 'connecting') {
          this.#teardown();
          this.#set('error', 'the module did not send anything - is it running firmware with snapshot streaming?');
        }
      }, 2000);
    } catch (e) {
      this.#teardown();
      this.#restore();
      this.#set('error', e.message);
    }
  }

  disconnect() {
    // Let go of whatever this page was holding before dropping the link, rather
    // than leaving the module to time the mailbox out a quarter second later.
    sim.remoteClear();
    try { this.#sendMailbox(); } catch { /* the port may already be gone */ }

    this.#teardown();
    this.#restore();
    this.#set('idle');
  }

  #teardown() {
    for (const t of this.timers) clearInterval(t);
    this.timers = [];
    clearTimeout(this.watchdog);
    if (this.input) this.input.onmidimessage = null;
  }

  #restore() {
    if (!this.saved) return;
    sim.importInstance(this.saved);
    this.saved = null;
  }

  /* ---- the two directions ------------------------------------------------ */

  #request() {
    this.output?.send([0xf0, ...SYSEX_ID, CMD_STREAM_REQ, 0xf7]);
  }

  #sendMailbox() {
    if (!this.output) return;
    const encoded = sim.sysex7Encode(sim.remoteBlob());
    this.output.send([0xf0, ...SYSEX_ID, CMD_REMOTE_INPUT, ...encoded, 0xf7]);
  }

  #onMessage(ev) {
    const d = ev.data;
    if (d.length < 6 || d[0] !== 0xf0) return;
    if (d[1] !== SYSEX_ID[0] || d[2] !== SYSEX_ID[1] || d[3] !== SYSEX_ID[2]) return;
    if (d[4] !== CMD_SNAPSHOT) return;

    // The browser's own MIDI stack did the reassembly - it does not deliver a
    // SysEx until its F7 arrives - so there is no framing code on this side at
    // all. What is left is: undo the seven-bit encoding, hand it over.
    const raw = sim.sysex7Decode(d.subarray(5, d.length - 1));

    if (!sim.importInstance(raw)) {
      this.#teardown();
      this.#restore();
      this.#set('error', `the module sent ${raw.length} bytes and this page expects ${sim.instanceSize} - it is running firmware built from different sources than this page`);
      return;
    }

    this.snapshots++;
    this.contiguous++;

    const now = performance.now();
    if (this.lastAt) {
      const hz = 1000 / Math.max(1, now - this.lastAt);
      this.hz += (hz - this.hz) * RATE_SMOOTHING;
    }
    this.lastAt = now;

    if (this.state !== 'live') {
      clearTimeout(this.watchdog);
      this.#set('live');
    } else {
      mode.drivenBy(this.hz, this.contiguous);
    }
  }
}

export const midilink = new MidiLink();

export const webmidiAvailable =
  typeof navigator !== 'undefined' && !!navigator.requestMIDIAccess;
