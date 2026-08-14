// A physical BMCV on the page, over its own USB port.
//
// The same two directions `probe.js` gets through an ST-Link, without the
// ST-Link: the module streams its whole state as SysEx on the MIDI endpoint it
// already enumerates, and takes the remote input mailbox back the same way.
//
// Nothing here decodes anything, exactly as in probe.js. The wasm is handed the
// bytes and asked what they mean, and the seven-bit codec that gets them there
// is the module's own sysex.c compiled to wasm rather than a copy written out
// in JS. See docs/live-module.md.
//
// The shape differs from probe.js in one way that matters: an ST-Link is polled
// and a module *pushes*. There is no read loop here - snapshots arrive, and the
// only thing on a timer is the request that keeps them coming.

import { sim } from '../sim.js';
import { MIDI } from '../mode.js';
import { Session } from './session.js';
import { identify, SYSEX_ID, CMD_REMOTE_INPUT, CMD_REMOTE_COMMAND, CMD_STREAM_REQ, CMD_SNAPSHOT } from './midiports.js';

// One request buys one snapshot, so this page paces the module rather than the
// other way round: it asks again once it has finished with the last one. That
// is the debug probe's arrangement - read, finish, read again - and it is why
// the probe's rate is steady where a free-running module's was not.
//
// Two outstanding at the start, so the module can begin the next snapshot while
// this thread is still decoding the previous one and the USB does not sit idle
// for a round trip. The firmware caps it at SYSEX_STREAM_MAX_CREDITS anyway.
const CREDITS = 2;

// Nothing else keeps the stream alive, so a stall has to be noticed rather than
// waited on. If a request or a snapshot goes missing the credit is gone and the
// module falls silent, so re-arm after a quiet spell.
const STALL_MS = 750;

// The mailbox goes out on a timer rather than on every gesture: it carries a
// sequence number the module reads as a heartbeat, so it has to keep arriving
// whether or not anything was touched. Same bargain probe.js makes by writing
// it on every poll.
const MAILBOX_MS = 40;

class MidiLink {
  constructor() {
    this.session = new Session(MIDI, { sendCommand: () => this.#sendCommand() });

    this.access = null;
    this.input = null;
    this.output = null;
    this.version = null;
    this.timers = [];

    if (typeof window !== 'undefined') {
      window.addEventListener('pagehide', () => this.#teardown());
    }
  }

  // The session owns everything that is not about MIDI, and the UI reads these
  // off the link rather than reaching through it.
  get state() { return this.session.state; }
  get error() { return this.session.error; }
  get stage() { return this.session.stage; }
  get snapshots() { return this.session.snapshots; }
  get hz() { return this.session.hz; }

  set onchange(fn) { this.session.onchange = () => fn(this); }
  get onchange() { return this.session.onchange; }

  /* ---- connecting -------------------------------------------------------- */

  async connect() {
    if (this.state === 'connecting' || this.state === 'live') return;
    this.session.set('connecting');

    try {
      // sysex: true is the whole point, and it is a separate permission from
      // plain MIDI - a browser that grants one may still refuse the other.
      this.access = await navigator.requestMIDIAccess({ sysex: true });

      const found = await identify(this.access, stage => this.session.setStage(stage));

      this.input = found.input;
      this.output = found.output;
      this.version = found.version;
      this.session.setStage('');

      this.session.begin();
      this.input.onmidimessage = ev => this.#onMessage(ev);

      for (let i = 0; i < CREDITS; i++) this.#request();
      this.timers.push(setInterval(() => this.#checkStall(), STALL_MS));
      this.timers.push(setInterval(() => this.#sendMailbox(), MAILBOX_MS));

      // Nothing is live until a snapshot actually arrives - a module that is
      // plugged in but running firmware without the stream command answers
      // nothing, and saying "live" before then would be a lie the panel tells.
      this.watchdog = setTimeout(() => {
        if (this.state === 'connecting') {
          this.#teardown();
          this.session.end();
          this.session.set('error', 'the module did not send anything - is it running firmware with snapshot streaming?');
        }
      }, 2000);
    } catch (e) {
      this.#teardown();
      this.session.end();
      this.session.set('error', e.message);
    }
  }

  disconnect() {
    // Let go of whatever this page was holding before dropping the link, rather
    // than leaving the module to time the mailbox out a quarter second later.
    sim.remoteClear();
    try { this.#sendMailbox(); } catch { /* the port may already be gone */ }

    this.#teardown();
    this.session.end();
    this.session.set('idle');
  }

  #teardown() {
    for (const t of this.timers) clearInterval(t);
    this.timers = [];
    clearTimeout(this.watchdog);
    if (this.input) this.input.onmidimessage = null;
  }

  /* ---- the two directions ------------------------------------------------ */

  #request() {
    this.output?.send([0xf0, ...SYSEX_ID, CMD_STREAM_REQ, 0xf7]);
  }

  // Credit is the only thing keeping the stream running, so a dropped message
  // in either direction stops it for good rather than costing one frame. Cheap
  // to put back: an extra request the module does not need is dropped by its
  // own cap.
  #checkStall() {
    if (this.state !== 'live') return;
    if (performance.now() - this.lastAt < STALL_MS) return;
    for (let i = 0; i < CREDITS; i++) this.#request();
  }

  #sendMailbox() {
    if (!this.output) return;
    const encoded = sim.sysex7Encode(sim.remoteBlob());
    this.output.send([0xf0, ...SYSEX_ID, CMD_REMOTE_INPUT, ...encoded, 0xf7]);
  }

  // Reset, or reset and forget storage. Sent once when asked for rather than on
  // the mailbox timer: it is an edge the module acts on when its sequence
  // number changes, so repeating it would change nothing and sending it late
  // would be a button that appears not to work.
  #sendCommand() {
    if (!this.output) return;
    const encoded = sim.sysex7Encode(new Uint8Array(sim.commandBlob()));
    this.output.send([0xf0, ...SYSEX_ID, CMD_REMOTE_COMMAND, ...encoded, 0xf7]);
  }

  // What to show once it is running: which port it turned out to be, since the
  // name is exactly the thing that could not be relied on to find it.
  get description() {
    return this.input ? `${this.input.name || '(unnamed port)'}, firmware ${this.version}` : '';
  }

  #onMessage(ev) {
    const d = ev.data;
    if (d.length < 6 || d[0] !== 0xf0) return;
    if (d[1] !== SYSEX_ID[0] || d[2] !== SYSEX_ID[1] || d[3] !== SYSEX_ID[2]) return;
    if (d[4] !== CMD_SNAPSHOT) return;

    // The browser's own MIDI stack did the reassembly - it does not deliver a
    // SysEx until its F7 arrives - so there is no framing code on this side at
    // all. What is left is: undo the seven-bit encoding, hand it over.
    const wire = d.subarray(5, d.length - 1);

    if (!sim.importSysex7(wire)) {
      this.#teardown();
      this.session.end();
      this.session.set('error',
        `the module sent ${wire.length} encoded bytes and this page expects ${sim.wireSize} - `
        + 'it is running firmware built from different sources than this page');
      return;
    }

    // Ask for the next one only now, with this one decoded and adopted. That is
    // the whole of the pacing: the module cannot get ahead of what this thread
    // has actually managed to draw.
    this.#request();

    this.lastAt = performance.now();
    clearTimeout(this.watchdog);
    this.session.adopted();
  }
}

export const midilink = new MidiLink();

export const webmidiAvailable =
  typeof navigator !== 'undefined' && !!navigator.requestMIDIAccess;
