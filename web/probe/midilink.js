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
const CMD_IDENTITY_REQ = 0x02;
const CMD_REMOTE_INPUT = 0x20;
const CMD_STREAM_REQ = 0x21;
const CMD_SNAPSHOT = 0x22;

// How long one output gets to produce an answer before moving on. Two USB
// frames would do; this is loose enough for a busy machine and still leaves a
// bus with a dozen ports identified inside a second.
const IDENTIFY_MS = 150;

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

const RATE_SMOOTHING = 0.1;

// Which pair of ports on the bus is a BMCV?
//
// Not by name. The module's USB *product* string is "BMCV", but a MIDI port's
// name is whatever the host driver decides to call it, and this device
// publishes no iJack strings at all - every iJack in the class descriptor is 0
// - so there is nothing for a driver to name the port after. Windows is
// particularly fond of calling such a device "USB Audio Device". Matching on
// the name is how this failed the first time it met real hardware.
//
// So ask instead. Each output gets an identity request and every input is
// listened to; the first output whose request produces our reply is the
// module's, and the input that answered is its other half. That is the same
// command the update page uses to read the firmware version, and it identifies
// the module positively rather than guessing from a label.
//
// Sequential rather than all at once, because the point is to learn *which*
// output the answer belongs to - firing at all of them together would identify
// the input and leave the output ambiguous on a bus with more than one device.
async function identify(access, onStage) {
  const inputs = [...access.inputs.values()];
  const outputs = [...access.outputs.values()];

  if (!inputs.length || !outputs.length) {
    throw new Error('this browser sees no MIDI ports at all - is the module plugged in and powered?');
  }

  // Anything that does look like a BMCV is tried first, so the usual case
  // costs one round trip rather than a walk of the whole bus.
  const likely = p => /bmcv|bemeier/i.test(p.name ?? '');
  outputs.sort((a, b) => (likely(b) ? 1 : 0) - (likely(a) ? 1 : 0));

  for (const output of outputs) {
    onStage?.(`asking ${output.name}`);

    const answer = await new Promise(resolve => {
      const timer = setTimeout(() => resolve(null), IDENTIFY_MS);

      for (const input of inputs) {
        input.onmidimessage = ev => {
          const d = ev.data;
          if (d.length < 9 || d[0] !== 0xf0) return;
          if (d[1] !== SYSEX_ID[0] || d[2] !== SYSEX_ID[1] || d[3] !== SYSEX_ID[2]) return;
          if (d[4] !== CMD_IDENTITY_REQ) return;

          clearTimeout(timer);
          resolve({ input, version: `${d[5]}.${d[6]}.${d[7]}` });
        };
      }

      try {
        output.send([0xf0, ...SYSEX_ID, CMD_IDENTITY_REQ, 0xf7]);
      } catch {
        clearTimeout(timer);
        resolve(null); // a port that will not take a message is not the one
      }
    });

    for (const input of inputs) input.onmidimessage = null;
    if (answer) return { input: answer.input, output, version: answer.version };
  }

  const names = ps => ps.map(p => p.name || '(unnamed)').join(', ');
  throw new Error(
    `no BMCV answered. Outputs tried: ${names(outputs)}. Inputs listened to: ${names(inputs)}. `
    + 'If the module is plugged in, it is probably running firmware without the identity command.',
  );
}

class MidiLink {
  constructor() {
    this.state = 'idle'; // idle | connecting | live | error
    this.error = null;
    this.stage = '';     // which port is being asked, during discovery
    this.version = null; // what the module said it is running
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

      const found = await identify(this.access, stage => {
        this.stage = stage;
        this.onchange(this);
      });

      this.input = found.input;
      this.output = found.output;
      this.version = found.version;
      this.stage = '';

      this.saved = sim.exportInstance();
      sim.remoteClear();

      this.lastAt = 0;
      this.contiguous = 0;
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
      this.#restore();
      this.#set('error',
        `the module sent ${wire.length} encoded bytes and this page expects ${sim.wireSize} - `
        + 'it is running firmware built from different sources than this page');
      return;
    }

    // Ask for the next one only now, with this one decoded and adopted. That is
    // the whole of the pacing: the module cannot get ahead of what this thread
    // has actually managed to draw.
    this.#request();

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
