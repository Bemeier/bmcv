// A physical BMCV on the page, over its own USB cable.
//
// The module carries a vendor-specific interface alongside its MIDI one, bound
// to WinUSB automatically by the descriptors in USB_Device/App/usbd_webusb.c,
// so there is nothing to install and no probe on the programming header. Two
// bulk endpoints: snapshots come in, requests and mailboxes go out.
//
// Nothing here decodes anything. The wasm is handed the bytes and asked what
// they mean - see sim.importInstance - so there is no struct parser in
// JavaScript and no second copy of any unit conversion.
//
// This replaced the same traffic carried over MIDI SysEx, which needed
// seven-bit encoding, framing across fifty-seven transfers, and a turn against
// the engine's own output on a shared endpoint. None of that is here. But the
// reason for moving was none of those: a browser enumerates MIDI once, so a
// module that leaves the bus - a power cycle, or a reflash - could not be
// reached again until the browser was restarted. WebUSB reports hot-plug
// properly and the device is picked when it is used.

import { sim } from '../sim.js';
import { USB } from '../mode.js';
import { Session } from './session.js';

// Mirrors of Core/Inc/Lib/usblink.h.
const OP_SNAPSHOT_REQ = 0x01;
const OP_REMOTE_INPUT = 0x02;
const OP_REMOTE_COMMAND = 0x03;
const OP_ENTER_DFU = 0x04;

const VENDOR_INTERFACE = 1;
const EP_IN = 2;
const EP_OUT = 2;

export const BMCV_VID = 0x0483;
export const BMCV_PID = 0x572b;

// One request buys one snapshot, so the page paces the module rather than the
// other way round. Two outstanding at a time so the endpoint does not idle for
// a round trip; the firmware caps it at USBLINK_MAX_CREDITS regardless.
const CREDITS = 2;

// The mailbox carries a sequence number the module reads as a heartbeat, so it
// has to keep arriving whether or not anything was touched.
const MAILBOX_MS = 40;

// How many times to try opening before giving up.
//
// The first attempt after the module has been sitting idle fails often enough
// to be worth this: the endpoints keep their data toggles across a browser
// letting go of the interface, so a device that was last spoken to by a
// previous session can answer the first transfer out of step and the read comes
// back short. clearHalt below resets the toggle, and this covers the case where
// the first exchange is already in flight when it does.
const OPEN_ATTEMPTS = 3;

// How many malformed frames in a row before the link is called off. A wrong
// length is usually a build mismatch, which no amount of retrying fixes - but
// it is also what a single interrupted transfer looks like, and ending a
// working session over one of those is worse than asking again.
const BAD_FRAMES_ALLOWED = 5;

// How long to wait for the module to answer before giving up on an attempt.
//
// WebUSB has no timeout of its own: a transferIn for data that never comes
// waits for ever, with no error and nothing to retry. That is survivable in the
// read loop, where the only cost is a link that has already stopped working -
// but during connect it means an attempt that can never fail, so the retries
// never happen and the switch that started it never finishes. Which locked out
// every button on the page, since a switch already running refuses to start
// another.
const ANSWER_TIMEOUT_MS = 400;

// The same for a link already running. Far looser, because at these rates the
// next snapshot is a few milliseconds away and anything approaching a second of
// silence is a stream that has stopped rather than one that is slow.
const STALL_TIMEOUT_MS = 1500;

// Reject if `promise` has not settled in time. The transfer itself cannot be
// cancelled - closing the device is what does that, which the caller does on
// its way round the retry loop.
function withTimeout(promise, ms, what) {
  return Promise.race([
    promise,
    new Promise((_resolve, reject) => setTimeout(() => reject(new Error(`${what} timed out`)), ms)),
  ]);
}

export class UsbLink {
  constructor() {
    this.session = new Session(USB, { sendCommand: () => this.#sendCommand() });

    this.device = null;
    this.timers = [];
    this.reading = false;

    // Bumped whenever a session ends, so a read still in flight cannot rejoin a
    // later one. Without it, a transfer left over from a previous connection
    // resolves after the next has set `reading` back to true, and two loops run
    // against one device.
    this.generation = 0;
  }

  get state() { return this.session.state; }
  get error() { return this.session.error; }
  get stage() { return this.session.stage; }
  get snapshots() { return this.session.snapshots; }
  get hz() { return this.session.hz; }

  set onchange(fn) { this.session.onchange = () => fn(this); }
  get onchange() { return this.session.onchange; }

  // What to show once it is running.
  get description() {
    return this.device
      ? `${this.device.productName || 'BMCV'}, serial ${this.device.serialNumber || 'none'}, no probe attached`
      : '';
  }

  /* ---- connecting -------------------------------------------------------- */

  // From a click, always: the browser will not show its device picker
  // otherwise. A device already granted comes back from getDevices() without
  // one, which is what makes reconnecting after a power cycle a click rather
  // than a browser restart.
  async connect() {
    if (this.state === 'connecting' || this.state === 'live') return;
    this.session.set('connecting');

    try {
      this.session.setStage('looking for a granted module');
      const granted = await navigator.usb.getDevices();
      this.device = granted.find(d => d.vendorId === BMCV_VID && d.productId === BMCV_PID);

      if (!this.device) {
        this.session.setStage('choose the module');
        this.device = await navigator.usb.requestDevice({
          filters: [{ vendorId: BMCV_VID, productId: BMCV_PID }],
        });
      }

      await this.#openWithRetries();

      this.session.setStage('');
      this.session.begin();

      // A device unplugged while open reports it, so the page can say so at the
      // time rather than on the next failed read.
      navigator.usb.addEventListener('disconnect', this.#onDisconnect);

      this.timers.push(setInterval(() => this.#sendMailbox(), MAILBOX_MS));

      for (let i = 0; i < CREDITS; i++) await this.#request();
      this.#readLoop();
    } catch (e) {
      await this.#teardown();
      this.session.end();
      // A cancelled device picker is a decision, not a fault.
      const cancelled = e.name === 'NotFoundError';
      this.session.set(cancelled ? 'idle' : 'error', cancelled ? null : describe(e));
    }
  }

  // Open, claim, and prove the module answers - retrying the lot if it does
  // not. Anything short of a whole snapshot means the endpoint was mid-thought
  // when we arrived, and starting over is cheaper than reasoning about it.
  async #openWithRetries() {
    let last;

    for (let attempt = 1; attempt <= OPEN_ATTEMPTS; attempt++) {
      this.session.setStage(attempt === 1 ? 'opening' : `opening, attempt ${attempt}`);
      try {
        await this.device.open();
        if (!this.device.configuration) await this.device.selectConfiguration(1);
        await this.device.claimInterface(VENDOR_INTERFACE);

        // Both directions, because a data toggle survives the browser letting
        // go of the interface: whatever the last session left the endpoint
        // expecting is what this one has to agree with, and this is what makes
        // them agree rather than hoping.
        await this.device.clearHalt('in', EP_IN).catch(() => {});
        await this.device.clearHalt('out', EP_OUT).catch(() => {});

        // One snapshot, before anything is built on top of it. A module that
        // answers a whole instance is a module this session can talk to.
        //
        // A previous session can leave the module holding credit it never got
        // to spend, so what comes back may be a snapshot from before this
        // attempt. That is still a whole instance and still proves the link, so
        // it is accepted rather than distinguished.
        await this.#request();
        const first = await withTimeout(
          this.device.transferIn(EP_IN, sim.instanceSize),
          ANSWER_TIMEOUT_MS,
          'the module',
        );
        if (first.status !== 'ok' || first.data.byteLength !== sim.instanceSize) {
          throw new Error(`the module answered ${first.data?.byteLength ?? 0} bytes of ${sim.instanceSize}`);
        }
        return;
      } catch (e) {
        last = e;
        // Closing is what cancels the transfer that just timed out, so the next
        // attempt starts against an endpoint with nothing outstanding.
        try { await withTimeout(this.device.close(), ANSWER_TIMEOUT_MS, 'closing'); } catch { /* already shut */ }
      }
    }

    throw last ?? new Error('the module could not be opened');
  }

  async disconnect() {
    // Let go of whatever this page was holding before dropping the link.
    sim.remoteClear();
    try {
      await withTimeout(this.#sendMailbox(), ANSWER_TIMEOUT_MS, 'the last mailbox');
    } catch { /* the cable may already be out, and this is a courtesy anyway */ }

    await this.#teardown();
    this.session.end();
    this.session.set('idle');
  }

  #onDisconnect = ev => {
    if (ev.device !== this.device) return;
    this.#teardown();
    this.session.end();
    this.session.set('error', 'The module was unplugged. Plug it back in and connect again.');
  };

  async #teardown() {
    for (const t of this.timers) clearInterval(t);
    this.timers = [];
    this.reading = false;
    this.generation++;
    navigator.usb.removeEventListener('disconnect', this.#onDisconnect);

    if (!this.device) return;

    // Bounded, like everything else that talks to the device. A close waits for
    // transfers to settle, and one of those may be a read for data that is
    // never coming - so an unbounded close is another way for a switch to hang
    // and take the buttons with it.
    const closing = this.device;
    this.device = null;
    try { await withTimeout(closing.close(), ANSWER_TIMEOUT_MS, 'closing'); } catch { /* already gone */ }
  }

  /* ---- the two directions ------------------------------------------------ */

  #send(bytes) {
    if (!this.device) return Promise.resolve();
    return this.device.transferOut(EP_OUT, bytes);
  }

  #request() {
    return this.#send(new Uint8Array([OP_SNAPSHOT_REQ]));
  }

  #sendMailbox() {
    const blob = sim.remoteBlob();
    const msg = new Uint8Array(1 + blob.length);
    msg[0] = OP_REMOTE_INPUT;
    msg.set(blob, 1);
    return this.#send(msg);
  }

  // Reset, or reset and forget storage. Sent once when asked for: the module
  // acts on a change of the command's sequence number, so repeating it would
  // change nothing and sending it late would be a button that appears not to
  // work.
  #sendCommand() {
    const blob = new Uint8Array(sim.commandBlob());
    const msg = new Uint8Array(1 + blob.length);
    msg[0] = OP_REMOTE_COMMAND;
    msg.set(blob, 1);
    return this.#send(msg);
  }

  // Hand the module to its bootloader. Here rather than over MIDI because the
  // page that wants it is already flashing over USB - see web/update.
  enterDfu() {
    return this.#send(new Uint8Array([OP_ENTER_DFU]));
  }

  // The same, on a device this link does not own. The update page opens the
  // module itself - it has no session and wants none - and this keeps the
  // opcode in one place rather than written out again over there.
  enterDfuOn(device) {
    return device.transferOut(EP_OUT, new Uint8Array([OP_ENTER_DFU]));
  }

  /* ---- reading ----------------------------------------------------------- */

  // One transferIn per snapshot. The instance is 2384 bytes and the endpoint
  // carries 64, but a bulk transfer is not a packet: the browser assembles them
  // and hands over the whole thing, so there is no reassembly on this side at
  // all - which is most of what this transport saved over the last one.
  async #readLoop() {
    this.reading = true;
    const generation = this.generation;

    // A frame that arrives malformed is dropped and asked for again rather than
    // ending the session. One is a hiccup; a run of them is a link that is not
    // going to recover by being asked more politely.
    let badFrames = 0;

    while (this.reading && this.device && this.generation === generation) {
      let result;
      try {
        // Bounded for the same reason the connect is, though far more loosely:
        // at these rates the next snapshot is a few milliseconds away, so a
        // second of silence is a stream that has stopped rather than one that
        // is slow. Unbounded, a stalled link leaves the page frozen on its last
        // snapshot with nothing said about why.
        result = await withTimeout(
          this.device.transferIn(EP_IN, sim.instanceSize),
          STALL_TIMEOUT_MS,
          'the module',
        );
      } catch (e) {
        if (!this.reading || this.generation !== generation) return; // a disconnect we already know about
        await this.#teardown();
        this.session.end();
        this.session.set('error', describe(e));
        return;
      }

      if (result.status !== 'ok') {
        // A stall is the module refusing, which nothing here should provoke.
        await this.#teardown();
        this.session.end();
        this.session.set('error', `the module's endpoint reported "${result.status}"`);
        return;
      }

      const bytes = new Uint8Array(result.data.buffer, result.data.byteOffset, result.data.byteLength);
      if (!sim.importInstance(bytes)) {
        if (++badFrames < BAD_FRAMES_ALLOWED) {
          this.#request().catch(() => {});
          continue;
        }

        await this.#teardown();
        this.session.end();
        this.session.set('error',
          `the module sent ${bytes.length} bytes and this page expects ${sim.instanceSize} - `
          + 'it is running firmware built from different sources than this page');
        return;
      }
      badFrames = 0;

      // Ask for the next only now, with this one decoded and adopted. That is
      // the whole of the pacing: the module cannot get ahead of what this
      // thread has actually managed to draw.
      this.#request().catch(() => {});
      this.session.adopted();
    }
  }
}

// What to tell someone about a failure they now have to act on.
function describe(err) {
  if (err.name === 'SecurityError') {
    return 'The browser refused access to the module. WebUSB needs a secure page - '
      + 'localhost counts, plain http on another host does not.';
  }
  if (err.name === 'NetworkError') {
    return 'The module could not be opened. On Windows this usually means another '
      + 'program is holding it, or it was unplugged mid-transfer.';
  }
  if (/timed out/.test(err.message)) {
    return `${err.message}. The module stopped answering - unplug it and plug it `
      + 'back in, or switch to a debug probe, which reads its memory directly and '
      + 'does not need it to be answering.';
  }
  return `${err.name}: ${err.message}`;
}

export const usblink = new UsbLink();

export const webusbAvailable = typeof navigator !== 'undefined' && !!navigator.usb;
