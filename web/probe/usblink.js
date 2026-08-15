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

export class UsbLink {
  constructor() {
    this.session = new Session(USB, { sendCommand: () => this.#sendCommand() });

    this.device = null;
    this.timers = [];
    this.reading = false;
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

      this.session.setStage('opening');
      await this.device.open();
      if (!this.device.configuration) await this.device.selectConfiguration(1);
      await this.device.claimInterface(VENDOR_INTERFACE);

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

  async disconnect() {
    // Let go of whatever this page was holding before dropping the link.
    sim.remoteClear();
    try { await this.#sendMailbox(); } catch { /* the cable may already be out */ }

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
    navigator.usb.removeEventListener('disconnect', this.#onDisconnect);

    if (!this.device) return;
    try { await this.device.close(); } catch { /* already gone */ }
    this.device = null;
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

    while (this.reading && this.device) {
      let result;
      try {
        result = await this.device.transferIn(EP_IN, sim.instanceSize);
      } catch (e) {
        if (!this.reading) return; // a disconnect we already know about
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
        await this.#teardown();
        this.session.end();
        this.session.set('error',
          `the module sent ${bytes.length} bytes and this page expects ${sim.instanceSize} - `
          + 'it is running firmware built from different sources than this page');
        return;
      }

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
  return `${err.name}: ${err.message}`;
}

export const usblink = new UsbLink();

export const webusbAvailable = typeof navigator !== 'undefined' && !!navigator.usb;
