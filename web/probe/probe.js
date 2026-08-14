// A physical BMCV, on the page.
//
// The chain is short because most of it already existed: an ST-Link reads the
// firmware's whole state out of RAM, and the wasm module - the same firmware
// code, compiled for the browser - is handed those bytes and asked what they
// mean. Nothing here decodes anything. There is no second copy of the LED
// curve, the unit conversions or the parameter maths, and there cannot be one,
// because this file never looks inside the blob.
//
// See docs/plans/probe-bridge.md, and Core/Inc/Lib/bmcv_probe.h for what the
// firmware publishes about itself.

import { sim } from '../sim.js';
import { mode } from '../mode.js';
import { Stlink, VERIFY_EVERY } from './stlink.js';

// Mirrors of Core/Inc/Lib/bmcv_probe.h. The two are checked against each other
// by web/smoke.mjs, which reads the header - the same arrangement the LED curve
// constants have, and for the same reason: two numbers that must agree and no
// compiler that can see both.
export const PROBE_INFO_ADDR = 0x08000200;
export const PROBE_MAGIC = 0x56434d42; // "BMCV"
export const PROBE_INFO_VERSION = 1;
const PROBE_INFO_BYTES = 28;

// Yield to the event loop without waiting.
//
// setTimeout cannot do this: after a few nested levels browsers clamp it to a
// 4ms minimum, which was a fixed tax on every snapshot - at 25ms a snapshot,
// a sixth of the budget spent asking to be woken up. A MessageChannel message
// is a macrotask like a timer callback, scheduled as soon as the current one
// finishes, and nothing clamps it.
//
// A yield at all, rather than looping straight into the next read, because
// rendering and clicks are macrotasks too and a loop of promise continuations
// would not let them in.
const channel = typeof MessageChannel !== 'undefined' ? new MessageChannel() : null;
const waiting = [];

if (channel) channel.port1.onmessage = () => waiting.shift()?.();

function yieldToEventLoop() {
  if (!channel) return new Promise(resolve => setTimeout(resolve, 0));
  return new Promise(resolve => {
    waiting.push(resolve);
    channel.port2.postMessage(0);
  });
}

// How hard to smooth the measured rate. The scopes redraw against it, and a
// number that jumped with every late frame would make a steady trace breathe.
const RATE_SMOOTHING = 0.1;

// What to tell someone about a failure they now have to act on. Only the
// timeout gets an addition, because it is the only one whose fix is not in the
// message: a probe left mid-exchange answers nothing, and while opening now
// resets it, a platform that refuses the reset needs the cable pulled.
function describe(err) {
  if (!/did not answer/.test(err.message)) return err.message;
  return `${err.message}. The probe's pipes are out of step with this page - `
    + 'connecting resets them, so try once more; if it keeps happening, unplug '
    + 'the probe and plug it back in.';
}

// Exported for web/frontend-check.mjs, which runs it against the bytes a real
// build actually puts in flash. It is the one part of the probe path that can
// be checked without hardware, and the one where a wrong offset would produce a
// plausible address rather than an error.
export function decodeInfo(bytes) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

  const magic = dv.getUint32(0, true);
  if (magic !== PROBE_MAGIC) {
    throw new Error(
      `no BMCV firmware here - 0x${PROBE_INFO_ADDR.toString(16)} reads 0x${magic.toString(16).padStart(8, '0')}, ` +
      'which is what an older build or another board looks like',
    );
  }

  const infoVersion = dv.getUint16(4, true);
  if (infoVersion !== PROBE_INFO_VERSION) {
    throw new Error(`this module publishes probe info v${infoVersion}; this page understands v${PROBE_INFO_VERSION}`);
  }

  const instanceSize = dv.getUint16(6, true);
  const instanceAddr = dv.getUint32(8, true);

  // NUL-terminated within its 16 bytes.
  const raw = bytes.subarray(12, 28);
  const end = raw.indexOf(0);
  const version = new TextDecoder().decode(end < 0 ? raw : raw.subarray(0, end));

  return { instanceSize, instanceAddr, version };
}

class Probe {
  constructor() {
    this.link = new Stlink();
    this.info = null;
    this.state = 'idle'; // idle | connecting | live | error
    // What open() is doing right now, so "connecting" is never just a word.
    this.stage = '';
    this.error = null;
    this.voltage = null;
    this.snapshots = 0;

    this.onchange = () => {};

    this.timer = null;
    this.stopping = false;
    this.inFlight = false;

    // Measured, not declared. A poll is several USB round trips and the browser
    // schedules the gaps between them; what comes out is nearer 25/s than 30 on
    // a good day and drops under load. The scopes plot samples at even spacing,
    // so if this number is wrong the time axis is wrong with it and a clean sine
    // reads as a wobbling one.
    this.hz = 0;
    this.lastPollAt = 0;

    // Samples taken back to back at that rate. Reset whenever the sampling
    // stops and starts again, so the scopes do not draw across the join.
    this.contiguous = 0;

    // Hidden tabs are throttled, not stopped, which is worse than stopping:
    // polls keep landing about once a second and fill the scope buffer with
    // history whose time axis bears no relation to the rest of it. Better to
    // stop deliberately and say so.
    this.paused = false;
    if (typeof document !== 'undefined') {
      document.addEventListener('visibilitychange', () => this.#visibilityChanged());
    }

    // Hand the probe back before the page goes away, so the next session does
    // not inherit a half-finished exchange. pagehide rather than beforeunload:
    // it fires for a refresh, a navigation and a tab being closed, and it is
    // the one the back/forward cache respects.
    //
    // Best effort by nature - nothing runs after a crash, and the close is
    // asynchronous while the unload is not, so it may not complete. The probe's
    // reset on the way back in is what makes that survivable; this only makes
    // it rare.
    if (typeof window !== 'undefined') {
      window.addEventListener('pagehide', () => {
        this.#stopPolling();
        this.link.close().catch(() => {});
      });
    }

    // The simulation as it was before a module took the page over. Watching
    // hardware must not cost you the patch you were working on: the first
    // import overwrites the whole instance, and without this the simulator
    // would come back holding the module's state and autosave it over the
    // browser's copy a couple of seconds later.
    this.saved = null;
  }

  #set(state, error = null) {
    this.state = state;
    this.error = error;
    mode.drivenBy(state === 'live' ? this.hz : null, this.contiguous);
    this.onchange(this);
  }

  // Nothing is drawn while the tab is hidden - requestAnimationFrame does not
  // run - so polling through it buys nothing and costs the continuity of the
  // buffer. Stop on the way out, start clean on the way back.
  #visibilityChanged() {
    if (this.state !== 'live') return;

    if (document.hidden) {
      this.paused = true;
      this.timer = null;
    } else if (this.paused) {
      this.paused = false;
      this.#startSampling();
      this.#schedule();
    }
    this.onchange(this);
  }

  // Begin a fresh run of samples: forget the measured interval, since the next
  // one would otherwise be however long the page spent in the background, and
  // forget the history, since the scopes must not draw across the join.
  #startSampling() {
    this.lastPollAt = 0;
    this.contiguous = 0;
  }

  // Whenever the page stops being live, however it stopped. Putting it here
  // rather than in disconnect() means an unplugged cable and a clicked button
  // leave the page in the same condition.
  #restoreSimulation() {
    if (!this.saved) return;
    sim.importInstance(this.saved);
    this.saved = null;
  }

  /* ---- connecting -------------------------------------------------------- */

  // From a click, always: the browser will not show its device picker
  // otherwise. Everything that can go wrong here is worth saying out loud, so
  // nothing is swallowed - a probe that cannot be claimed and a module that is
  // not powered look identical from the outside and have different fixes.
  async connect() {
    if (this.state === 'connecting' || this.state === 'live') return;
    this.#set('connecting');

    try {
      this.link.onStage = stage => {
        this.stage = stage;
        this.onchange(this);
      };
      await this.link.open();
      this.stage = '';

      // Before anything else, because "the target is not powered" explains
      // every subsequent failure and is otherwise invisible.
      this.voltage = await this.link.targetVoltage();
      if (this.voltage < 1.5) {
        throw new Error(`target reads ${this.voltage.toFixed(2)}V - is the module powered?`);
      }

      this.info = decodeInfo(await this.link.readMem(PROBE_INFO_ADDR, PROBE_INFO_BYTES));

      // The one check that matters before believing a byte of it. Both sides
      // were built from the same struct or they were not, and if they were not
      // then every reading downstream is plausible nonsense rather than an
      // error. sim/include/layout_target.h is what keeps them equal; this is
      // what notices when the module is running a different build than the
      // page was compiled from.
      if (this.info.instanceSize !== sim.instanceSize) {
        throw new Error(
          `the module's state is ${this.info.instanceSize} bytes and this page expects ${sim.instanceSize} - ` +
          `it is running firmware ${this.info.version}, built from different sources than this page`,
        );
      }

      this.saved = sim.exportInstance();

      // Start with empty hands. A button still down in the mailbox from a
      // previous session would otherwise be pressed on this module the moment
      // the first write lands.
      sim.remoteClear();

      this.#startSampling();
      await this.#poll(); // one now, so the panel is right before the first frame
      this.stopping = false;
      this.#set('live');
      this.#schedule();
    } catch (e) {
      await this.link.close().catch(() => {});
      this.#restoreSimulation();
      // A cancelled device picker is a decision, not a fault.
      const cancelled = e.name === 'NotFoundError';
      this.#set(cancelled ? 'idle' : 'error', cancelled ? null : describe(e));
    }
  }

  async disconnect() {
    this.#stopPolling();
    await this.#settle();

    // Let go on the way out, rather than leaving the module to time this page
    // out a quarter of a second later. Best effort: if the cable is already
    // gone the write fails, and the timeout is what covers that case anyway.
    sim.remoteClear();
    await this.#writeRemote().catch(() => {});

    await this.link.close();
    this.info = null;
    this.#restoreSimulation();
    this.#set('idle');
  }

  /* ---- polling ----------------------------------------------------------- */

  // Wait for whatever the loop is in the middle of before pulling the device
  // out from under it. A transfer abandoned half-done is exactly the state the
  // next session then has to recover from.
  async #settle() {
    for (let i = 0; i < 20 && this.inFlight; i++) {
      await new Promise(resolve => setTimeout(resolve, 10));
    }
  }

  // The page's own panel, into the module's memory.
  //
  // Every poll, not only when something was clicked: the sequence number is a
  // heartbeat as well as an update, and a module that stops hearing it lets go
  // of whatever this page was holding. That is what stops a refresh with a
  // button down from stranding the module holding it.
  //
  // Two transfers, because SWD moves words and the module may fold a tick
  // between them. The fields go first and the sequence number - the last word,
  // by construction; bmcv_sim.c asserts it - goes after, so a half-arrived
  // mailbox is simply not acted on yet.
  //
  // Cheap enough to do unconditionally: a write is a command and its data on
  // the OUT pipe with no reply to wait for, against a read's round trip.
  async #writeRemote() {
    const blob = sim.remoteBlob();
    const at = this.info.instanceAddr + sim.remoteOffset;

    await this.link.writeMem(at, blob.subarray(0, blob.length - 4));
    await this.link.writeMem(at + blob.length - 4, blob.subarray(blob.length - 4));
  }

  async #poll() {
    // Asking the probe whether the last access landed costs a round trip, and
    // the answer only changes when the cable comes out - at which point the
    // transfer itself fails, on this read or the next. Worth a third of the
    // snapshot budget to ask every time; worth almost nothing to ask often.
    const verify = this.snapshots % VERIFY_EVERY === 0;
    const bytes = await this.link.readMem(this.info.instanceAddr, this.info.instanceSize, { verify });
    if (!sim.importInstance(bytes)) throw new Error('the module\'s state was the wrong length for this build');
    this.snapshots++;
    this.contiguous++;

    // After the read, so what was just imported is the module as it was before
    // this update rather than midway through applying it.
    await this.#writeRemote();

    // Timed at the point the sample lands, so what is measured is the interval
    // the scope is actually plotting rather than the one that was asked for.
    const now = performance.now();
    if (this.lastPollAt) {
      const hz = 1000 / Math.max(1, now - this.lastPollAt);
      this.hz += (hz - this.hz) * RATE_SMOOTHING;
    }
    this.lastPollAt = now;
    mode.drivenBy(this.hz, this.contiguous);
  }

  #stopPolling() {
    this.stopping = true;
    this.paused = false;
    this.timer = null;
  }

  // Self-scheduling rather than setInterval, and never two at once. A poll is
  // several USB round trips; setInterval starts the next one on a fixed clock
  // whether or not the last has finished, so a slow read used to be followed by
  // an immediate one and the samples arrived in clumps - which the scope draws
  // as evenly spaced, because it has no way to know otherwise. Waiting a fixed
  // gap *after* each read is what makes the spacing regular.
  #schedule() {
    this.timer = true;
    yieldToEventLoop().then(() => this.#tick());
  }

  async #tick() {
    if (this.stopping || this.paused || this.state !== 'live') return;
    this.inFlight = true;
    try {
      await this.#poll();
      if (!this.stopping && !this.paused) this.#schedule();
    } catch (e) {
      // Unplugging it is the usual way this ends, and it is not an error worth
      // a red banner - but a bus fault is, and the message says which.
      this.#stopPolling();
      await this.link.close().catch(() => {});
      this.#restoreSimulation();
      this.#set('error', describe(e));
    } finally {
      this.inFlight = false;
    }
  }
}

export const probe = new Probe();

// WebUSB is Chromium's alone, and a page that simply has no button on Firefox
// is a bug report waiting to happen. The UI asks this so it can say why.
export const webusbAvailable = typeof navigator !== 'undefined' && !!navigator.usb;
