// An ST-Link, over WebUSB.
//
// Enough of the protocol to read and write a running Cortex-M's memory and
// nothing else: no flashing, no registers, no breakpoints, no halting. That is
// the whole of what showing a physical module needs, and it is a small, stable
// corner of a protocol that has not moved in years. openocd's `stlink_usb.c` is
// the reference implementation and the source of every constant below.
//
// Reads do not stop the core. A Cortex-M's debug unit reaches memory over the
// AHB-AP independently of what the CPU is doing, so the module keeps playing
// while it is being watched - which is the point, since what we want to see is
// it playing.
//
// Requirements a caller has to live with:
//
//   - **Chromium only.** WebUSB is not in Firefox or Safari and is not coming.
//   - **A user gesture.** requestDevice() must be called from a click.
//   - **A secure context.** localhost counts, so `just web` is fine.
//   - **The OS must not be holding the interface.** On Linux that means a udev
//     rule; on Windows the debug interface has to be bound to WinUSB, which is
//     what Zadig is for. This is the step most likely to be the problem.

const ST_VID = 0x0483;

// Top-level commands.
const CMD_GET_VERSION = 0xf1;
const CMD_DEBUG = 0xf2;
const CMD_DFU = 0xf3;
const CMD_GET_CURRENT_MODE = 0xf5;
const CMD_GET_TARGET_VOLTAGE = 0xf7;
const CMD_GET_VERSION_EX = 0xfb; // V3 only

// Sub-commands of CMD_DEBUG.
const DBG_READMEM_32BIT = 0x07;
const DBG_WRITEMEM_32BIT = 0x08;
const DBG_ENTER = 0x30; // APIv2
const DBG_EXIT = 0x21;
const DBG_GETLASTRWSTATUS = 0x3b;
const DBG_GETLASTRWSTATUS2 = 0x3e;
const DBG_INIT_AP = 0x4b;
const DBG_ENTER_SWD = 0xa3;

const DFU_EXIT = 0x07;

// What GET_CURRENT_MODE reports. A probe that was last used to flash something
// can still be sitting in DFU mode, and will refuse debug commands until it is
// told to leave.
const MODE_DFU = 0x00;
const MODE_MASS = 0x01;
const MODE_DEBUG = 0x02;

const ERR_OK = 0x80;


// Every command is a 16-byte packet, zero-padded. The device reads a fixed
// length and a short packet leaves it waiting.
const CMD_BYTES = 16;

// How much memory one READMEM_32BIT may ask for, and where the splits fall: a
// transfer may not cross a multiple of this.
//
// Every split costs a whole browser round trip, and browser round trips are
// what this is made of - so the fewer the better. Splits are also seams between
// two different instants of a module that never stopped running, which is the
// other reason to want none.
//
// 4KB is a Cortex-M3/M4's address auto-increment range and what openocd limits
// itself to. Whether the ST-Link's own firmware handles a longer read by
// re-arming the address register is not something to take on faith, so it is
// measured at connect instead - see #calibrateBlocks(). SAFE is the value that
// cannot be wrong; FAST is used only once it has been shown to give identical
// bytes.
const BLOCK_SAFE = 4096;
const BLOCK_FAST = 6144;

// How often the polling loop asks whether its reads are actually landing.
// Every read was two extra round trips for a question whose answer changes
// about once a session - when the probe is unplugged, and then every subsequent
// read fails at the transfer itself anyway.
export const VERIFY_EVERY = 32;

const ERRORS = {
  0x81: 'target fault',
  0x10: 'AP wait',
  0x11: 'AP fault',
  0x12: 'AP error',
  0x13: 'AP parity error',
  0x14: 'DP wait',
  0x15: 'DP fault',
  0x16: 'DP error',
  0x17: 'DP parity error',
  0x18: 'AP write data error',
  0x19: 'AP sticky error',
  0x1a: 'AP sticky overrun',
  0x1d: 'bad access port',
};

// Draining takes one max packet at a time. A bulk transfer completes when it is
// filled or the device sends a short packet, and a leftover reply is 2304 bytes
// - an exact multiple of 64 - so asking for "everything" would wait forever for
// a terminating short packet that never comes. One packet always completes on a
// full one.
const DRAIN_PACKET = 64;

// How long to wait for a packet before deciding the pipe is empty, and how many
// to take before deciding something else is wrong. One whole instance read is
// 36 packets; the status reply that may follow it is one more.
const DRAIN_QUIET_MS = 60;
const DRAIN_MAX_PACKETS = 96;

// A sentinel for "the deadline won", distinguishable from any transfer result.
const QUIET = Symbol('quiet');

// How long any single transfer may take before it is called a failure.
//
// Nothing here is slow: a read is under a couple of milliseconds and the
// handshake commands answer immediately. A transfer that has not finished in
// two seconds is not slow, it is never going to finish - and WebUSB transfers
// have no timeout of their own, so without this the page sits in "connecting"
// forever with no way to find out why.
const TRANSFER_TIMEOUT_MS = 2000;

// A transfer that cannot be allowed to hang. The underlying request is not
// cancellable, so a timed-out one stays queued on the endpoint - which is why
// timing out is treated as fatal to the connection rather than something to
// retry over: the next reply would go to the abandoned request.
function withTimeout(promise, what) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(
      () => reject(new StlinkError(`${what} did not answer within ${TRANSFER_TIMEOUT_MS}ms`)),
      TRANSFER_TIMEOUT_MS,
    );
    promise.then(
      value => { clearTimeout(timer); resolve(value); },
      err => { clearTimeout(timer); reject(err); },
    );
  });
}

// Thrown for anything the far end reports, so a caller can tell "the probe said
// no" from "the browser said no" - the two have completely different fixes.
export class StlinkError extends Error {
  constructor(message, code) {
    super(message);
    this.name = 'StlinkError';
    this.code = code;
  }
}

// An error with the step it happened at, since "did not answer" means something
// quite different during a version read than during a memory read.
function describeStage(err) {
  return err.stage ? `${err.message} (${err.stage})` : err.message;
}

function statusText(code) {
  return ERRORS[code] ?? `status 0x${code.toString(16)}`;
}

// The shape both memory commands share: sub-command, then a little-endian
// address and length. #send pads it out to the 16 bytes the device reads.
function memCommand(sub, addr, length) {
  const cmd = new Uint8Array(8);
  const dv = new DataView(cmd.buffer);
  cmd[0] = CMD_DEBUG;
  cmd[1] = sub;
  dv.setUint32(2, addr, true);
  dv.setUint16(6, length, true);
  return cmd;
}

export class Stlink {
  constructor() {
    this.device = null;
    // Where open() has got to, so a failure can say which step it failed at
    // rather than only what the failure was. Reported through onStage as it
    // happens, and carried on the error afterwards.
    this.stage = 'idle';
    this.onStage = null;
    this.iface = 0;
    this.epOut = 0;
    this.epIn = 0;
    this.version = null; // { major, jtag, name }

    // Raised to BLOCK_FAST at connect if the probe proves it can be trusted
    // with it. See #calibrateBlocks().
    this.block = BLOCK_SAFE;

    // Microseconds of wall time inside the last readMem, smoothed. What it is
    // for is telling "the USB is slow" apart from "something else is holding
    // the main thread", which look identical from the rate alone.
    this.readMs = 0;
  }

  get connected() {
    return this.device !== null && this.device.opened;
  }

  /* ---- connecting -------------------------------------------------------- */

  // Must be called from a user gesture. Shows the browser's device picker,
  // filtered to ST, then brings the probe up in SWD debug mode.
  //
  // Two attempts. A probe that was left mid-exchange by a page that went away -
  // a refresh, a crash, a closed tab - is still holding a reply nobody read,
  // and the first command of the next session gets *that* back instead of its
  // own answer. Everything after it is one reply behind and the handshake
  // stalls. Nothing on the target clears it, which is why power-cycling the
  // module does not help and replugging the probe does.
  //
  // So: try it clean, and if anything at all goes wrong, do the replug in
  // software and try once more. Reset only when it is needed rather than every
  // time, because a port reset is disruptive in its own right - on some
  // platforms it invalidates the handle we are holding.
  async open() {
    if (!navigator.usb) {
      throw new Error('this browser has no WebUSB - Chromium is the only one that does');
    }

    this.device = await navigator.usb.requestDevice({ filters: [{ vendorId: ST_VID }] });

    try {
      await this.#bringUp(false);
    } catch (first) {
      // Tagged where it happened, not where it is caught - this.stage is still
      // whatever #bringUp had reached when it threw.
      first.stage ??= this.stage;
      this.onStage?.('resetting the probe');
      try {
        await this.#bringUp(true);
      } catch (second) {
        second.stage ??= this.stage;
        // The first failure is the interesting one - the second is what the
        // recovery made of it. Both are reported, in that order.
        throw new StlinkError(`${describeStage(first)}; after a reset, ${describeStage(second)}`);
      }
    }

    return this.version;
  }

  // Everything from an unopened device to one answering memory reads. Torn down
  // and repeated on failure, so it has to be safe to run twice.
  async #bringUp(withReset) {
    await this.#teardown();

    this.stage = 'opening the device';
    this.onStage?.(this.stage);
    await this.device.open();

    if (withReset) {
      this.stage = 'resetting the device';
      this.onStage?.(this.stage);
      // Best effort. Some platforms refuse it outright, and where it is
      // refused the transfer timeouts are what stop a stale reply hanging us.
      try {
        await this.device.reset();
      } catch { /* see above */ }
    }

    // reset() can drop the configuration, so this is checked after it.
    this.stage = 'selecting a configuration';
    this.onStage?.(this.stage);
    if (this.device.configuration === null) await this.device.selectConfiguration(1);

    this.stage = 'claiming the debug interface';
    this.onStage?.(this.stage);
    this.#findInterface();
    await this.device.claimInterface(this.iface);

    this.stage = 'resetting the endpoints';
    this.onStage?.(this.stage);
    await this.#resync();

    this.stage = 'clearing the previous session';
    this.onStage?.(this.stage);
    await this.#drain();

    this.stage = 'reading the probe version';
    this.onStage?.(this.stage);
    await this.#readVersion();
    if (this.version.major < 2) {
      throw new Error(`ST-Link V${this.version.major} speaks a different protocol - V2 or later is needed`);
    }

    this.stage = 'entering SWD debug mode';
    this.onStage?.(this.stage);
    await this.#enterDebugMode();

    this.stage = 'measuring the transfer size';
    this.onStage?.(this.stage);
    await this.#calibrateBlocks();

    this.stage = 'connected';
  }

  // Back to an unopened device, from wherever we got to. Every step is allowed
  // to fail: this runs on the way in to a retry, so the thing it is undoing may
  // never have happened.
  async #teardown() {
    if (!this.device) return;
    try {
      if (this.device.opened) await this.device.releaseInterface(this.iface);
    } catch { /* may not have been claimed */ }
    try {
      if (this.device.opened) await this.device.close();
    } catch { /* may not have been open */ }
  }

  // Leave the probe as we found it: out of debug mode, interface released,
  // device closed. Every step tolerates having nothing to do, because this also
  // runs from the page's unload handler and from failed connections.
  async close() {
    if (!this.device) return;

    // Told to stop debugging, rather than simply abandoned. A probe left in
    // debug mode is the state the next session has to talk its way out of.
    try {
      if (this.device.opened && this.version) await this.#xfer([CMD_DEBUG, DBG_EXIT], 0);
    } catch { /* it may already be gone - that is what we wanted anyway */ }

    await this.#teardown();
    this.device = null;
    this.version = null;
    this.stage = 'idle';
  }

  // The debug interface is the vendor-specific one. A V2-1 or V3 also presents
  // mass storage and one or two serial ports, and those belong to the OS -
  // picking by interface number rather than by class lands on whichever the
  // probe happened to enumerate first.
  #findInterface() {
    for (const iface of this.device.configuration.interfaces) {
      const alt = iface.alternate;
      if (alt.interfaceClass !== 0xff) continue;

      const outs = alt.endpoints.filter(e => e.direction === 'out' && e.type === 'bulk');
      const ins = alt.endpoints.filter(e => e.direction === 'in' && e.type === 'bulk');
      if (!outs.length || !ins.length) continue;

      this.iface = iface.interfaceNumber;
      this.epOut = outs[0].endpointNumber;
      // Two IN endpoints on every ST-Link since V2: the lower is data, the
      // higher is the SWO trace stream we do not use.
      this.epIn = Math.min(...ins.map(e => e.endpointNumber));
      return;
    }
    throw new Error('no debug interface on this device - is it an ST-Link?');
  }

  // Put both pipes back in step with the device's idea of them.
  //
  // This is what a refresh breaks. A bulk endpoint carries a DATA0/DATA1 toggle
  // that host and device advance together, and a page that vanishes mid-read
  // leaves the host's pipe torn down with a transfer outstanding while the
  // device's toggle has moved on. Reopening gives us a fresh pipe starting at
  // DATA0; the ST-Link answers with DATA1; the host discards it as a retransmit
  // and waits for a DATA0 that is never sent. The probe looks like it has
  // stopped answering, which is exactly what "did not answer within 2000ms
  // (reading the probe version)" is.
  //
  // CLEAR_FEATURE(ENDPOINT_HALT) - what clearHalt sends - resets that toggle to
  // DATA0 at the device. It is the one thing short of re-enumeration that fixes
  // it, and it is why unplugging the probe worked and everything else did not.
  //
  async #resync() {
    for (const [direction, endpoint] of [['in', this.epIn], ['out', this.epOut]]) {
      try {
        await this.device.clearHalt(direction, endpoint);
      } catch {
        // Not every platform allows it on an endpoint that is not halted, and
        // a probe that did not need it is the common case.
      }
    }

    // SET_INTERFACE resets the toggles of every endpoint in the interface at
    // once, and does it in the device rather than only in the host's idea of
    // it. Devices with a single alternate setting are allowed to stall it, so
    // this is best effort like the rest.
    try {
      await this.device.selectAlternateInterface(this.iface, 0);
    } catch { /* single-setting interfaces may refuse */ }
  }

  // Take back whatever the last session walked away from.
  //
  // This is the state a refresh leaves behind. The poll loop reads 2304 bytes
  // seventy times a second, so a page that disappears almost certainly does so
  // with a read outstanding: the ST-Link has begun handing over a reply and is
  // waiting for the rest of it to be collected. Its firmware will not look at
  // the command pipe until that finishes, which is why it goes silent rather
  // than answering wrongly - and why nothing on the target end helps.
  //
  // The trap, and the reason an earlier attempt at this broke healthy probes:
  // a read issued against an *empty* pipe never completes, and a WebUSB
  // transfer cannot be cancelled. Left pending it swallows the first real reply
  // of this session - the very failure being undone. The only thing that does
  // cancel it is closing the device it belongs to. So that is what happens: if
  // the deadline passes with nothing to collect, the handle is dropped and
  // retaken, which takes the abandoned read down with it.
  async #drain() {
    let discarded = 0;

    for (let i = 0; i < DRAIN_MAX_PACKETS; i++) {
      const result = await this.#readBefore(DRAIN_PACKET, DRAIN_QUIET_MS);

      if (result === QUIET) {
        // Nothing there - either it was clean all along, or we have just taken
        // the last of it. Either way there is a read stuck on the endpoint now.
        await this.#reopen();
        return discarded;
      }

      const got = result?.data?.byteLength ?? 0;
      discarded += got;
      if (!got) return discarded;
    }

    return discarded;
  }

  // A read with a deadline, where losing the race is survivable only because
  // the caller closes the device afterwards.
  #readBefore(length, ms) {
    const read = this.device.transferIn(this.epIn, length).catch(() => null);
    const deadline = new Promise(resolve => setTimeout(() => resolve(QUIET), ms));
    return Promise.race([read, deadline]);
  }

  // Drop the handle and take it again, cancelling anything queued against it.
  async #reopen() {
    await this.#teardown();
    await this.device.open();
    if (this.device.configuration === null) await this.device.selectConfiguration(1);
    await this.device.claimInterface(this.iface);
    await this.#resync();
  }

  /* ---- the wire ---------------------------------------------------------- */

  // Commands are a fixed 16 bytes, zero-padded; the device reads that many and
  // a short packet leaves it waiting.
  #packet(bytes) {
    const cmd = new Uint8Array(CMD_BYTES);
    cmd.set(bytes);
    return cmd;
  }

  async #send(bytes) {
    const r = await withTimeout(this.device.transferOut(this.epOut, this.#packet(bytes)), 'the probe');
    if (r.status !== 'ok') throw new StlinkError(`command not accepted (${r.status})`);
  }

  async #recv(length) {
    const r = await withTimeout(this.device.transferIn(this.epIn, length), 'the probe');
    if (r.status !== 'ok') throw new StlinkError(`no reply (${r.status})`);
    return new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength);
  }

  // One command, one reply. `length` of zero is for the commands that do not
  // answer - asking for a reply that is not coming stalls until the transfer
  // times out.
  async #xfer(bytes, length) {
    await this.#send(bytes);
    return length ? await this.#recv(length) : null;
  }

  /* ---- bringing it up ---------------------------------------------------- */

  async #readVersion() {
    const r = await this.#xfer([CMD_GET_VERSION], 6);

    // Big-endian, uniquely in this protocol: three fields packed into 16 bits.
    const v = (r[0] << 8) | r[1];
    let major = (v >> 12) & 0x0f;
    let jtag = (v >> 6) & 0x3f;

    if (major === 3) {
      // A V3 reports 3 here and nothing else useful; the real versions come
      // from a command a V2 does not have.
      const ex = await this.#xfer([CMD_GET_VERSION_EX], 12);
      major = ex[0];
      jtag = ex[2];
    }

    // Checked for plausibility, not just read. Six bytes left over from a
    // previous session parse into a version quite happily - and every command
    // after them is then one reply behind. An ST-Link is V2 or V3 with a
    // two-digit firmware revision; anything else means the pipe is carrying
    // somebody else's answer, and saying so here is what sends open() to its
    // retry with a port reset.
    if (major < 2 || major > 3 || jtag < 1 || jtag > 63) {
      throw new StlinkError(`implausible version (V${major}, JTAG ${jtag}) - the reply does not belong to this command`);
    }

    this.version = { major, jtag, name: `ST-Link V${major} (JTAG firmware ${jtag})` };
  }

  async #enterDebugMode() {
    const mode = (await this.#xfer([CMD_GET_CURRENT_MODE], 2))[0];

    // A probe left in DFU mode by a flashing tool answers debug commands with
    // silence. Mass-storage mode is the V2-1's drive and needs nothing.
    if (mode === MODE_DFU) await this.#xfer([CMD_DFU, DFU_EXIT], 0);

    // Entered unconditionally, including when the probe already reports debug
    // mode: that only says it is debugging something, not that it is doing it
    // over SWD, and this board has no JTAG pins brought out.
    await this.#xfer([CMD_DEBUG, DBG_ENTER, DBG_ENTER_SWD], 2);

    // Newer firmware wants the access port opened before it will answer a
    // memory read. Older firmware has no such command and says so; that is not
    // a failure, so it is not treated as one.
    if (this.version.major >= 3 || this.version.jtag >= 28) {
      try {
        await this.#xfer([CMD_DEBUG, DBG_INIT_AP, 0], 2);
      } catch {
        // Nothing to do about it here - if the port really is not open, the
        // first read says so with a status that names the problem.
      }
    }
  }

  // Volts at the target's VDD, measured by the probe. Worth having because
  // "nothing works" and "the module is not powered" look identical otherwise,
  // and this tells them apart before any of the rest is attempted.
  async targetVoltage() {
    const r = await this.#xfer([CMD_GET_TARGET_VOLTAGE], 8);
    const dv = new DataView(r.buffer, r.byteOffset, r.byteLength);
    const ref = dv.getUint32(0, true);
    const target = dv.getUint32(4, true);
    return ref === 0 ? 0 : (2 * target * 1.2) / ref;
  }

  /* ---- memory ------------------------------------------------------------ */

  // Whether the last read or write actually reached the target. The transfer
  // itself succeeding only means the probe accepted the command; a bus fault
  // on the far side is reported separately, and skipping this check is how a
  // read of an unmapped address turns into plausible-looking zeroes.
  //
  // Asked once per call rather than once per chunk. It reports the *last*
  // access, so in principle a fault on an early chunk followed by a good one
  // would be missed - in practice a faulting access means a bad address or a
  // target that has gone, and the next chunk faults too. The trade is worth
  // naming: it is two USB round trips out of eight per snapshot, and this runs
  // as fast as the bus allows.
  async #checkStatus() {
    const big = this.version.jtag >= 15;
    const sub = big ? DBG_GETLASTRWSTATUS2 : DBG_GETLASTRWSTATUS;
    const r = await this.#xfer([CMD_DEBUG, sub], big ? 12 : 2);
    if (r[0] !== ERR_OK) throw new StlinkError(`target refused the access: ${statusText(r[0])}`, r[0]);
  }

  // Read `length` bytes from `addr`. Both must be 4-byte aligned - this is the
  // 32-bit access command, and the probe answers a misaligned request with
  // rubbish rather than an error.
  //
  // `verify` asks the probe afterwards whether the accesses landed. It costs a
  // round trip, so the polling loop does it periodically rather than every time
  // - see VERIFY_EVERY.
  async readMem(addr, length, { verify = true, block = this.block } = {}) {
    if (addr % 4 || length % 4) throw new Error(`readMem needs 4-byte alignment, got ${addr}+${length}`);

    const started = performance.now();
    const out = new Uint8Array(length);
    let done = 0;

    while (done < length) {
      const at = addr + done;
      // Never let one transfer cross a block boundary.
      const room = block - (at % block);
      const n = Math.min(length - done, room);

      // Command out and reply in are issued together rather than one after the
      // other. They are separate endpoints, so the IN request can already be
      // waiting when the device answers - the device NAKs until it has
      // something, exactly as it would if we asked later. What that saves is a
      // full browser round trip per chunk, and browser round trips are the
      // entire cost here: the bytes themselves are nothing.
      const [sent, got] = await withTimeout(Promise.all([
        this.device.transferOut(this.epOut, this.#packet(memCommand(DBG_READMEM_32BIT, at, n))),
        this.device.transferIn(this.epIn, n),
      ]), `a read of 0x${at.toString(16)}`);

      if (sent.status !== 'ok') throw new StlinkError(`command not accepted (${sent.status})`);
      if (got.status !== 'ok') throw new StlinkError(`no reply (${got.status})`);

      const chunk = new Uint8Array(got.data.buffer, got.data.byteOffset, got.data.byteLength);
      if (chunk.length !== n) {
        throw new StlinkError(`short read at 0x${at.toString(16)}: wanted ${n}, got ${chunk.length}`);
      }

      out.set(chunk, done);
      done += n;
    }

    if (verify) await this.#checkStatus();

    this.readMs = this.readMs ? this.readMs * 0.9 + (performance.now() - started) * 0.1
                              : performance.now() - started;
    return out;
  }

  // Can this probe be trusted with a read longer than the target's address
  // auto-increment range, or does it need them split at 4KB?
  //
  // Asked rather than assumed, because guessing wrong does not fail - it
  // returns the wrong bytes, and a module drawn from the wrong bytes looks like
  // a module doing something strange rather than like a bug.
  //
  // Flash is the one region that is guaranteed not to change underneath the
  // question: read a stretch spanning a 4KB boundary in one go, read the same
  // stretch split at that boundary, and require them to be identical. A running
  // module's RAM could differ between two reads for honest reasons; its
  // firmware image cannot.
  async #calibrateBlocks() {
    const at = 0x08001000 - 256; // 256 bytes either side of a boundary
    const n = 512;

    try {
      const whole = await this.readMem(at, n, { block: BLOCK_FAST });
      const split = await this.readMem(at, n, { block: BLOCK_SAFE });
      const same = whole.length === split.length && whole.every((v, i) => v === split[i]);
      this.block = same ? BLOCK_FAST : BLOCK_SAFE;
    } catch {
      // A probe that will not read flash at all is a problem for the caller to
      // discover on the read it actually wanted, with the address it wanted.
      this.block = BLOCK_SAFE;
    }
  }

  // Write `bytes` to `addr`, same alignment rule. The data goes out as a second
  // transfer on the same endpoint, after the command.
  async writeMem(addr, bytes) {
    if (addr % 4 || bytes.length % 4) {
      throw new Error(`writeMem needs 4-byte alignment, got ${addr}+${bytes.length}`);
    }

    let done = 0;
    while (done < bytes.length) {
      const at = addr + done;
      const room = this.block - (at % this.block);
      const n = Math.min(bytes.length - done, room);

      await this.#send(memCommand(DBG_WRITEMEM_32BIT, at, n));
      const r = await withTimeout(
        this.device.transferOut(this.epOut, bytes.subarray(done, done + n)),
        `a write to 0x${at.toString(16)}`,
      );
      if (r.status !== 'ok') throw new StlinkError(`write not accepted (${r.status})`);

      done += n;
    }

    await this.#checkStatus();
  }
}
