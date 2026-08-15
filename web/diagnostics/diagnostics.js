// Instruments for the link between this browser and a module.
//
// Every time a fault here was reasoned about it was diagnosed wrongly, and every
// time one of these was pointed at it the answer arrived in a single run. The
// snapshot rate separated a slow transport from a stalling one; the device check
// separates a driver that did not bind from a module that is not answering.
// Neither distinction is visible from the simulator page.
//
// See docs/live-module.md.

// wire.js rather than usblink.js, and the distinction is the point: usblink.js
// instantiates the wasm, and a page that measures a transport must not be
// measuring a page that loaded a simulator first.
import {
  readVersion, BMCV_VID, BMCV_PID, VENDOR_INTERFACE, EP_IN, EP_OUT, OP_SNAPSHOT_REQ,
} from '../probe/wire.js';

// How long the snapshot measurement runs.
const SNAPSHOT_SECONDS = 8;

// The instance size this page expects. Only used to size a read; the simulator
// page gets it from the wasm, and this one deliberately loads no wasm at all so
// that what it measures is the transport and nothing else.
const INSTANCE_BYTES = 2384;

const el = id => document.getElementById(id);
const ui = {
  status: el('status'), log: el('log'), verdict: el('verdict'),
};

function log(line) {
  ui.log.textContent += `${line}\n`;
}

function setStatus(text) {
  ui.status.textContent = text;
}

const fmt = (n, digits = 0) => n.toLocaleString(undefined, {
  minimumFractionDigits: digits, maximumFractionDigits: digits,
});

const kb = n => `${fmt(n / 1024, 1)} kB/s`;

/* ---- can a browser reach the module directly? ---------------------------- */

// The one question that decides whether the WebUSB route is available at all.
//
// On Windows a vendor interface is only reachable if the OS bound WinUSB to it,
// which the module's Microsoft OS 2.0 descriptors ask for. Whether that worked
// is awkward to establish from Device Manager and trivial to establish here: if
// Chrome offers the device and lets go of interface 1, it worked. If it does
// not appear in the picker at all, the descriptors did not take.

async function inspectDevice() {
  if (!navigator.usb) {
    log('This browser has no WebUSB.');
    return;
  }

  let device;
  try {
    // No filter, so the picker shows everything - a module that is missing from
    // an unfiltered list says something quite different from one missing from a
    // filtered one.
    device = await navigator.usb.requestDevice({ filters: [] });
  } catch (e) {
    log(`No device chosen (${e.name}).`);
    log('If BMCV was not in the list, Windows has not bound WinUSB to its vendor');
    log('interface, and the descriptors are what need looking at.');
    return;
  }

  log(`chosen: ${device.manufacturerName || '?'} ${device.productName || '?'}`);
  log(`  vendorId 0x${device.vendorId.toString(16)} productId 0x${device.productId.toString(16)}`);
  log(`  USB ${device.usbVersionMajor}.${device.usbVersionMinor}`);

  const expected = device.vendorId === BMCV_VID && device.productId === BMCV_PID;
  if (!expected) log('  (that is not the BMCV this page expects)');

  try {
    await device.open();
    log('  opened');
    if (!device.configuration) await device.selectConfiguration(1);

    for (const iface of device.configuration.interfaces) {
      const alt = iface.alternate;
      const eps = alt.endpoints.map(e => `${e.direction}${e.endpointNumber} ${e.type} ${e.packetSize}B`).join(', ');
      log(`  interface ${iface.interfaceNumber}: class ${alt.interfaceClass} [${eps || 'no endpoints'}]`
        + `${iface.claimed ? ' (claimed)' : ''}`);
    }

    // The decisive step. A system driver holding the interface refuses here.
    await device.claimInterface(VENDOR_INTERFACE);
    log(`  claimed interface ${VENDOR_INTERFACE} - WinUSB is bound and WebUSB works.`);

    // And what it is running, which is the first thing worth knowing when a
    // page and a module disagree about anything.
    const version = await readVersion(device).catch(e => `unavailable (${e.name})`);
    log(`  firmware: ${version ?? 'not reported - older firmware will not'}`);

    await device.releaseInterface(VENDOR_INTERFACE);
    await device.close();
  } catch (e) {
    log(`  FAILED: ${e.name}: ${e.message}`);
    log('  A device that opens but will not give up interface 1 is one Windows has');
    log('  bound something else to. One that will not open at all has no WinUSB.');
    try { await device.close(); } catch { /* already gone */ }
  }
}

/* ---- how fast can it actually go? ---------------------------------------- */

// Snapshots over the vendor interface, decoding nothing and drawing nothing.
//
// The gap between this and the simulator page is the whole point of running it
// here: fast here and slow there means the cost is in decoding and drawing;
// slow here too means the transport or the firmware. Nothing else distinguishes
// those, and guessing at it is what cost several evenings on the transport this
// replaced.
async function measureSnapshots(device, seconds = SNAPSHOT_SECONDS) {
  const size = INSTANCE_BYTES;
  const request = () => device.transferOut(EP_OUT, new Uint8Array([OP_SNAPSHOT_REQ]));

  await request();
  await request();

  const until = performance.now() + seconds * 1000;
  let count = 0, bytes = 0, short = 0, first = 0, last = 0;
  const gaps = [];

  while (performance.now() < until) {
    const r = await device.transferIn(EP_IN, size);
    if (r.status !== 'ok') {
      log(`  transfer reported "${r.status}"`);
      break;
    }
    const now = performance.now();
    if (!count) first = now; else gaps.push(now - last);
    last = now;
    count++;
    bytes += r.data.byteLength;
    if (r.data.byteLength !== size) short++;
    request().catch(() => {});
  }

  const elapsed = (last - first) / 1000;
  const rate = elapsed > 0 ? (count - 1) / elapsed : 0;
  const sorted = [...gaps].sort((a, b) => a - b);
  const median = sorted[Math.floor(sorted.length / 2)] ?? 0;
  const worst = sorted[sorted.length - 1] ?? 0;

  log(`${count} snapshots in ${fmt(elapsed, 2)}s, ${fmt(bytes / 1024, 0)} kB`);
  if (short) log(`${short} were not ${size} bytes`);

  ui.verdict.hidden = false;
  ui.verdict.textContent =
    `${fmt(rate, 1)} snapshots/s, decoding nothing and drawing nothing. `
    + `Gap: ${fmt(median, 1)}ms typical, ${fmt(worst, 0)}ms worst. `
    + (worst > median * 5
      ? 'That spread means they arrive in bursts, so something is stalling the stream rather than slowing it.'
      : 'Evenly spaced, so the transport itself is steady.');
}

/* ---- driving the module's clock ------------------------------------------ */

// A MIDI clock, sent to the module over the MIDI half of its own cable.
//
// The module follows MIDI Clock only when nothing is patched for the job -
// input_fold sets clock_source_is_midi from that, and the engine then counts 24
// pulses to the beat instead of 4. Which makes "is the module ignoring me or is
// nothing arriving?" the question this exists to answer: it reports what it
// actually sent, so a module that disagrees is a module, not a browser.

// What MIDI means by a clock: 24 to the quarter note. CLOCK_PULSES_PER_BEAT_MIDI
// in Core/Inc/Lib/clock_sync.h is the other end of this number.
const MIDI_CLOCKS_PER_BEAT = 24;

const MIDI_CLOCK = 0xf8;
const MIDI_START = 0xfa;
const MIDI_STOP = 0xfc;

// How far ahead to hand messages to the browser's MIDI scheduler, and how often
// to top that up.
//
// Timestamped rather than sent from a timer callback. setInterval is subject to
// every other thing this tab does and drifts by milliseconds at a time, which a
// module measuring intervals reads as tempo wobble - the browser's own MIDI
// clock does not, so the jitter never reaches the wire and what the module
// measures is what was asked for.
const LOOKAHEAD_MS = 250;
const REFILL_MS = 100;

const midiUi = {
  port: el('midi-out'), bpm: el('midi-bpm'),
  start: el('midi-start'), stop: el('midi-stop'),
  status: el('midi-status'), verdict: el('midi-verdict'),
};

let midiAccess = null;
let clockTimer = null;
let nextAt = 0;   // when the next clock is due, on the MIDI clock's timebase
let sentCount = 0;
let startedAt = 0;

const bpmNow = () => Math.min(300, Math.max(20, +midiUi.bpm.value || 69));
const clockIntervalMs = bpm => 60000 / (bpm * MIDI_CLOCKS_PER_BEAT);

function midiStatus(text) {
  midiUi.status.textContent = text;
}

function stopClock(reason) {
  if (clockTimer) clearInterval(clockTimer);
  clockTimer = null;

  const port = midiAccess?.outputs.get(midiUi.port.value);
  // Stop, so a DAW-like receiver does not sit there thinking it is running.
  // The module has no use for it - it follows pulses, not transport - but
  // leaving one out would make this tool lie about what it sends.
  try { port?.send([MIDI_STOP]); } catch { /* the port may already be gone */ }

  midiUi.start.disabled = false;
  midiUi.stop.disabled = true;
  midiStatus(reason ?? 'stopped');
}

// Hand the scheduler every clock due in the next LOOKAHEAD_MS.
//
// The interval is read on each message rather than once, so changing the bpm
// while it runs takes effect within a lookahead instead of needing a restart -
// which is the thing you actually want when watching a module follow it.
function pump() {
  const port = midiAccess?.outputs.get(midiUi.port.value);
  if (!port) return stopClock('the port went away');

  const until = performance.now() + LOOKAHEAD_MS;
  try {
    while (nextAt < until) {
      port.send([MIDI_CLOCK], nextAt);
      nextAt += clockIntervalMs(bpmNow());
      sentCount++;
    }
  } catch (e) {
    return stopClock(`${e.name}: ${e.message}`);
  }

  // What went out, as a rate, so this can be compared with what the module
  // says it sees rather than taken on trust.
  const elapsed = (performance.now() - startedAt) / 1000;
  if (elapsed > 0.5) {
    const hz = sentCount / elapsed;
    midiUi.verdict.hidden = false;
    midiUi.verdict.textContent =
      `sending ${fmt(bpmNow())} bpm - ${fmt(hz, 1)} clocks/s over ${fmt(elapsed, 1)}s. `
      + `The module should report ${fmt(hz * 60 / MIDI_CLOCKS_PER_BEAT, 1)} bpm. `
      + 'If it reports nothing, an input jack is still set to CLOCK and the '
      + 'module is ignoring MIDI on purpose.';
  }
}

function startClock() {
  const port = midiAccess?.outputs.get(midiUi.port.value);
  if (!port) return midiStatus('pick a port first');

  sentCount = 0;
  startedAt = performance.now();
  // A beat ahead, so the first clock is scheduled rather than late.
  nextAt = startedAt + 50;

  // Start before the pulses: a receiver that tracks transport wants to know the
  // clock is running, and the module reads it as a reset - which puts every
  // channel's phase at a known place before the tempo arrives.
  try { port.send([MIDI_START], startedAt + 25); } catch { /* reported below */ }

  pump();
  clockTimer = setInterval(pump, REFILL_MS);
  midiUi.start.disabled = true;
  midiUi.stop.disabled = false;
  midiStatus(`running on ${port.name}`);
}

async function initMidi() {
  if (!navigator.requestMIDIAccess) {
    midiStatus('this browser has no Web MIDI');
    return;
  }

  try {
    // No sysex: this sends three status bytes and nothing else, and asking for
    // sysex is a scarier permission prompt for no reason.
    midiAccess = await navigator.requestMIDIAccess();
  } catch (e) {
    midiStatus(`MIDI refused: ${e.name}`);
    return;
  }

  const fill = () => {
    const chosen = midiUi.port.value;
    midiUi.port.innerHTML = '';
    const ports = [...midiAccess.outputs.values()];
    if (!ports.length) {
      midiUi.port.append(new Option('no MIDI outputs', ''));
      midiUi.start.disabled = true;
      midiStatus('no MIDI outputs - is the module plugged in?');
      return;
    }
    for (const p of ports) midiUi.port.append(new Option(p.name, p.id));

    // The module's own port, if it is there. Named by the host driver from the
    // device's product string, so this is a guess and the picker is the answer
    // when it guesses wrong.
    const mine = ports.find(p => /bmcv/i.test(p.name));
    midiUi.port.value = chosen && ports.some(p => p.id === chosen) ? chosen : (mine ?? ports[0]).id;
    midiUi.start.disabled = false;
    midiStatus(mine ? 'found a port named BMCV' : 'pick the module\'s port');
  };

  fill();
  // Ports come and go - the module leaves the bus on a reflash, and this is the
  // page most likely to be open when it does.
  midiAccess.onstatechange = () => { if (!clockTimer) fill(); };
}

midiUi.start.addEventListener('click', startClock);
midiUi.stop.addEventListener('click', () => stopClock());
window.addEventListener('pagehide', () => { if (clockTimer) stopClock(); });

initMidi();

/* ---- wiring -------------------------------------------------------------- */

if (!navigator.usb) {
  setStatus('this browser has no WebUSB');
  for (const id of ['run-snapshots', 'run-webusb']) el(id).disabled = true;
} else {
  setStatus('ready');
}

async function withModule(fn) {
  const granted = await navigator.usb.getDevices();
  let device = granted.find(d => d.vendorId === BMCV_VID && d.productId === BMCV_PID);
  if (!device) {
    device = await navigator.usb.requestDevice({
      filters: [{ vendorId: BMCV_VID, productId: BMCV_PID }],
    });
  }

  await device.open();
  if (!device.configuration) await device.selectConfiguration(1);
  await device.claimInterface(VENDOR_INTERFACE);
  try {
    await fn(device);
  } finally {
    try { await device.close(); } catch { /* already gone */ }
  }
}

el('run-webusb')?.addEventListener('click', async () => {
  const button = el('run-webusb');
  button.disabled = true;
  ui.verdict.hidden = true;
  ui.log.textContent = '';
  setStatus('pick the BMCV in the browser\'s device list');

  try {
    await inspectDevice();
    setStatus('done - copy the log');
  } catch (e) {
    setStatus(`${e.name}: ${e.message}`);
  } finally {
    button.disabled = false;
  }
});

el('run-snapshots')?.addEventListener('click', async () => {
  const button = el('run-snapshots');
  button.disabled = true;
  ui.verdict.hidden = true;
  ui.log.textContent = '';

  try {
    setStatus(`streaming for ${SNAPSHOT_SECONDS}s`);
    await withModule(measureSnapshots);
    setStatus('done');
  } catch (e) {
    setStatus(`${e.name}: ${e.message}`);
  } finally {
    button.disabled = false;
  }
});
