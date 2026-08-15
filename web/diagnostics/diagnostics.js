// Instruments for the link between this browser and a module.
//
// Every time a fault here was reasoned about it was diagnosed wrongly, and every
// time one of these was pointed at it the answer arrived in a single run. The
// snapshot rate separated a slow transport from a stalling one; the device check
// separates a driver that did not bind from a module that is not answering.
// Neither distinction is visible from the simulator page.
//
// See docs/live-module.md.

import { readVersion } from '../probe/usblink.js';

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

/* ---- finding the module -------------------------------------------------- */

// By handshake, not by name - see midiports.js. A MIDI port is named by the
// host driver and this device gives it nothing to work with.
const findPorts = access => identify(access, stage => setStatus(stage));

/* ---- can a browser reach the module directly? ---------------------------- */

// The one question that decides whether the WebUSB route is available at all.
//
// On Windows a vendor interface is only reachable if the OS bound WinUSB to it,
// which the module's Microsoft OS 2.0 descriptors ask for. Whether that worked
// is awkward to establish from Device Manager and trivial to establish here: if
// Chrome offers the device and lets go of interface 1, it worked. If it does
// not appear in the picker at all, the descriptors did not take.
const BMCV_VID = 1155;
const BMCV_PID = 22315;
const BMCV_VENDOR_INTERFACE = 1;

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
    await device.claimInterface(BMCV_VENDOR_INTERFACE);
    log(`  claimed interface ${BMCV_VENDOR_INTERFACE} - WinUSB is bound and WebUSB works.`);

    // And what it is running, which is the first thing worth knowing when a
    // page and a module disagree about anything.
    const version = await readVersion(device).catch(e => `unavailable (${e.name})`);
    log(`  firmware: ${version ?? 'not reported - older firmware will not'}`);

    await device.releaseInterface(BMCV_VENDOR_INTERFACE);
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
  const OP_SNAPSHOT_REQ = 0x01;
  const EP = 2;

  const size = INSTANCE_BYTES;
  const request = () => device.transferOut(EP, new Uint8Array([OP_SNAPSHOT_REQ]));

  await request();
  await request();

  const until = performance.now() + seconds * 1000;
  let count = 0, bytes = 0, short = 0, first = 0, last = 0;
  const gaps = [];

  while (performance.now() < until) {
    const r = await device.transferIn(EP, size);
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
  await device.claimInterface(BMCV_VENDOR_INTERFACE);
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
