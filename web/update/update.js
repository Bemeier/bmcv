// Page logic for the firmware updater: talk to the running module over
// WebMIDI to get it into DFU, then talk to ST's ROM bootloader over WebUSB to
// write the image.
//
// The two halves are independent on purpose. If the firmware on the module is
// too broken to answer MIDI, the user holds FN2 at power-on and the WebUSB half
// still works on its own.

import { requestDfuDevice, describeImageProblem, FLASH_START, DfuError } from './dfuse.js';

import { usblink, BMCV_VID, BMCV_PID } from '../probe/usblink.js';

// F0 7D 42 4D <cmd> F7 - see Core/Inc/Lib/sysex.h. 0x7D is the non-commercial
// manufacturer ID; 'B','M' after it is what keeps this from colliding with
// every other project using the same ID.

const el = (id) => document.getElementById(id);

const ui = {
  unsupported: el('unsupported'),
  mainFlow: el('main-flow'),
  windowsNote: el('windows-note'),
  btnConnect: el('btn-connect'),
  btnReboot: el('btn-reboot'),
  midiStatus: el('midi-status'),
  releaseSelect: el('release-select'),
  file: el('file'),
  fileStatus: el('file-status'),
  btnFlash: el('btn-flash'),
  flashStatus: el('flash-status'),
  progress: el('progress'),
  log: el('log'),
};

let image = null;
let moduleDevice = null;

function log(message, cls) {
  const line = document.createElement('div');
  line.textContent = message;
  if (cls) line.className = cls;
  ui.log.append(line);
  ui.log.scrollTop = ui.log.scrollHeight;
}

function setStatus(node, message, cls) {
  node.textContent = message;
  node.className = cls ?? 'muted';
}

// --- module side, over MIDI ------------------------------------------------

// Find the module and ask it to reboot into its bootloader.
//
// Over the module's own vendor interface rather than over MIDI, which is what
// this used to be. The page already flashes over WebUSB, so this makes the
// whole of flashing one transport - and it removes the failure that made the
// MIDI version unreliable in exactly this situation: a browser enumerates MIDI
// once, and the thing this page does most is make the module leave the bus.
async function connectModule() {
  if (!navigator.usb) {
    setStatus(ui.midiStatus, 'no WebUSB in this browser', 'warn');
    log('This browser has no WebUSB. Use the FN2 route instead.', 'warn');
    return;
  }

  try {
    const granted = await navigator.usb.getDevices();
    let device = granted.find(d => d.vendorId === BMCV_VID && d.productId === BMCV_PID);
    if (!device) {
      device = await navigator.usb.requestDevice({
        filters: [{ vendorId: BMCV_VID, productId: BMCV_PID }],
      });
    }

    await device.open();
    if (!device.configuration) await device.selectConfiguration(1);
    await device.claimInterface(1);

    moduleDevice = device;
    ui.btnReboot.disabled = false;
    setStatus(ui.midiStatus, `connected to ${device.productName || 'BMCV'}`, 'good');
    log(`Module connected over USB${device.serialNumber ? `, serial ${device.serialNumber}` : ''}.`);
  } catch (err) {
    setStatus(ui.midiStatus, 'module not found', 'warn');
    log(err.name === 'NotFoundError' ? 'No module chosen.' : `${err.name}: ${err.message}`, 'warn');
    log('Use the FN2 route instead: hold FN2 while powering the case on.', 'warn');
  }
}

async function rebootIntoUpdateMode() {
  try {
    await usblink.enterDfuOn(moduleDevice);
  } catch (err) {
    // The module reboots as it is told, so the transfer that tells it often
    // fails on the way out. That is success, not failure.
    log(`(the module left mid-transfer, which is what being told to reboot looks like: ${err.name})`);
  }

  moduleDevice = null;
  ui.btnReboot.disabled = true;
  setStatus(ui.midiStatus, 'rebooted into update mode', 'good');
  log('Told the module to reboot into update mode. The panel should be amber.');
  log('It has left the USB bus as a BMCV and come back as a DFU device.');
}

// --- bootloader side, over WebUSB -----------------------------------------

// Shared by both image sources: a local file and a fetched release. Checked
// here rather than at the point of flashing, so a bad image can never get as
// far as erasing the module.
function loadImage(bytes, label) {
  const candidate = new Uint8Array(bytes);
  const problem = describeImageProblem(candidate);
  if (problem) {
    image = null;
    ui.btnFlash.disabled = true;
    setStatus(ui.fileStatus, problem, 'bad');
    log(`Refused ${label}: ${problem}`, 'bad');
    return;
  }

  image = candidate;
  setStatus(ui.fileStatus, `${label} — ${image.length.toLocaleString()} bytes`, 'good');
  ui.btnFlash.disabled = false;
  log(`Loaded ${label}, ${image.length} bytes.`);
}

function pickedFile(file) {
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => loadImage(reader.result, file.name);
  reader.onerror = () => setStatus(ui.fileStatus, 'could not read that file', 'bad');
  reader.readAsArrayBuffer(file);
}

// --- released versions, mirrored onto this same origin ---------------------
//
// Not fetched from the release itself: GitHub's release-asset CDN does not
// send Access-Control-Allow-Origin, so a browser fetch() of it is blocked
// regardless of which of GitHub's asset URL forms is used. scripts/fetch-
// releases.sh mirrors each release's .bin here at Pages-deploy time instead,
// which sidesteps that rather than working around it.

async function loadReleases() {
  try {
    const res = await fetch('../firmware/index.json');
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const releases = await res.json();

    ui.releaseSelect.innerHTML = '';
    if (releases.length === 0) {
      ui.releaseSelect.append(new Option('no released versions yet', ''));
      return;
    }

    ui.releaseSelect.append(new Option('choose a version…', ''));
    for (const r of releases) {
      const date = new Date(r.date).toLocaleDateString();
      ui.releaseSelect.append(new Option(`${r.tag} — ${date}, ${(r.size / 1024).toFixed(0)} KB`, r.path));
    }
    ui.releaseSelect.disabled = false;
  } catch (err) {
    ui.releaseSelect.innerHTML = '';
    ui.releaseSelect.append(new Option('could not load releases', ''));
    log(`Could not load the release list: ${err.message}. Pick a local file instead.`, 'warn');
  }
}

async function pickedRelease(path) {
  if (!path) return;
  ui.file.value = '';
  setStatus(ui.fileStatus, 'fetching…');
  try {
    const res = await fetch(`../firmware/${path}`);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    loadImage(await res.arrayBuffer(), path.split('/')[0]);
  } catch (err) {
    image = null;
    ui.btnFlash.disabled = true;
    setStatus(ui.fileStatus, 'could not fetch that release', 'bad');
    log(`Could not fetch ${path}: ${err.message}`, 'bad');
  }
}

async function flash() {
  ui.btnFlash.disabled = true;
  ui.progress.value = 0;

  let dfu = null;
  try {
    // Checked at pick time too. Kept here because this is the line after which
    // the module's flash is gone, and that is worth two redundant lines.
    const problem = describeImageProblem(image);
    if (problem) throw new DfuError(problem);

    setStatus(ui.flashStatus, 'waiting for you to pick the device…');
    dfu = await requestDfuDevice();
    log(`Connected to the bootloader, ${dfu.transferSize}-byte transfers.`);

    await dfu.toIdle();

    setStatus(ui.flashStatus, 'erasing…');
    await dfu.erase(FLASH_START, image.length, (p) => {
      ui.progress.value = p * 0.4;
    });
    log('Erase done.');

    setStatus(ui.flashStatus, 'writing…');
    await dfu.write(FLASH_START, image, (p) => {
      ui.progress.value = 0.4 + p * 0.6;
    });
    log('Write done.');

    await dfu.finish(FLASH_START);
    ui.progress.value = 1;

    setStatus(ui.flashStatus, 'done — now power-cycle the case', 'good');
    log('Flashed successfully.', 'good');
    log('Power the case off and on to start the new firmware.', 'warn');
  } catch (err) {
    // A user who closes the device picker is not an error worth shouting about.
    if (err.name === 'NotFoundError') {
      setStatus(ui.flashStatus, 'cancelled');
      log('No device picked.');
    } else {
      setStatus(ui.flashStatus, 'failed', 'bad');
      log(`Failed: ${err.message}`, 'bad');
      if (!(err instanceof DfuError)) {
        log('If Windows has not been set up with Zadig yet, that is the usual cause.', 'warn');
      }
      log('The module is still in update mode and can be flashed again.', 'warn');
    }
  } finally {
    await dfu?.close();
    ui.btnFlash.disabled = image === null;
  }
}

// --- wiring ----------------------------------------------------------------

function init() {
  if (!navigator.usb) {
    ui.unsupported.hidden = false;
    ui.mainFlow.hidden = true;
    return;
  }

  if (navigator.userAgent.includes('Windows')) {
    ui.windowsNote.hidden = false;
  }

  ui.btnConnect.addEventListener('click', connectModule);
  ui.btnReboot.addEventListener('click', rebootIntoUpdateMode);
  ui.file.addEventListener('change', (e) => {
    ui.releaseSelect.value = '';
    pickedFile(e.target.files[0]);
  });
  ui.releaseSelect.addEventListener('change', (e) => pickedRelease(e.target.value));
  ui.btnFlash.addEventListener('click', flash);

  loadReleases();
  log('Ready.');
}

init();
