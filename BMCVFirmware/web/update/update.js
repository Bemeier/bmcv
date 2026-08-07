// Page logic for the firmware updater: talk to the running module over
// WebMIDI to get it into DFU, then talk to ST's ROM bootloader over WebUSB to
// write the image.
//
// The two halves are independent on purpose. If the firmware on the module is
// too broken to answer MIDI, the user holds FN2 at power-on and the WebUSB half
// still works on its own.

import { requestDfuDevice, describeImageProblem, FLASH_START, DfuError } from './dfuse.js';

// F0 7D 42 4D <cmd> F7 - see Core/Inc/Lib/sysex.h. 0x7D is the non-commercial
// manufacturer ID; 'B','M' after it is what keeps this from colliding with
// every other project using the same ID.
const SYSEX_ENTER_UPDATE = [0xf0, 0x7d, 0x42, 0x4d, 0x01, 0xf7];
const SYSEX_IDENTITY_REQ = [0xf0, 0x7d, 0x42, 0x4d, 0x02, 0xf7];

const el = (id) => document.getElementById(id);

const ui = {
  unsupported: el('unsupported'),
  mainFlow: el('main-flow'),
  windowsNote: el('windows-note'),
  btnConnect: el('btn-connect'),
  btnReboot: el('btn-reboot'),
  midiStatus: el('midi-status'),
  file: el('file'),
  fileStatus: el('file-status'),
  btnFlash: el('btn-flash'),
  flashStatus: el('flash-status'),
  progress: el('progress'),
  log: el('log'),
};

let image = null;
let midiOutput = null;

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

function isModule(port) {
  return /bmcv/i.test(port.name ?? '');
}

// Ask the module what version it is running, and give up quickly if nothing
// answers. The reply is nine SysEx bytes; we only care about the last three
// before the terminator.
function requestVersion(input, output) {
  return new Promise((resolve) => {
    const timer = setTimeout(() => {
      input.onmidimessage = null;
      resolve(null);
    }, 500);

    input.onmidimessage = (event) => {
      const d = event.data;
      if (d.length < 9 || d[0] !== 0xf0 || d[1] !== 0x7d || d[2] !== 0x42 || d[3] !== 0x4d || d[4] !== 0x02) {
        return;
      }
      clearTimeout(timer);
      input.onmidimessage = null;
      resolve(`${d[5]}.${d[6]}.${d[7]}`);
    };

    output.send(SYSEX_IDENTITY_REQ);
  });
}

async function connectMidi() {
  if (!navigator.requestMIDIAccess) {
    setStatus(ui.midiStatus, 'no WebMIDI in this browser', 'warn');
    log('This browser has no WebMIDI. Use the FN2 route instead.', 'warn');
    return;
  }

  let access;
  try {
    access = await navigator.requestMIDIAccess({ sysex: true });
  } catch (err) {
    setStatus(ui.midiStatus, 'permission denied', 'bad');
    log(`MIDI access refused: ${err.message}`, 'bad');
    return;
  }

  const output = [...access.outputs.values()].find(isModule);
  const input = [...access.inputs.values()].find(isModule);

  if (!output) {
    setStatus(ui.midiStatus, 'module not found', 'warn');
    log('No BMCV on the MIDI bus. Check the cable and that the case is powered, or use the FN2 route.', 'warn');
    return;
  }

  midiOutput = output;
  ui.btnReboot.disabled = false;

  const version = input ? await requestVersion(input, output) : null;
  if (version) {
    setStatus(ui.midiStatus, `connected, running ${version}`, 'good');
    log(`Module connected, firmware ${version}.`);
  } else {
    setStatus(ui.midiStatus, 'connected', 'good');
    log('Module connected. It did not report a version, which older firmware will not.');
  }
}

function rebootIntoUpdateMode() {
  midiOutput.send(SYSEX_ENTER_UPDATE);
  ui.btnReboot.disabled = true;
  setStatus(ui.midiStatus, 'rebooted into update mode', 'good');
  log('Told the module to reboot into update mode. The panel should be amber.');
  log('It has left the MIDI bus and is now a DFU device.');
}

// --- bootloader side, over WebUSB -----------------------------------------

function pickedFile(file) {
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    const candidate = new Uint8Array(reader.result);

    // Checked here rather than at the point of flashing, so a bad file can
    // never get as far as erasing the module.
    const problem = describeImageProblem(candidate);
    if (problem) {
      image = null;
      ui.btnFlash.disabled = true;
      setStatus(ui.fileStatus, problem, 'bad');
      log(`Refused ${file.name}: ${problem}`, 'bad');
      return;
    }

    image = candidate;
    setStatus(ui.fileStatus, `${file.name} — ${image.length.toLocaleString()} bytes`, 'good');
    ui.btnFlash.disabled = false;
    log(`Loaded ${file.name}, ${image.length} bytes.`);
  };
  reader.onerror = () => {
    setStatus(ui.fileStatus, 'could not read that file', 'bad');
  };
  reader.readAsArrayBuffer(file);
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

  ui.btnConnect.addEventListener('click', connectMidi);
  ui.btnReboot.addEventListener('click', rebootIntoUpdateMode);
  ui.file.addEventListener('change', (e) => pickedFile(e.target.files[0]));
  ui.btnFlash.addEventListener('click', flash);

  log('Ready.');
}

init();
