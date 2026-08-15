// BMCV web simulator: wiring and the frame loop.
//
// Everything of substance is in a module of its own:
//
//   sim.js        the wasm module behind readable names
//   probe/        a physical module, over WebUSB, into that same wasm module
//   spec.js       the generated panel spec and what is derived from it
//   panel.js      the SVG panel and its pointer handling
//   leds.js       the 21 WS2812s, driven from the real framebuffer
//   scope.js      the output and input scopes
//   inputs.js     the input faders, gate buttons and clock generator
//   readouts.js   the mode/scene/tempo readouts and the channel table
//   storage.js    mirroring the preset store into localStorage
//   midi.js       the module's MIDI output, onto a Web MIDI port
//   const.js      the handful of numbers more than one of them needs
//
// The panel is built entirely from panel.json, which is generated from the
// hardware's KiCad placement data plus the firmware's own index tables - so
// nothing here knows where anything is, and the picture cannot drift from the
// board.

import { MAX_CATCHUP_TICKS, TICK_US } from './const.js';
import { sim } from './sim.js';
import { activeSession } from './probe/session.js';
import { drawEncoderIndicators, drawSliderFromModule, setSliderFromPos, SLIDER_START_POS } from './panel.js';
import { drawLeds } from './leds.js';
import { drawScopes } from './scope.js';
import { drawInputModes, runTicks } from './inputs.js';
import { drawReadouts } from './readouts.js';
import { forget, persist, restore } from './storage.js';
import { drawMidi, initMidi, pumpMidi } from './midi.js';
import { mode, SIM } from './mode.js';
import { drawProbeRates, initProbe } from './probe/ui.js';

/* ---- startup ------------------------------------------------------------ */

// Restoring reboots the module, so it has to happen before the drawn slider
// position is published - a reboot puts slider_raw back at one end, and the
// panel would then disagree with the engine about where the handle is.
restore();
setSliderFromPos(SLIDER_START_POS);

const resetButton = document.getElementById('reset');
const resetFramButton = document.getElementById('reset-fram');

// Both act on whatever module the page is showing.
//
// In the simulation that is the wasm instance in this tab. With a module
// connected it is the module, over whichever link is carrying it - the command
// goes into the same mailbox either way and the module performs it itself, so
// nothing here needs to know which transport is in use.
//
// They used to be disabled whenever a module was driving the page, because
// resetting the simulation nobody was looking at would have appeared to do
// nothing. Now they do the thing the label promises in all three modes, and the
// label says which module is about to be affected.
function askModule(wipeStorage) {
  if (mode.live) {
    sim.remoteReset(wipeStorage);
    activeSession()?.sendCommand();
    return;
  }

  sim.reset(wipeStorage);
  if (wipeStorage) forget();
  setSliderFromPos(SLIDER_START_POS);
}

// No confirmation message. What either of these did is visible immediately in
// everything the page is already showing - the panel, the scopes, the channel
// table - so a line of text saying it happened was one more thing to place and
// one more thing to time out, restating what the page had already said.
resetButton.addEventListener('click', () => askModule(false));
resetFramButton.addEventListener('click', () => askModule(true));

// What the buttons are about to affect, said on the buttons themselves. The
// difference between wiping a simulation and wiping a module's FRAM is worth
// more than a tooltip.
mode.onChange(source => {
  const target = source === SIM ? 'the simulation' : 'the connected module';
  resetButton.title = `Restart ${target}, keeping stored presets`;
  resetFramButton.title = `Restart ${target} and forget every preset it has stored`;
});

// Not awaited: it ends in a permission prompt on some browsers and is absent on
// others, and neither should hold up the module coming to life.
initMidi();

initProbe();

/* ---- frame loop --------------------------------------------------------- */

const READOUT_INTERVAL_MS = 100;   // numbers a person reads do not need 60Hz
const PERSIST_INTERVAL_MS = 2000;  // the same cadence as the firmware's autosave

let lastT = performance.now();
let lastReadoutT = 0;
let lastPersistT = 0;
let carryUs = 0;

function frame(now) {
  let elapsedUs = (now - lastT) * 1000 + carryUs;
  lastT = now;

  // rAF hands the callback the *current frame's* start time, which can be
  // slightly earlier than a performance.now() sampled just before scheduling
  // it. That makes elapsed negative on the first frame often enough to matter,
  // and a negative tick count used to reach the engine as a colossal unsigned
  // one and freeze the tab for good.
  if (elapsedUs < 0) elapsedUs = 0;

  let ticks = Math.floor(elapsedUs / TICK_US);
  carryUs = elapsedUs - ticks * TICK_US;
  if (ticks > MAX_CATCHUP_TICKS) { ticks = MAX_CATCHUP_TICKS; carryUs = 0; }

  // A physical module is already running; there is nothing here to advance.
  // Its state arrives from the probe's own timer and is written straight into
  // the wasm instance, so everything below draws hardware without knowing it.
  if (!mode.live) {
    runTicks(ticks);

    // Straight after the ticks that produced the messages, and before the
    // drawing - a frame's worth of canvas work between the two would be latency
    // added to every control change for no reason.
    //
    // Skipped when a module is driving: the queue in the snapshot is the
    // hardware's own, and the hardware is already sending it over its own USB
    // port. Draining it here would put every control change on the bus twice.
    pumpMidi();
  }

  drawEncoderIndicators();
  drawLeds();
  drawScopes();
  if (mode.live) drawSliderFromModule();

  if (now - lastReadoutT > READOUT_INTERVAL_MS) {
    lastReadoutT = now;
    drawReadouts();
    drawInputModes();
    drawMidi();
    drawProbeRates();
  }

  // Not while a module is driving: what is in the wasm instance then is a copy
  // of the hardware's patch, and mirroring it into browser storage would
  // quietly overwrite the simulator's own with it.
  if (!mode.live && now - lastPersistT > PERSIST_INTERVAL_MS) { lastPersistT = now; persist(); }

  // The page starts dimmed and stops being so once there is something real on
  // it. Not on load: the wasm has to be up, the panel built and one frame drawn
  // before any of these numbers means anything, and undimming before that shows
  // a complete-looking page full of dashes.
  if (document.body.classList.contains('loading')) {
    document.body.classList.remove('loading');
  }

  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
