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
import { drawEncoderIndicators, drawSliderFromModule, setSliderFromPos, SLIDER_START_POS } from './panel.js';
import { drawLeds } from './leds.js';
import { drawScopes } from './scope.js';
import { runTicks } from './inputs.js';
import { drawReadouts, setStatus } from './readouts.js';
import { forget, persist, restore } from './storage.js';
import { drawMidi, initMidi, pumpMidi } from './midi.js';
import { mode } from './mode.js';
import { drawProbeRates, initProbe } from './probe/ui.js';

/* ---- startup ------------------------------------------------------------ */

// Restoring reboots the module, so it has to happen before the drawn slider
// position is published - a reboot puts slider_raw back at one end, and the
// panel would then disagree with the engine about where the handle is.
if (restore()) setStatus('restored saved state');
setSliderFromPos(SLIDER_START_POS);

const resetButton = document.getElementById('reset');
const resetFramButton = document.getElementById('reset-fram');

resetButton.addEventListener('click', () => {
  sim.reset(false);          // reboot, keeping the stored presets
  setSliderFromPos(SLIDER_START_POS);
  setStatus('module reset');
});

resetFramButton.addEventListener('click', () => {
  sim.reset(true);           // wipe the module's own preset slots too
  forget();
  setSliderFromPos(SLIDER_START_POS);
  setStatus('FRAM cleared');
});

// Both of these reset the simulator, which is not what the page is showing
// while a module is driving it: the next snapshot lands a few milliseconds
// later and overwrites the reset, and disconnecting restores the simulation
// that was running before the probe was connected. So they would appear to do
// nothing, which is worse than being unavailable.
mode.onChange(live => {
  resetButton.disabled = live;
  resetFramButton.disabled = live;
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
    drawMidi();
    drawProbeRates();
  }

  // Not while a module is driving: what is in the wasm instance then is a copy
  // of the hardware's patch, and mirroring it into browser storage would
  // quietly overwrite the simulator's own with it.
  if (!mode.live && now - lastPersistT > PERSIST_INTERVAL_MS) { lastPersistT = now; persist(); }

  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
