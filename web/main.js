// BMCV web simulator: wiring and the frame loop.
//
// Everything of substance is in a module of its own:
//
//   sim.js        the wasm module behind readable names
//   spec.js       the generated panel spec and what is derived from it
//   panel.js      the SVG panel and its pointer handling
//   leds.js       the 21 WS2812s, driven from the real framebuffer
//   scope.js      the output and input scopes
//   inputs.js     the input faders, gate buttons and clock generator
//   readouts.js   the mode/scene/tempo readouts and the channel table
//   storage.js    mirroring the preset store into localStorage
//   const.js      the handful of numbers more than one of them needs
//
// The panel is built entirely from panel.json, which is generated from the
// hardware's KiCad placement data plus the firmware's own index tables - so
// nothing here knows where anything is, and the picture cannot drift from the
// board.

import { MAX_CATCHUP_TICKS, TICK_US } from './const.js';
import { sim } from './sim.js';
import { drawEncoderIndicators, setSliderFromPos, SLIDER_START_POS } from './panel.js';
import { drawLeds } from './leds.js';
import { drawScopes } from './scope.js';
import { runTicks } from './inputs.js';
import { drawReadouts, setStatus } from './readouts.js';
import { forget, persist, restore } from './storage.js';

/* ---- startup ------------------------------------------------------------ */

// Restoring reboots the module, so it has to happen before the drawn slider
// position is published - a reboot puts slider_raw back at one end, and the
// panel would then disagree with the engine about where the handle is.
if (restore()) setStatus('restored saved state');
setSliderFromPos(SLIDER_START_POS);

document.getElementById('reset').addEventListener('click', () => {
  sim.reset(false);          // reboot, keeping the stored presets
  setSliderFromPos(SLIDER_START_POS);
  setStatus('module reset');
});

document.getElementById('reset-fram').addEventListener('click', () => {
  sim.reset(true);           // wipe the module's own preset slots too
  forget();
  setSliderFromPos(SLIDER_START_POS);
  setStatus('FRAM cleared');
});

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

  runTicks(ticks);

  drawEncoderIndicators();
  drawLeds();
  drawScopes();

  if (now - lastReadoutT > READOUT_INTERVAL_MS) { lastReadoutT = now; drawReadouts(); }
  if (now - lastPersistT > PERSIST_INTERVAL_MS) { lastPersistT = now; persist(); }

  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
