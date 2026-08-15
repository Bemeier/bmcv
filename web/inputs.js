// The four input jacks: a voltage each, set by pointing at the trace of what the
// engine latched from it, plus a gate button and a clock generator per jack.
//
// The level has no control of its own. It had a fader - overlaid on the scope
// cell at first, then under it - and both arrangements meant reading a level
// off one strip and setting it on another, in units that lined up only because
// both had been told the same range. Pointing at the trace removes the question:
// the pointer is at 3V because 3V is where the trace would be.
//
// Everything here drives real voltages through sim.setCv(): a gate goes through
// the ADC threshold and the hysteresis exactly as a patch cable would, rather
// than being injected as a synthesised edge.

import { IN_ORDER } from './spec.js';
import { sim, N_IN, INPUT_MODE_NAMES } from './sim.js';
import { GATE_V, IN_V, PULSE_MS, TICK_US } from './const.js';
import { inputCellAt } from './scope.js';
import { mode } from './mode.js';

// InputMode.INPUT_CLOCK, which is what a generator is allowed to drive. Found
// by name in the firmware's own table rather than written as 1, so a mode
// inserted before it in config.h moves this with it.
const CLOCK_MODE = INPUT_MODE_NAMES.indexOf('CLOCK');

const controls = document.getElementById('in-controls');
const inCanvas = document.getElementById('inscope');

// What each jack sits at, which is also what a gate falls back to when it ends.
const inputLevel = new Array(N_IN).fill(0);

for (const i of IN_ORDER) {
  const cell = document.createElement('div');
  cell.className = 'in-cell';
  // Pulse first: every input has one, and only the jacks configured as clocks
  // have a generator - so the button is the column the eye can follow across
  // all four cells, and the generator is what appears beside it.
  cell.innerHTML = `
    <div class="in-ctl">
      <button type="button" data-pulse="${i}" title="send one ${GATE_V}V gate pulse">pulse</button>
      <span class="clock" data-clock="${i}">
        <input type="checkbox" data-clock-on="${i}" title="generate a clock on this input">
        <input type="number" data-clock-bpm="${i}" min="20" max="300" step="1" value="120" title="bpm">bpm
      </span>
    </div>`;
  controls.appendChild(cell);

  cell.querySelector('[data-pulse]').addEventListener('click', () => {
    sim.setCv(i, GATE_V);
    setTimeout(() => sim.setCv(i, inputLevel[i]), PULSE_MS * 2);
  });
}

function setLevel(i, volts) {
  inputLevel[i] = Math.max(-IN_V, Math.min(IN_V, volts));
  sim.setCv(i, inputLevel[i]);
}

/* ---- the trace is the fader ---------------------------------------------- */

// Drag anywhere in an input's cell to set that input's voltage, at the height
// you point at.
//
// There were four faders under the scope doing this, which meant reading a
// level off one strip and setting it on another, in units that only lined up
// because both were told the same range. Setting it where it is drawn removes
// the question entirely: the pointer is at 3V because 3V is where the trace
// would be.
//
// Refused under exactly the conditions the faders were refused under - a jack
// being driven by its own clock generator, and any jack at all while a physical
// module is driving the page, where the levels belong to whatever is patched
// into it.
function canSet(index) {
  return !mode.live && !gens.find(g => g.index === index)?.on.checked;
}

let dragging = null;

function pointAt(ev) {
  const hit = inputCellAt(ev.clientX, ev.clientY);
  return hit && canSet(hit.index) ? hit : null;
}

inCanvas.addEventListener('pointerdown', ev => {
  const hit = pointAt(ev);
  if (!hit) return;
  ev.preventDefault();
  inCanvas.setPointerCapture(ev.pointerId);

  // The cell is chosen once, on the way down. Dragging past a cell edge should
  // go on setting the input you grabbed rather than reaching into its
  // neighbour, which is the difference between a fader and a smear.
  dragging = hit.index;
  setLevel(hit.index, hit.volts);
});

inCanvas.addEventListener('pointermove', ev => {
  if (dragging === null) {
    // Only the cursor, so a jack that cannot be set does not invite a drag.
    inCanvas.style.cursor = pointAt(ev) ? 'ns-resize' : 'default';
    return;
  }
  const hit = inputCellAt(ev.clientX, ev.clientY);
  if (hit) setLevel(dragging, hit.volts);
});

const endDrag = () => { dragging = null; };
inCanvas.addEventListener('pointerup', endDrag);
inCanvas.addEventListener('pointercancel', endDrag);

// Back to zero, the same gesture the faders had.
inCanvas.addEventListener('dblclick', ev => {
  const hit = pointAt(ev);
  if (hit) setLevel(hit.index, 0);
});

/* ---- clock generators ---------------------------------------------------- */

// One per input, and shown only on the inputs actually configured as clocks.
//
// It used to belong to input 0, because that is the one that boots as
// INPUT_CLOCK. But which jacks are clocks is a thing the module decides and can
// be told to change, and a generator wired to a jack that is no longer a clock
// drives a modulation input with a square wave for no visible reason. So the
// control follows the configuration instead of assuming it.

const gens = IN_ORDER.map(i => ({
  index: i,
  box: controls.querySelector(`[data-clock="${i}"]`),
  on: controls.querySelector(`[data-clock-on="${i}"]`),
  bpm: controls.querySelector(`[data-clock-bpm="${i}"]`),
  accumUs: 0,
  high: false,
}));

function setClockLevel(g, high) {
  if (high === g.high) return;
  g.high = high;
  sim.setCv(g.index, high ? GATE_V : inputLevel[g.index]);
}

// Whatever the jack was left at becomes the gate's low level, so a jack sitting
// at -4V would produce a clock that never crosses the threshold. Zero it either
// way, on the way in and on the way out.
function syncGenerator(g) {
  setLevel(g.index, 0);
  if (!g.on.checked) setClockLevel(g, false);
}

for (const g of gens) {
  g.on.addEventListener('change', () => syncGenerator(g));
}

/* ---- who these belong to ------------------------------------------------- */

// Every control in the overlay, disabled outright while a module is driving the
// page.
//
// They were only dimmed, and dimmed is not disabled: the stylesheet turned the
// pointer off on the container, but the controls turn it back on for themselves
// - they sit over a canvas that takes drags, so they have to - and the second
// rule won. What that produced was a row of controls that looked unavailable,
// kept its hover cursor, took clicks and typing, and did nothing with any of
// it, because runTicks is not called and a page cannot put a voltage into a
// physical jack anyway.
//
// The attribute rather than more CSS, so the cursor, the click, the keyboard
// and what a screen reader says all come from one fact instead of four rules
// that have to agree.
const interactive = [...controls.querySelectorAll('button, input')];

mode.onChange(() => {
  for (const el of interactive) el.disabled = mode.live;

  // Said on the group, which is not itself disabled and so can still answer
  // the question a greyed-out control raises.
  controls.title = mode.live
    ? 'These drive the simulation. A module\'s inputs are physical: patch a cable, '
      + 'or send it a MIDI clock - see web/diagnostics.'
    : '';
});

// Show a generator only where the module says the jack is a clock. Called on
// the readout cadence, so a mode changed on the panel takes effect here without
// anything having to notice it happened.
export function drawInputModes() {
  for (const g of gens) {
    const isClock = sim.inputMode(g.index) === CLOCK_MODE;
    if (g.box.hidden === !isClock) continue;

    g.box.hidden = !isClock;

    // A jack that stops being a clock stops being driven like one, rather than
    // leaving a square wave running into whatever it became.
    if (!isClock && g.on.checked) {
      g.on.checked = false;
      syncGenerator(g);
    }
  }
}

export function runTicks(ticks) {
  if (ticks <= 0) return;

  const running = gens.filter(g => g.on.checked);
  if (!running.length) {
    sim.run(TICK_US, ticks);
    return;
  }

  // 4 pulses per beat, matching ClockState.PULSES_PER_BEAT.
  const shape = g => {
    const bpm = Math.min(300, Math.max(20, +g.bpm.value || 120));
    const periodUs = 60e6 / (bpm * 4);
    return { periodUs, widthUs: Math.min(PULSE_MS * 1000, periodUs * 0.4) };
  };

  let remaining = ticks;
  while (remaining > 0) {
    // Never step past an edge on any of them: a pulse that falls entirely
    // between two calls is skipped, and the tempo jitters. With more than one
    // generator running the step is whichever wants the next edge soonest.
    let step = remaining;
    for (const g of running) {
      const { periodUs, widthUs } = shape(g);
      const untilEdgeUs = g.high ? widthUs - g.accumUs : periodUs - g.accumUs;
      step = Math.min(step, Math.max(1, Math.ceil(untilEdgeUs / TICK_US)));
    }

    sim.run(TICK_US, step);
    remaining -= step;

    for (const g of running) {
      const { periodUs, widthUs } = shape(g);
      g.accumUs += step * TICK_US;
      if (g.accumUs >= periodUs) g.accumUs -= periodUs;
      setClockLevel(g, g.accumUs < widthUs);
    }
  }
}
