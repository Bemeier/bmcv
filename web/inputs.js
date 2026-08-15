// The four input jacks: a level fader and a pulse button each, under the trace
// of what the engine actually latched from them, plus the clock generator on
// input 0 and what each jack is configured as.
//
// The controls used to be overlaid on the scope cells. That cost the trace the
// space they took and put a fader across the signal it was setting; under it,
// each column is one input from configuration down to control.
//
// Everything here drives real voltages through sim.setCv(): a gate goes through
// the ADC threshold and the hysteresis exactly as a patch cable would, rather
// than being injected as a synthesised edge.

import { IN_ORDER } from './spec.js';
import { sim, N_IN, INPUT_MODE_NAMES } from './sim.js';

// InputMode.INPUT_CLOCK, which is what a generator is allowed to drive.
const CLOCK_MODE = INPUT_MODE_NAMES.indexOf('CLOCK');
import { GATE_V, IN_V, PULSE_MS, TICK_US } from './const.js';

const controls = document.getElementById('in-controls');

// What the fader says each jack sits at, which is also what a gate falls back
// to when it ends.
const inputLevel = new Array(N_IN).fill(0);
const cells = [];

for (const i of IN_ORDER) {
  const cell = document.createElement('div');
  cell.className = 'in-cell';
  cell.innerHTML = `
    <div class="hslider" title="IN${i} level"><div class="fill"></div><div class="zero"></div><div class="knob"></div></div>
    <div class="in-ctl">
      <span class="clock" data-clock="${i}" hidden>
        <input type="checkbox" data-clock-on="${i}" title="generate a clock on this input">
        <input type="number" data-clock-bpm="${i}" min="20" max="300" step="1" value="120" title="bpm">bpm
      </span>
      <button type="button" title="send one ${GATE_V}V gate pulse">pulse</button>
    </div>`;
  controls.appendChild(cell);

  const fader = cell.querySelector('.hslider');
  const fill = cell.querySelector('.fill');
  const knob = cell.querySelector('.knob');

  // Fill from the centre outwards, so the bar means "offset from 0V". Left is
  // -10V and right is +10V, which is the way the trace above it reads too.
  const paint = v => {
    const pct = ((v + IN_V) / (2 * IN_V)) * 100;
    knob.style.left = `${pct}%`;
    fill.style.left = `${Math.min(pct, 50)}%`;
    fill.style.width = `${Math.abs(pct - 50)}%`;
    fader.title = `IN${i} ${v.toFixed(1)} V`;
  };

  const setLevel = v => {
    inputLevel[i] = Math.max(-IN_V, Math.min(IN_V, v));
    sim.setCv(i, inputLevel[i]);
    paint(inputLevel[i]);
  };

  let dragging = false;
  const fromEvent = ev => {
    const r = fader.getBoundingClientRect();
    const frac = Math.min(1, Math.max(0, (ev.clientX - r.left) / r.width));
    setLevel(frac * 2 * IN_V - IN_V);
  };
  fader.addEventListener('pointerdown', ev => {
    ev.preventDefault(); fader.setPointerCapture(ev.pointerId); dragging = true; fromEvent(ev);
  });
  fader.addEventListener('pointermove', ev => { if (dragging) fromEvent(ev); });
  fader.addEventListener('pointerup', () => { dragging = false; });
  fader.addEventListener('pointercancel', () => { dragging = false; });
  fader.addEventListener('dblclick', () => setLevel(0));

  cell.querySelector('button').addEventListener('click', () => {
    sim.setCv(i, GATE_V);
    setTimeout(() => sim.setCv(i, inputLevel[i]), PULSE_MS * 2);
  });

  paint(0);
  cells.push({ index: i, fader, setLevel });
}

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
  cell: cells.find(c => c.index === i),
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

// While a generator is running the fader would only fight it - and whatever it
// was left at becomes the gate's low level, so a jack sitting at -4V produces a
// clock that never crosses the threshold. Zero it either way.
function syncGenerator(g) {
  g.cell.fader.dataset.disabled = g.on.checked ? '1' : '0';
  g.cell.setLevel(0);
  if (!g.on.checked) setClockLevel(g, false);
}

for (const g of gens) {
  g.on.addEventListener('change', () => syncGenerator(g));
}

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
