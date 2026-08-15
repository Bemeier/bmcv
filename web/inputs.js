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
import { CLOCK_INPUT, GATE_V, IN_V, PULSE_MS, TICK_US } from './const.js';

const controls = document.getElementById('in-controls');

// What the fader says each jack sits at, which is also what a gate falls back
// to when it ends.
const inputLevel = new Array(N_IN).fill(0);
const cells = [];

for (const i of IN_ORDER) {
  const cell = document.createElement('div');
  cell.className = 'in-cell';
  cell.innerHTML = `
    <div class="in-head"><span class="who">IN${i}</span><span class="mode" data-mode="${i}">—</span></div>
    <div class="hslider" title="IN${i} level"><div class="fill"></div><div class="zero"></div><div class="knob"></div></div>
    <div class="in-ctl"><button type="button" title="send one ${GATE_V}V gate pulse">pulse</button></div>`;
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

// What each jack is configured as, out of the module's own config rather than
// out of anything this file decides. An input in CLOCK mode behaves nothing
// like one in SLIDER, and until this was shown the only way to tell was to
// remember what the module had been told.
const modeEls = [...controls.querySelectorAll('[data-mode]')];

export function drawInputModes() {
  for (const el of modeEls) {
    const i = +el.dataset.mode;
    const mode = sim.inputMode(i);
    const name = INPUT_MODE_NAMES[mode] ?? '?';
    if (el.textContent !== name) el.textContent = name;
    el.classList.toggle('off', mode === 0);
  }
}

/* ---- clock generator ---------------------------------------------------- */

// It belongs to input 0, which is the one that boots as INPUT_CLOCK, so it
// sits on that cell rather than in a control panel somewhere else.
const clockCell = cells.find(c => c.index === CLOCK_INPUT);
clockCell.fader.parentElement.querySelector('.in-ctl').insertAdjacentHTML('beforeend',
  `<span class="clock"><input id="clock-on" type="checkbox" title="generate a clock on this input">
   <input id="clock-bpm" type="number" min="20" max="300" step="1" value="120" title="bpm">bpm</span>`);

const clockOn = document.getElementById('clock-on');
const clockBpm = document.getElementById('clock-bpm');

// While the clock is generating, the fader would only fight it - and whatever
// it was left at becomes the gate's low level, so a jack sitting at -4V would
// produce a clock that never crosses the threshold. Zero it either way.
function syncClockUi() {
  clockCell.fader.dataset.disabled = clockOn.checked ? '1' : '0';
  clockCell.setLevel(0);
}
clockOn.addEventListener('change', syncClockUi);
syncClockUi();

let clockAccumUs = 0;
let clockHigh = false;

function setClockLevel(high) {
  if (high === clockHigh) return;
  clockHigh = high;
  sim.setCv(CLOCK_INPUT, high ? GATE_V : inputLevel[CLOCK_INPUT]);
}

// Advance the engine by `ticks`, generating a gate train on the clock input if
// the generator is on.
//
// The stepping matters: the engine has to be run in pieces short enough that no
// edge falls entirely between two calls, or a pulse is skipped and the tempo
// jitters. Owning both the gate and the stepping here is what keeps that
// correct - the main loop just says how much time passed.
export function runTicks(ticks) {
  if (ticks <= 0) return;

  if (!clockOn.checked) {
    setClockLevel(false);
    sim.run(TICK_US, ticks);
    return;
  }

  // 4 pulses per beat, matching ClockState.PULSES_PER_BEAT.
  const bpm = Math.min(300, Math.max(20, +clockBpm.value || 120));
  const periodUs = 60e6 / (bpm * 4);
  const widthUs = Math.min(PULSE_MS * 1000, periodUs * 0.4);

  let remaining = ticks;
  while (remaining > 0) {
    // Never step past an edge.
    const untilEdgeUs = clockHigh ? widthUs - clockAccumUs : periodUs - clockAccumUs;
    const step = Math.min(remaining, Math.max(1, Math.ceil(untilEdgeUs / TICK_US)));

    sim.run(TICK_US, step);
    remaining -= step;
    clockAccumUs += step * TICK_US;
    if (clockAccumUs >= periodUs) clockAccumUs -= periodUs;

    setClockLevel(clockAccumUs < widthUs);
  }
}
