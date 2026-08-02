// The numbers: mode, parameter, scene, tempo, and the per-channel table.
//
// The table shows what each channel is *actually* doing after the scene
// crossfade and the parameter maths - EngineState.channels_effective[] - rather
// than what was dialled into the active scene, which is a different number
// whenever the crossfader is anywhere but an end stop.

import { PARAM_NAMES } from './spec.js';
import { sim, EFF, N_CH, SHIFT_NAMES } from './sim.js';

// The frequency parameter is a ratio against the beat rate, dialled in as
// musical fractions, so show it that way rather than as 0.333.
function ratioText(r) {
  // The parameter is stored in 1/255ths, so a ratio meant as 5/4 arrives as
  // 1.2510. Match loosely enough to see through that quantisation.
  const NEAR = 0.02;
  if (!isFinite(r) || r === 0) return '0';
  if (Math.abs(r - Math.round(r)) < NEAR) return `${Math.round(r)}`;
  const inv = 1 / r;
  if (Math.abs(inv - Math.round(inv)) < NEAR) return `1/${Math.round(inv)}`;
  for (const d of [2, 3, 4, 5, 6, 8, 12, 16]) {
    const n = r * d;
    if (Math.abs(n - Math.round(n)) < NEAR) return `${Math.round(n)}/${d}`;
  }
  return r.toFixed(2);
}

const EFF_COLS = [
  ['freq', c => c[EFF.FREQ_HZ] >= 10 ? c[EFF.FREQ_HZ].toFixed(1) : c[EFF.FREQ_HZ].toFixed(3)],
  ['ratio', c => ratioText(c[EFF.FREQ_RATIO])],
  ['shape', c => c[EFF.SHAPE].toFixed(2)],
  ['mod', c => c[EFF.MOD].toFixed(2)],
  ['phase', c => c[EFF.PHASE].toFixed(3)],
  ['amp', c => c[EFF.AMP_V].toFixed(2)],
  ['ofs', c => c[EFF.OFFSET_V].toFixed(2)],
];

const paramsTable = document.getElementById('params');
paramsTable.innerHTML =
  `<tr><th>ch</th>${EFF_COLS.map(([h]) => `<th>${h}</th>`).join('')}<th>out</th></tr>` +
  Array.from({ length: N_CH }, (_, c) =>
    `<tr data-ch="${c}"><td>${c}</td>${EFF_COLS.map(() => '<td>0</td>').join('')}<td>0.00</td></tr>`).join('');
const paramRows = [...paramsTable.querySelectorAll('tr[data-ch]')];

const rShift = document.getElementById('r-shift');
const rParam = document.getElementById('r-param');
const rScene = document.getElementById('r-scene');
const rBpm = document.getElementById('r-bpm');

// Only touch the DOM when the text actually differs: this table is the single
// largest source of DOM churn on the page.
const setText = (node, value) => { if (node.textContent !== value) node.textContent = value; };

export function drawReadouts() {
  setText(rShift, SHIFT_NAMES[sim.shiftState()] ?? '—');
  setText(rParam, PARAM_NAMES[sim.selectedParam()] ?? '—');
  setText(rScene, String(sim.activeScene()));
  setText(rBpm, sim.bpm().toFixed(1));

  const outs = sim.outputs();
  const eff = sim.effective();
  for (let c = 0; c < N_CH; c++) {
    const cells = paramRows[c].children;
    const row = eff.subarray(c * EFF.COUNT, (c + 1) * EFF.COUNT);
    for (let i = 0; i < EFF_COLS.length; i++) setText(cells[i + 1], EFF_COLS[i][1](row));
    setText(cells[EFF_COLS.length + 1], outs[c].toFixed(2));
    paramRows[c].classList.toggle('muted-row', !!sim.muted(c));
  }
}

const statusEl = document.getElementById('status');

export function setStatus(text) {
  statusEl.textContent = text;
  setTimeout(() => { if (statusEl.textContent === text) statusEl.textContent = ''; }, 2000);
}
