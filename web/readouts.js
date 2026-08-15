// The numbers: mode, parameter, scene, tempo, and the per-channel table.
//
// The table shows what each channel is *actually* doing after the scene
// crossfade and the parameter maths - EngineState.channels_effective[] - rather
// than what was dialled into the active scene, which is a different number
// whenever the crossfader is anywhere but an end stop.

import { PARAM_NAMES } from './spec.js';
import { sim, EFF, N_CH, N_IN, SHAPE_NAMES, SHIFT_NAMES, QUANTIZE_MODE_NAMES } from './sim.js';

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

// The live phase is already on the scope, moving; what is not visible anywhere
// else is the phase *offset* that was dialled in, so that is the column.
const EFF_COLS = [
  ['mode', (_c, ch) => SHAPE_NAMES[sim.shapeMode(ch)] ?? '—'],
  // No absolute frequency column: the ratio is what was dialled in and what the
  // channel is locked to, and the Hz it works out to is the tempo times that.
  ['ratio', c => ratioText(c[EFF.FREQ_RATIO])],
  ['shape', c => c[EFF.SHAPE].toFixed(2)],
  ['mod', c => c[EFF.MOD].toFixed(2)],
  ['phs', c => c[EFF.PHASE_OFS].toFixed(3)],
  ['amp', c => c[EFF.AMP_V].toFixed(2)],
  ['ofs', c => c[EFF.OFFSET_V].toFixed(2)],

  // Whether the channel snaps to the scale, and what makes it do so. "trig"
  // without a source is a channel waiting on something nothing is patched to,
  // which looks identical to a broken quantizer until the source is shown
  // beside it.
  ['qnt', (_c, ch) => {
    const mode = sim.channelQuantizeMode(ch);
    const name = QUANTIZE_MODE_NAMES[mode] ?? '?';
    if (name !== 'trig') return name;
    return `trig ${trigName(sim.channelTrigSrc(ch))}`;
  }],
];

// A trigger source, in the composite space HwState.trigger_src uses: the input
// jacks first, then the channel outputs. -1 is nothing patched.
function trigName(src) {
  if (src < 0) return '—';
  return src < N_IN ? `IN${src}` : `CH${src - N_IN}`;
}

const paramsTable = document.getElementById('params');
paramsTable.innerHTML =
  `<tr><th>ch</th>${EFF_COLS.map(([h]) => `<th>${h}</th>`).join('')}<th>out</th></tr>` +
  Array.from({ length: N_CH }, (_, c) =>
    `<tr data-ch="${c}"><td>${c}</td>${EFF_COLS.map(() => '<td>0</td>').join('')}<td>0.00</td></tr>`).join('');
const paramRows = [...paramsTable.querySelectorAll('tr[data-ch]')];

/* ---- the quantizer's scale ----------------------------------------------- */

// Twelve bits, drawn as one octave of a keyboard.
//
// A channel set to quantize looked the same on this page whether it was
// snapping to a major scale or to all twelve semitones, which is most of what
// you want to know when a sequence comes out wrong. Twelve labelled boxes would
// have said it too; a keyboard says it without being read.
//
// Semitone 0 is C, which is what the firmware's mask counts from - see
// quantizer.c.
const NATURALS = [0, 2, 4, 5, 7, 9, 11];        // C D E F G A B
const ACCIDENTALS = [[1, 1], [3, 2], [6, 4], [8, 5], [10, 6]]; // semitone, gap after nth natural

const keysEl = document.getElementById('keys');
keysEl.innerHTML =
  NATURALS.map(n => `<div class="nat" data-semi="${n}"></div>`).join('') +
  ACCIDENTALS.map(([semi, after]) => `<div class="acc" data-semi="${semi}" style="left:${after * 18}px"></div>`).join('');
const keyEls = [...keysEl.querySelectorAll('[data-semi]')];

function drawScale() {
  const mask = sim.quantizeMask();
  for (const el of keyEls) {
    el.classList.toggle('on', (mask & (1 << +el.dataset.semi)) !== 0);
  }
}

const rShift = document.getElementById('r-shift');
const rParam = document.getElementById('r-param');
const rScene = document.getElementById('r-scene');
const rBpm = document.getElementById('r-bpm');
const rBpmIn = document.getElementById('r-bpm-in');

// Only touch the DOM when the text actually differs: this table is the single
// largest source of DOM churn on the page.
const setText = (node, value) => { if (node.textContent !== value) node.textContent = value; };

export function drawReadouts() {
  drawScale();
  setText(rShift, SHIFT_NAMES[sim.shiftState()] ?? '—');
  setText(rParam, PARAM_NAMES[sim.selectedParam()] ?? '—');
  setText(rScene, String(sim.activeScene()));
  // Detected is what the clock jack is doing right now and reads as nothing
  // when there is no clock; running is what the oscillators are locked to,
  // which free-runs at the last tempo after the pulses stop.
  setText(rBpmIn, sim.haveBeat() ? sim.bpm().toFixed(1) : '—');
  setText(rBpm, sim.activeBpm().toFixed(1));

  const outs = sim.outputs();
  const eff = sim.effective();
  for (let c = 0; c < N_CH; c++) {
    const cells = paramRows[c].children;
    const row = eff.subarray(c * EFF.COUNT, (c + 1) * EFF.COUNT);
    for (let i = 0; i < EFF_COLS.length; i++) setText(cells[i + 1], EFF_COLS[i][1](row, c));
    setText(cells[EFF_COLS.length + 1], outs[c].toFixed(2));
    paramRows[c].classList.toggle('muted-row', !!sim.muted(c));
  }
}

const statusEl = document.getElementById('status');

export function setStatus(text) {
  statusEl.textContent = text;
  setTimeout(() => { if (statusEl.textContent === text) statusEl.textContent = ''; }, 2000);
}
