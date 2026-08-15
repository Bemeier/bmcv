// The numbers: mode, parameter, scene, tempo, and the per-channel table.
//
// The table shows what each channel is *actually* doing after the scene
// crossfade and the parameter maths - EngineState.channels_effective[] - rather
// than what was dialled into the active scene, which is a different number
// whenever the crossfader is anywhere but an end stop.

import { PARAM_NAMES } from './spec.js';
import { sim, EFF, N_CH, N_IN, SHAPE_NAMES, SHIFT_NAMES, QUANTIZE_MODE_NAMES, AMP_MODE_NAMES } from './sim.js';

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
  ['phs', c => c[EFF.PHASE_OFS].toFixed(2)],
  ['amp', c => c[EFF.AMP_V].toFixed(2)],
  ['ofs', c => c[EFF.OFFSET_V].toFixed(2)],

  // Whether the channel snaps to the scale, and what makes it do so. "trig"
  // without a source is a channel waiting on something nothing is patched to,
  // which looks identical to a broken quantizer until the source is shown
  // beside it.
  // Which input the channel folds in, and how. A source with the mixing off is
  // a patch someone set up and then disabled, which is worth being able to see
  // rather than having it vanish from the table.
  ['src', (_c, ch) => {
    const mode = AMP_MODE_NAMES[sim.channelAmpMode(ch)] ?? '?';
    const src = sim.channelSrcInput(ch);
    if (src < 0) return '—';
    return `IN${src} ${mode === 'add' ? '+' : mode === 'mult' ? '\u00d7' : 'off'}`;
  }],

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
  `<tr><th class="c-ch">ch</th>${EFF_COLS.map(([h]) => `<th class="c-${h}">${h}</th>`).join('')}<th>out</th></tr>` +
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
// SVG rather than stacked divs, and the white keys are notched rather than
// plain rectangles. With a rectangle under it, a translucent black key shows
// the white one through - which at these opacities made an unlit sharp look
// like a lit one. Cutting the white key around its neighbours means no two
// shapes ever overlap, so what is drawn is what is seen.
//
// Semitone 0 is C, which is what the firmware's mask counts from - see
// quantizer.c.
const WHITE_SEMIS = [0, 2, 4, 5, 7, 9, 11];  // C D E F G A B
const BLACK_SEMIS = [1, 3, 6, 8, 10];        // the five sharps

// One white key wide, in the units the viewBox is drawn in, and how far down a
// black key reaches.
const KEY_H = 4.4;
const BLACK_W = 0.62;
const BLACK_H = KEY_H * 0.6;

// Which white key each sharp sits after: C#, D#, then F#, G#, A#.
const BLACK_AFTER = [0, 1, 3, 4, 5];

// A white key, cut around whichever sharps overlap it.
function whitePath(w) {
  const x0 = w, x1 = w + 1;
  const half = BLACK_W / 2;
  const lb = BLACK_AFTER.includes(w - 1); // a sharp ends on this key's left
  const rb = BLACK_AFTER.includes(w);     // and one starts on its right

  const d = [`M ${x0 + (lb ? half : 0)} 0`];
  d.push(`L ${x1 - (rb ? half : 0)} 0`);
  if (rb) d.push(`L ${x1 - half} ${BLACK_H}`, `L ${x1} ${BLACK_H}`);
  d.push(`L ${x1} ${KEY_H}`, `L ${x0} ${KEY_H}`);
  if (lb) d.push(`L ${x0} ${BLACK_H}`, `L ${x0 + half} ${BLACK_H}`);
  return `${d.join(' ')} Z`;
}

const keysEl = document.getElementById('keys');
keysEl.innerHTML =
  `<svg viewBox="0 0 7 ${KEY_H}" preserveAspectRatio="none">`
  + WHITE_SEMIS.map((semi, w) => `<path class="nat" data-semi="${semi}" d="${whitePath(w)}"/>`).join('')
  + BLACK_SEMIS.map((semi, i) => {
    const x = BLACK_AFTER[i] + 1 - BLACK_W / 2;
    return `<rect class="acc" data-semi="${semi}" x="${x}" y="0" width="${BLACK_W}" height="${BLACK_H}"/>`;
  }).join('')
  + '</svg>';

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
    setText(cells[EFF_COLS.length + 1], `${outs[c].toFixed(2)} V`);
    paramRows[c].classList.toggle('muted-row', !!sim.muted(c));
  }
}

