// The two scopes: eight output channels, four input jacks.
//
// Both are laid out like the panel - the outputs under their own encoders, the
// inputs like the jacks - and both run at the same pixel density, the same time
// per pixel and the same cell size, so a trace means the same thing in either.
// They were near-identical copies of one another; the cell drawing is shared
// now, and the two canvases sit in one box so the geometry cannot drift.

import { IN_ORDER, SCOPE_ORDER } from './spec.js';
import { EFF, sim, SCOPE_LEN, INPUT_MODE_NAMES } from './sim.js';
import { CELL_FILL, CELL_GAP, IN_V, INPUT_MODE_COLORS, SCOPE_SECONDS, SCOPE_SECONDS_LIVE, SCOPE_V, TRACE, TRACE_IN } from './const.js';
import { mode } from './mode.js';

// How many samples that span works out to, given whatever is filling the ring.
// Recomputed per draw because a probe's rate is measured rather than declared,
// and clamped: never more history than the ring holds, never so few points that
// a "trace" is two of them.
export function spanSamples() {
  const seconds = mode.live ? SCOPE_SECONDS_LIVE : SCOPE_SECONDS;
  return Math.max(2, Math.min(SCOPE_LEN, Math.round(seconds * mode.captureHz)));
}

const scopeCanvas = document.getElementById('scope');
const ctx = scopeCanvas.getContext('2d');

const inCanvas = document.getElementById('inscope');
const inCtx = inCanvas.getContext('2d');

// The output scope is 4x2 cells at this aspect. The input canvas takes its
// height from the output scope rather than an aspect of its own, and the CSS
// gives it exactly half the width - so 2 columns against 4 over the same height
// makes a cell the same rectangle in either, and a trace the same shape.
const OUT_ASPECT = 0.44;

const dpr = () => window.devicePixelRatio || 1;

// Match the backing store to the element's real on-screen size. The canvas used
// to be a fixed 1000x440 stretched to whatever width the layout gave it, which
// on a wide or HiDPI display meant visibly upscaled, soft pixels.
//
// The width always comes from the layout; the height is either an aspect of
// that width or an explicit number of CSS pixels. Returns the CSS height it
// settled on, so one canvas can be sized from another.
function fitCanvas(canvas, { aspect, cssHeight }) {
  const rect = canvas.getBoundingClientRect();
  if (!rect.width) return 0;

  const k = dpr();
  const cssH = Math.round(cssHeight ?? rect.width * aspect);
  const w = Math.round(rect.width * k);
  const h = Math.round(cssH * k);
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
    canvas.style.height = `${cssH}px`;
  }
  return cssH;
}

// One cell: voltage grid, then the trace, then a border. `data` is the whole
// channel-major ring; `lane` picks which channel or jack out of it.
function drawCell(c, { x0, y0, cw, ch, data, lane, head, span, valid, vRange, pad, label, labelColor, aliased, trace }) {
  const k = dpr();
  const plotH = ch - pad * 2;
  const mid = y0 + pad + plotH / 2;
  const vScale = (plotH / 2) / vRange;

  c.save();
  c.beginPath();
  c.rect(x0, y0, cw, ch);
  c.clip();

  // The surface, behind the grid and the trace.
  c.fillStyle = CELL_FILL;
  c.fillRect(x0, y0, cw, ch);

  // 0V solid, +/-5V fainter. The rails are not ruled: they sit within a pixel or
  // two of the cell's own edge, so a line there reads as a thicker frame rather
  // than as a level. The numbers stay - it is the scale that matters, not a
  // line under it.
  //
  // Near-neutral, like the ground they are drawn on. They were blue-grey
  // against a blue-grey fill, which was invisible until the fill went neutral
  // and left them the only cool thing in the cell.
  c.lineWidth = k;
  c.strokeStyle = '#2a2a2e';
  for (const v of [-5, 5]) {
    c.beginPath();
    c.moveTo(x0, mid - v * vScale);
    c.lineTo(x0 + cw, mid - v * vScale);
    c.stroke();
  }
  c.strokeStyle = '#46464c';
  c.beginPath(); c.moveTo(x0, mid); c.lineTo(x0 + cw, mid); c.stroke();

  // Decimated to the cell width: drawing 1500 points into ~250px was the
  // largest cost on the page and cannot show more than the pixels allow.
  //
  // `drawn` is usually the whole window and is less only just after a gap in
  // the sampling - tabbing back to the page, most often. The trace then grows
  // in from the right edge over the next couple of seconds rather than being
  // stretched across the cell, so the x axis keeps meaning one thing.
  const drawn = Math.min(span, valid);
  const step = Math.max(1, Math.floor(drawn / cw));
  c.strokeStyle = trace;
  c.lineWidth = 1.25 * k;
  c.beginPath();
  for (let i = 0, n = 0; i < drawn; i += step, n++) {
    const idx = (head - drawn + i + SCOPE_LEN * 2) % SCOPE_LEN;
    const v = data[lane * SCOPE_LEN + idx];
    const x = x0 + (span - drawn + i) / (span - 1) * cw;
    const y = mid - v * vScale;
    n ? c.lineTo(x, y) : c.moveTo(x, y);
  }
  c.stroke();

  c.textBaseline = 'middle';

  // Which jack or channel this is, and the only thing on a cell worth reading
  // from across the room: with two dozen traces on screen the label is how you
  // find the one you want. Bigger and brighter than the scale marks, which you
  // only look at once.
  if (label) {
    const lfs = Math.max(11, Math.round(8 * k));
    c.font = `600 ${lfs}px ui-monospace, SFMono-Regular, Menlo, monospace`;
    // A label may name its own colour. The inputs use it to say what each jack
    // is configured as in the same hue the module's own LED shows for that
    // mode, so the two do not have to be matched up by memory.
    c.fillStyle = labelColor || '#aab2bd';
    c.fillText(label, x0 + lfs * 0.55, y0 + pad + lfs * 0.85);
  }

  const fs = Math.max(9, Math.round(5.5 * k));
  c.font = `${fs}px ui-monospace, monospace`;

  // Said outright rather than left to be discovered. Above half the sample rate
  // a trace is not a worse drawing of the waveform, it is a drawing of a
  // different waveform - a smooth, plausible one the module is not producing.
  // Nothing about the picture gives that away, so the cell has to.
  if (aliased) {
    c.fillStyle = '#e3b341';
    c.fillText('aliased', x0 + fs * 0.6, y0 + ch - pad - fs * 0.6);
  }

  // The range markers go on every cell - a trace means nothing without the
  // scale it is drawn against.
  c.fillStyle = '#63636b';
  const mark = `${vRange}`;
  c.fillText(`+${mark}`, x0 + cw - fs * (mark.length + 1.6), mid - vRange * vScale + fs * 0.8);
  c.fillText(`-${mark}`, x0 + cw - fs * (mark.length + 1.6), mid + vRange * vScale - fs * 0.8);
  c.restore();
}

function drawGrid(c, canvas, { cols, rows, lanes, data, head, span, valid, vRange, pad, label, labelColor, aliased, trace }) {
  const W = canvas.width, H = canvas.height;
  const cw = W / cols, ch = H / rows;

  // Cleared rather than filled: the canvas sits on the panel box's own
  // background, so the cells are grid lines and traces over it rather than a
  // black rectangle laid on top of it.
  c.clearRect(0, 0, W, H);

  // Each cell inset by half a gap on every side, so neighbours are separated by
  // a whole one and the page shows through - the same separation the readout
  // cells get from the flexbox, drawn by hand because these share a canvas.
  const g = Math.round(CELL_GAP * dpr());

  for (let cell = 0; cell < lanes.length; cell++) {
    drawCell(c, {
      x0: (cell % cols) * cw + g / 2,
      y0: Math.floor(cell / cols) * ch + g / 2,
      cw: cw - g, ch: ch - g,
      data, head, span, valid, vRange, pad, trace,
      lane: lanes[cell],
      label: label ? label(lanes[cell]) : null,
      labelColor: labelColor ? labelColor(lanes[cell]) : null,
      aliased: aliased ? aliased(lanes[cell]) : false,
    });
  }
}

export function drawScopes() {
  const head = sim.scopeHead();
  const span = spanSamples();
  const valid = mode.contiguous;

  // Only worth asking on hardware. The simulator captures every engine tick, so
  // its Nyquist limit is 2kHz and no channel comes close; a probe's is around
  // fifteen hertz and channels cross it all the time.
  const nyquist = mode.captureHz / 2;
  const eff = mode.live ? sim.effective() : null;
  const isAliased = c => eff !== null && eff[c * EFF.COUNT + EFF.FREQ_HZ] > nyquist;

  if (scopeCanvas.width) {
    drawGrid(ctx, scopeCanvas, {
      cols: 4, rows: 2,
      lanes: SCOPE_ORDER,
      data: sim.scope(),
      head, span, valid, vRange: SCOPE_V, pad: 4, trace: TRACE,
      label: c => `CH${c}`,
      aliased: isAliased,
    });
  }

  if (inCanvas.width) {
    drawGrid(inCtx, inCanvas, {
      cols: 4, rows: 1,
      lanes: IN_ORDER,
      data: sim.inputScope(),
      head, span, valid, vRange: IN_V, pad: 4 * dpr(), trace: TRACE_IN,
      // The jack and what it is configured as, together. It was under the
      // scope with the controls, which meant it disappeared along with them
      // whenever a module was driving the page - exactly when knowing an
      // input's mode matters most.
      label: i => {
        const mode = INPUT_MODE_NAMES[sim.inputMode(i)] ?? '';
        return mode && mode !== '—' ? `IN${i} ${mode}` : `IN${i}`;
      },
      labelColor: i => INPUT_MODE_COLORS[sim.inputMode(i)] ?? null,
    });
  }
}

// Both canvases size themselves from their container, on resize only.
// getBoundingClientRect() forces a layout, so doing it per frame - which the
// input scope used to - is a cost for nothing: the size only changes when the
// window does.
// The inputs are one row under the outputs' two, at the same width, so they get
// half the height rather than being matched to it.
const fitAll = () => {
  const outH = fitCanvas(scopeCanvas, { aspect: OUT_ASPECT });
  fitCanvas(inCanvas, { cssHeight: (outH || 440) / 2 });
};
new ResizeObserver(fitAll).observe(scopeCanvas.parentElement);
new ResizeObserver(fitAll).observe(inCanvas.parentElement);
fitAll();
