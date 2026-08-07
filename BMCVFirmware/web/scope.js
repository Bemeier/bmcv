// The two scopes: eight output channels, four input jacks.
//
// Both are laid out like the panel - the outputs under their own encoders, the
// inputs like the jacks - and both run at the same pixel density and the same
// time per pixel, so a trace means the same thing in either. They were
// near-identical copies of one another; the cell drawing is shared now.

import { IN_ORDER, SCOPE_ORDER } from './spec.js';
import { sim, SCOPE_LEN } from './sim.js';
import { IN_V, SCOPE_SPAN, SCOPE_V, TRACE } from './const.js';

const scopeCanvas = document.getElementById('scope');
const ctx = scopeCanvas.getContext('2d');

const inCanvas = document.getElementById('inscope');
const inCtx = inCanvas.getContext('2d');

// The output scope is 4x2 cells at this aspect. The input canvas takes its
// height from the output scope instead of from an aspect of its own, so the two
// boxes end level however wide the grid makes each of them.
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
function drawCell(c, { x0, y0, cw, ch, data, lane, head, vRange, pad, label }) {
  const k = dpr();
  const plotH = ch - pad * 2;
  const mid = y0 + pad + plotH / 2;
  const vScale = (plotH / 2) / vRange;

  c.save();
  c.beginPath();
  c.rect(x0, y0, cw, ch);
  c.clip();

  // 0V solid, +/-5V and +/-10V fainter.
  c.lineWidth = k;
  for (const v of [-10, -5, 5, 10]) {
    c.strokeStyle = Math.abs(v) === 10 ? '#262b33' : '#1d222a';
    c.beginPath();
    c.moveTo(x0, mid - v * vScale);
    c.lineTo(x0 + cw, mid - v * vScale);
    c.stroke();
  }
  c.strokeStyle = '#39404a';
  c.beginPath(); c.moveTo(x0, mid); c.lineTo(x0 + cw, mid); c.stroke();

  // Decimated to the cell width: drawing 1500 points into ~250px was the
  // largest cost on the page and cannot show more than the pixels allow.
  const step = Math.max(1, Math.floor(SCOPE_SPAN / cw));
  c.strokeStyle = TRACE;
  c.lineWidth = 1.25 * k;
  c.beginPath();
  for (let i = 0, n = 0; i < SCOPE_SPAN; i += step, n++) {
    const idx = (head - SCOPE_SPAN + i + SCOPE_LEN * 2) % SCOPE_LEN;
    const v = data[lane * SCOPE_LEN + idx];
    const x = x0 + i / (SCOPE_SPAN - 1) * cw;
    const y = mid - v * vScale;
    n ? c.lineTo(x, y) : c.moveTo(x, y);
  }
  c.stroke();

  // The range markers go on every cell - a trace means nothing without the
  // scale it is drawn against - and only the lane name is optional.
  const fs = Math.max(9, Math.round(5.5 * k));
  c.font = `${fs}px ui-monospace, monospace`;
  c.textBaseline = 'middle';
  if (label) {
    c.fillStyle = '#8b929d';
    c.fillText(label, x0 + fs * 0.6, y0 + pad + fs);
  }
  c.fillStyle = '#5a616b';
  const mark = `${vRange}`;
  c.fillText(`+${mark}`, x0 + cw - fs * (mark.length + 1.6), mid - vRange * vScale + fs * 0.8);
  c.fillText(`-${mark}`, x0 + cw - fs * (mark.length + 1.6), mid + vRange * vScale - fs * 0.8);
  c.restore();

  c.strokeStyle = '#242931';
  c.lineWidth = k;
  c.strokeRect(x0 + .5, y0 + .5, cw - 1, ch - 1);
}

function drawGrid(c, canvas, { cols, rows, lanes, data, head, vRange, pad, label }) {
  const W = canvas.width, H = canvas.height;
  const cw = W / cols, ch = H / rows;

  c.fillStyle = '#101216';
  c.fillRect(0, 0, W, H);

  for (let cell = 0; cell < lanes.length; cell++) {
    drawCell(c, {
      x0: (cell % cols) * cw,
      y0: Math.floor(cell / cols) * ch,
      cw, ch, data, head, vRange, pad,
      lane: lanes[cell],
      label: label ? label(lanes[cell]) : null,
    });
  }
}

export function drawScopes() {
  const head = sim.scopeHead();

  if (scopeCanvas.width) {
    drawGrid(ctx, scopeCanvas, {
      cols: 4, rows: 2,
      lanes: SCOPE_ORDER,
      data: sim.scope(),
      head, vRange: SCOPE_V, pad: 4,
      label: c => `CH${c}`,
    });
  }

  if (inCanvas.width) {
    drawGrid(inCtx, inCanvas, {
      cols: 2, rows: 2,
      lanes: IN_ORDER,
      data: sim.inputScope(),
      head, vRange: IN_V, pad: 4 * dpr(),
    });
  }
}

// Both canvases size themselves from their container, on resize only.
// getBoundingClientRect() forces a layout, so doing it per frame - which the
// input scope used to - is a cost for nothing: the size only changes when the
// window does.
const fitAll = () => {
  const outH = fitCanvas(scopeCanvas, { aspect: OUT_ASPECT });
  fitCanvas(inCanvas, { cssHeight: outH || inCanvas.getBoundingClientRect().width * 0.88 });
};
new ResizeObserver(fitAll).observe(scopeCanvas.parentElement);
new ResizeObserver(fitAll).observe(inCanvas.parentElement);
fitAll();
