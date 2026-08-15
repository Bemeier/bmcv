// The two scopes: eight output channels, four input jacks.
//
// Both are laid out like the panel - the outputs under their own encoders, the
// inputs like the jacks - and both run at the same pixel density, the same time
// per pixel and the same cell size, so a trace means the same thing in either.
// They were near-identical copies of one another; the cell drawing is shared
// now, and the two canvases sit in one box so the geometry cannot drift.

import { IN_ORDER, SCOPE_ORDER } from './spec.js';
import { EFF, sim, SCOPE_LEN, INPUT_MODE_NAMES, SHAPE_NAMES } from './sim.js';
import {
  CELL_FILL, CELL_GAP, CELL_RADIUS, IN_V, INPUT_MODE_COLORS, SCOPE_SECONDS,
  SCOPE_V, SHAPE_MODE_COLORS, TRACE, TRACE_IN,
} from './const.js';
import { mode } from './mode.js';

// How many samples that span works out to, given whatever is filling the ring.
// Recomputed per draw because a link's rate is measured rather than declared,
// and clamped: never more history than the ring holds, never so few points that
// a "trace" is two of them.
//
// The seconds are the same whatever is driving the page; only the rate the ring
// fills at differs, which is exactly what this converts.
export function spanSamples() {
  return Math.max(2, Math.min(SCOPE_LEN, Math.round(SCOPE_SECONDS * mode.captureHz)));
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

// Where one cell of a cols x rows grid sits on a canvas of W x H device pixels.
//
// The gap goes *between* cells and nowhere else, so the outer ones sit flush
// against the canvas edge. Insetting every cell by half a gap on every side -
// which is what this used to do - spends the same gap again around the outside,
// where there is no neighbour to separate from. That was invisible at a gap of
// four and obvious at six: the grid pulled three pixels off its container on
// every side, and the scopes no longer lined up with the box holding them.
//
// One function because drawGrid and inputCellAt are inverses of each other and
// have to agree exactly - a fader that sets 3V where the trace draws 3.2V is
// worse than no fader.
function cellGeom(W, H, cols, rows) {
  const g = Math.round(CELL_GAP * dpr());
  return {
    g,
    cw: (W - g * (cols - 1)) / cols,
    ch: (H - g * (rows - 1)) / rows,
  };
}

// One cell: voltage grid, then the trace, then a border. `data` is the whole
// channel-major ring; `lane` picks which channel or jack out of it.
function drawCell(c, { x0, y0, cw, ch, data, lane, head, span, valid, vRange, pad, label, labelColor, aliased, trace }) {
  const k = dpr();
  const plotH = ch - pad * 2;
  const mid = y0 + pad + plotH / 2;
  const vScale = (plotH / 2) / vRange;

  // Rounded by the same couple of pixels as everything else on the page, and
  // clipped to that shape so the trace and the grid stop at the corner rather
  // than running into it.
  c.save();
  c.beginPath();
  c.roundRect(x0, y0, cw, ch, CELL_RADIUS * k);
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
  c.strokeStyle = '#3d3d43';
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

  // Cleared rather than filled: the canvas sits on the panel box's own
  // background, so the cells are grid lines and traces over it rather than a
  // black rectangle laid on top of it.
  c.clearRect(0, 0, W, H);

  // Neighbours separated by a whole gap and the page showing through - the same
  // separation the readout cells get from the flexbox, drawn by hand because
  // these share a canvas.
  const { g, cw, ch } = cellGeom(W, H, cols, rows);

  for (let cell = 0; cell < lanes.length; cell++) {
    drawCell(c, {
      x0: (cell % cols) * (cw + g),
      y0: Math.floor(cell / cols) * (ch + g),
      cw, ch,
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
  // Two limits on how far back a cell may draw, and the smaller wins.
  //
  // contiguous is about *time*: a tab in the background stops being sampled, so
  // everything before the gap would be drawn as though the axis meant something
  // across it.
  //
  // scopeFilled is about *provenance*: the ring is cleared when the page
  // changes what fills it, and until it refills, the samples behind the newest
  // one are either zeroes or the source that was on screen a moment ago. A
  // simulation showing the tail of a module it is no longer connected to is one
  // trace that appears to have done something abrupt.
  const valid = Math.min(mode.contiguous, sim.scopeFilled());

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

      // The channel and the shape it is running, in the hue the module lights
      // that shape with - the same arrangement the inputs below use, and for
      // the same reason. It was a column in the channel table, where it was a
      // word to be looked up against a row number; here it is on the trace it
      // describes, which is where you are already looking when a channel is not
      // doing what you expected.
      label: c => {
        const shape = SHAPE_NAMES[sim.shapeMode(c)] ?? '';
        return shape ? `CH${c} ${shape}` : `CH${c}`;
      },
      labelColor: c => SHAPE_MODE_COLORS[sim.shapeMode(c)] ?? null,
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

/* ---- pointing at an input cell ------------------------------------------- */

// Which input a point on the input canvas is in, and what voltage its height
// means.
//
// Exported rather than worked out again in inputs.js, because it is the inverse
// of what drawCell does and the two have to agree exactly: a fader that sets 3V
// where the trace draws 3.2V is worse than no fader. The cell rectangle comes
// from cellGeom, the same one drawGrid lays the cells out with, and the pad is
// the same number drawScopes passes it.
//
// Returns null outside the canvas. Inside it every point belongs to a column,
// including the gap between two of them.
export function inputCellAt(clientX, clientY) {
  const r = inCanvas.getBoundingClientRect();
  if (!r.width || !r.height) return null;

  // Client pixels to canvas pixels, which differ by the device ratio.
  const x = ((clientX - r.left) / r.width) * inCanvas.width;
  const y = ((clientY - r.top) / r.height) * inCanvas.height;

  if (x < 0 || x >= inCanvas.width || y < 0 || y >= inCanvas.height) return null;

  const cols = IN_ORDER.length;
  const { g, cw, ch } = cellGeom(inCanvas.width, inCanvas.height, cols, 1);

  // Half a gap of bias, so the split between two columns falls down the middle
  // of the seam rather than at the right edge of the left cell: a pointer a
  // pixel into the gap plainly meant one of the jacks either side of it, and
  // the one it lands in should be the one it is nearer.
  const col = Math.min(cols - 1, Math.max(0, Math.floor((x + g / 2) / (cw + g))));

  const pad = 4 * dpr();
  const plotH = ch - pad * 2;
  const mid = pad + plotH / 2;
  const vScale = (plotH / 2) / IN_V;

  // Above the plot reads as the top of the range and below it as the bottom,
  // rather than as nothing: the pad is a couple of pixels and a click that
  // lands in it plainly meant the end of the travel.
  const volts = Math.max(-IN_V, Math.min(IN_V, (mid - y) / vScale));
  return { index: IN_ORDER[col], volts };
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
