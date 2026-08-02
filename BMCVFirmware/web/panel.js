// The panel: SVG built from the generated spec, and the pointer handling that
// maps it onto the module's buttons, encoders and crossfader.
//
// Nothing here knows where any control is. Positions, sizes, LED assignments
// and the roles behind the hover hints all come from panel.json.

import { spec, px } from './spec.js';
import { sim } from './sim.js';
import { initLeds, registerLed } from './leds.js';

const SVG_NS = 'http://www.w3.org/2000/svg';

// Part sizes in mm, matching the hardware: 6mm illuminated switches, 7mm
// unlit tactiles, and an encoder body smaller than its 12mm courtyard.
const ENC_R = 5.6;
// The artwork's encoder cutout is 7.03mm across; sit just inside it so a
// hairline of the cutout still shows.
const ENC_CAP_R = 3.35;
const BTN_SWITCH_R = 3.0;
const BTN_TACTILE_R = 3.5;

// 0 is the left end of travel. Scene A anchors at SLIDER_MAX_VALUE and sits on
// the left of the panel, so the leftmost position is full scale.
export const SLIDER_START_POS = 0;

const svg = document.getElementById('panel');
svg.setAttribute('viewBox', `0 0 ${spec.panel.width_mm} ${spec.panel.height_mm}`);

const el = (name, attrs = {}, parent = svg) => {
  const n = document.createElementNS(SVG_NS, name);
  for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v);
  parent.appendChild(n);
  return n;
};

const hint = document.getElementById('hint');
const setHint = t => { hint.textContent = t || ' '; };

const describe = b => {
  const r = b.roles, bits = [];
  if (r.param_name) bits.push(`${r.param_name} / ${r.ctrl_name}`);
  else if (r.ctrl_name) bits.push(r.ctrl_name);
  if (r.scene !== undefined) bits.push(`scene ${r.scene}`);
  if (r.semitone_name) bits.push(r.semitone_name);
  if (r.channel !== undefined) bits.push(`ch${r.channel} push`);
  return `${b.designator} — ${bits.join(' / ') || 'button ' + b.index}`;
};

/* ---- interaction -------------------------------------------------------- */

function bindButton(node, b) {
  const down = ev => { ev.preventDefault(); node.setPointerCapture(ev.pointerId); sim.setButton(b.index, 1); };
  const up = () => sim.setButton(b.index, 0);
  node.addEventListener('pointerdown', down);
  node.addEventListener('pointerup', up);
  node.addEventListener('pointercancel', up);
  node.addEventListener('pointerenter', () => setHint(describe(b)));
  node.addEventListener('pointerleave', () => setHint(''));
}

// Cosmetic needle angles. The firmware's encoders are relative and endless, so
// this only shows that something turned - but it has to move by exactly what
// was sent, which is why every turn goes through turnEncoder().
const encAngle = new Map();
const encIndicators = new Map();

function turnEncoder(index, detents) {
  sim.addEncoder(index, detents);
  encAngle.set(index, (encAngle.get(index) ?? 0) + detents * 12);
}

function bindEncoder(ring, cap, e) {
  // Push is the centre cap; turning is the ring. Holding Shift while turning
  // asserts the push too, which is the press-and-turn the firmware treats as a
  // fine-adjust modifier.
  const capDown = ev => { ev.preventDefault(); cap.setPointerCapture(ev.pointerId); sim.setButton(e.push_button, 1); };
  const capUp = () => sim.setButton(e.push_button, 0);
  cap.addEventListener('pointerdown', capDown);
  cap.addEventListener('pointerup', capUp);
  cap.addEventListener('pointercancel', capUp);

  let dragging = false, lastY = 0, accum = 0, shifted = false;

  ring.addEventListener('pointerdown', ev => {
    ev.preventDefault();
    ring.setPointerCapture(ev.pointerId);
    dragging = true; lastY = ev.clientY; accum = 0;
    shifted = ev.shiftKey;
    if (shifted) sim.setButton(e.push_button, 1);
  });
  ring.addEventListener('pointermove', ev => {
    if (!dragging) return;
    accum += (lastY - ev.clientY) / 6;  // ~6px per detent, up = clockwise
    lastY = ev.clientY;
    const steps = Math.trunc(accum);
    if (steps) { accum -= steps; turnEncoder(e.index, steps); }
  });
  const stop = () => {
    if (!dragging) return;
    dragging = false;
    if (shifted) { sim.setButton(e.push_button, 0); shifted = false; }
  };
  ring.addEventListener('pointerup', stop);
  ring.addEventListener('pointercancel', stop);

  // On the cap as well as the ring: the cap is a good half of the knob, and
  // having the wheel do nothing over the middle of it reads as a dead spot.
  const wheel = ev => { ev.preventDefault(); turnEncoder(e.index, ev.deltaY < 0 ? 1 : -1); };
  ring.addEventListener('wheel', wheel, { passive: false });
  cap.addEventListener('wheel', wheel, { passive: false });

  const label = `${e.designator} — encoder ${e.index}, channel ${e.channel}`;
  for (const n of [ring, cap]) {
    n.addEventListener('pointerenter', () => setHint(label));
    n.addEventListener('pointerleave', () => setHint(''));
  }
}

// Returns the one function that moves both the drawn knob and the engine, so
// the two cannot disagree - the panel used to draw the fader at one end while
// the engine had booted with it at the other, and scene B stayed active until
// the first drag snapped it over.
function bindSlider(hit, knob, cx, cy, travel, horizontal) {
  let dragging = false;
  hit.style.cursor = horizontal ? 'ew-resize' : 'ns-resize';

  const setFromPos = pos => {
    sim.setSlider01(1 - pos);
    if (horizontal) knob.setAttribute('x', cx - travel / 2 - 2 + pos * travel);
    else knob.setAttribute('y', cy + travel / 2 - 2 - pos * travel);
  };

  const apply = ev => {
    const r = svg.getBoundingClientRect();
    // Client px -> viewBox mm, then to 0..1. Right / up is 1.0.
    let pos;
    if (horizontal) {
      const mm = (ev.clientX - r.left) / r.width * spec.panel.width_mm;
      pos = (mm - (cx - travel / 2)) / travel;
    } else {
      const mm = (ev.clientY - r.top) / r.height * spec.panel.height_mm;
      pos = ((cy + travel / 2) - mm) / travel;
    }
    pos = Math.min(1, Math.max(0, pos));
    setFromPos(pos);
    setHint(`slider ${pos.toFixed(3)}`);
  };

  hit.addEventListener('pointerdown', ev => { ev.preventDefault(); hit.setPointerCapture(ev.pointerId); dragging = true; apply(ev); });
  hit.addEventListener('pointermove', ev => { if (dragging) apply(ev); });
  hit.addEventListener('pointerup', () => { dragging = false; });
  hit.addEventListener('pointercancel', () => { dragging = false; });
  hit.addEventListener('pointerenter', () => setHint('scene crossfader'));
  hit.addEventListener('pointerleave', () => { if (!dragging) setHint(''); });

  return setFromPos;
}

/* ---- building ----------------------------------------------------------- */

// The exported panel artwork, 1:1 over the panel rectangle. Everything else
// draws on top of it, so the controls stay live - and it landing exactly on the
// generated geometry is a useful check on the whole KiCad-derived pipeline.
el('image', {
  href: 'bmcv_panel.png', x: 0, y: 0,
  width: spec.panel.width_mm, height: spec.panel.height_mm,
  preserveAspectRatio: 'none',
});

// Jacks. Position identifies these well enough - the outputs sit under their
// own encoders and the inputs are the middle pair.
for (const list of [spec.outputs, spec.inputs]) {
  for (const j of list) {
    const [x, y] = px(j.pos_mm);
    el('circle', { cx: x, cy: y, r: 3, fill: '#0d0f12', stroke: '#7d848f', 'stroke-width': .4 });
    el('circle', { cx: x, cy: y, r: 1.1, fill: '#2a2f36' });
  }
}

// The halo layer sits behind every control, so the spill reads as light coming
// from around the part rather than as a disc painted over it.
const haloLayer = el('g');
initLeds(el('defs'));

// Encoders: outer ring turns, centre cap pushes.
for (const e of spec.encoders) {
  const [x, y] = px(e.pos_mm);

  // The WS2812 sits behind the encoder, so its light spills around the body
  // rather than through it: a ring in the layer behind everything.
  registerLed(e.led, el('circle', { cx: x, cy: y, r: ENC_R * 1.45, fill: '#000', 'fill-opacity': 0 }, haloLayer), 'halo');

  const g = el('g');
  el('circle', { cx: x, cy: y, r: ENC_R, fill: '#6b7078', 'fill-opacity': .16, stroke: '#101317', 'stroke-width': .45 }, g);

  // ...plus a faint wash across the face, so the colour bleeds through the
  // knob instead of stopping at its edge.
  registerLed(e.led, el('circle', {
    cx: x, cy: y, r: ENC_R * 0.95, fill: '#000', 'fill-opacity': 0, 'pointer-events': 'none',
  }, g), 'lamp', 0.85);

  // Grab area well wider than the body: the ring is the control you turn, and
  // the body itself is small enough that hitting it takes aim otherwise.
  const ring = el('circle', { cx: x, cy: y, r: ENC_R + 2.4, fill: 'transparent', class: 'enc-ring' }, g);

  // Four spokes, not one: a single mark reads like an absolute pointer, and
  // these are endless relative encoders with no meaningful zero.
  // pointer-events off, or a spoke swallows the drag meant for the ring.
  const spokes = el('g', { 'pointer-events': 'none' }, g);
  for (let k = 0; k < 4; k++) {
    const a = (k * Math.PI) / 2;
    el('line', {
      x1: x + Math.sin(a) * (ENC_CAP_R + 0.3), y1: y - Math.cos(a) * (ENC_CAP_R + 0.3),
      x2: x + Math.sin(a) * (ENC_R - 0.5), y2: y - Math.cos(a) * (ENC_R - 0.5),
      stroke: '#101317', 'stroke-width': .55, 'stroke-linecap': 'round',
    }, spokes);
  }
  encIndicators.set(e.index, spokes);

  const cap = el('circle', {
    cx: x, cy: y, r: ENC_CAP_R, fill: '#6b7078', 'fill-opacity': .3,
    stroke: '#101317', 'stroke-width': .4, class: 'hit',
  }, g);

  bindEncoder(ring, cap, e);
}

// Crossfader. The artwork draws the slot, so only the handle is drawn here.
let setSliderFromPos;
{
  const s = spec.slider;
  const [x, y] = px(s.pos_mm);
  const travel = s.travel_mm;
  const horizontal = (s.axis ?? 'x') === 'x';

  // The same grey the encoder bodies wear, so the two read as the same kind of
  // part rather than the fader being the brightest thing on the panel.
  const knob = horizontal
    ? el('rect', { x: x - travel / 2 - 2, y: y - 3.3, width: 4, height: 6.6, rx: 1.1, fill: '#6b7078', stroke: '#101317', 'stroke-width': .4 })
    : el('rect', { x: x - 4.4, y: y + travel / 2 - 2, width: 8.8, height: 4, rx: 1.2, fill: '#6b7078', stroke: '#101317', 'stroke-width': .4 });
  const hit = horizontal
    ? el('rect', { x: x - travel / 2 - 3, y: y - 5.5, width: travel + 6, height: 11, fill: 'transparent', class: 'slider-hit' })
    : el('rect', { x: x - 5.5, y: y - travel / 2 - 3, width: 11, height: travel + 6, fill: 'transparent', class: 'slider-hit' });

  setSliderFromPos = bindSlider(hit, knob, x, y, travel, horizontal);
}

// Buttons, skipping encoder pushes - those are already drawn as caps.
//
// Round and sized like the parts. Only the ctrl functions carry a legend: the
// scene numbers and semitone names are left off, since a 6mm cap has no room
// and the layout already says which is which.
for (const b of spec.buttons) {
  if (b.kind === 'encoder_push') continue;

  const [x, y] = px(b.pos_mm);
  const r = b.kind === 'tactile' ? BTN_TACTILE_R : BTN_SWITCH_R;
  const lit = b.led !== null && b.led !== undefined && b.led >= 0;
  const g = el('g');

  if (lit) {
    registerLed(b.led, el('circle', { cx: x, cy: y, r: r * 1.55, fill: '#000', 'fill-opacity': 0 }, haloLayer), 'halo');
  }
  el('circle', {
    cx: x, cy: y, r,
    fill: b.kind === 'tactile' ? '#2b3038' : '#c2c7ce',
    stroke: '#101317', 'stroke-width': .35,
  }, g);
  if (lit) {
    registerLed(b.led, el('circle', {
      cx: x, cy: y, r: r - 0.2, fill: '#000', 'fill-opacity': 0, 'pointer-events': 'none',
    }, g), 'lamp');
  }

  // Legends are drawn rather than left to the artwork, so they stay correct if
  // a button's function is ever reassigned. The parameter goes above and the
  // shift mode below; the three action buttons only latch a mode, so they get
  // the lower slot alone.
  const label = (text, dy, size, fill) => {
    const t = el('text', {
      x, y: y + dy, 'font-size': size, fill, 'text-anchor': 'middle', 'pointer-events': 'none',
      'font-weight': 700, stroke: '#e9e9e6', 'stroke-width': 0.32, 'paint-order': 'stroke',
    }, g);
    t.textContent = text;
  };
  if (b.roles.param_name) label(b.roles.param_name, -r - 1.6, 2.7, '#15181c');
  if (b.roles.ctrl_name) label(b.roles.ctrl_name, r + 3.3, 2.3, '#3a4048');

  const hit = el('circle', { cx: x, cy: y, r: r + 1.4, fill: 'transparent', class: 'hit' }, g);
  bindButton(hit, b);
}

// Deliberately not called here. The engine boots with slider_raw at one end
// and has no other way to learn where the handle is drawn, so someone must
// publish it - but restoring a saved patch reboots the module, which would
// undo it. main.js calls this once startup has settled.
export { setSliderFromPos };

export function drawEncoderIndicators() {
  for (const e of spec.encoders) {
    const [cx, cy] = px(e.pos_mm);
    encIndicators.get(e.index).setAttribute('transform', `rotate(${encAngle.get(e.index) ?? 0} ${cx} ${cy})`);
  }
}
