// The panel: SVG built from the generated spec, and the pointer handling that
// maps it onto the module's buttons, encoders and crossfader.
//
// Nothing here knows where any control is. Positions, sizes, LED assignments
// and the roles behind the hover hints all come from panel.json.

import { spec, px } from './spec.js';
import { sim } from './sim.js';
import { input } from './input.js';
import { initLeds, registerLed } from './leds.js';

const SVG_NS = 'http://www.w3.org/2000/svg';

// Part sizes in mm, matching the hardware: 6mm illuminated switches, 7mm
// unlit tactiles, and an encoder body smaller than its 12mm courtyard.
const ENC_R = 5.6;
// The artwork's encoder cutout is 7.03mm across, so a cap that sits just inside
// it leaves barely a millimetre of ring to catch. The cap is a target for one
// click; the ring is a target you have to hold and drag, which wants the room
// more.
const ENC_CAP_R = 2.6;
const BTN_SWITCH_R = 3.0;
const BTN_TACTILE_R = 3.5;

// The three tactile buttons are coloured plastic on the hardware, not RGB - so
// they are drawn as a fill and nothing else. No halo and no LED registration:
// there is no light behind them to spill, and a glow would say they were doing
// something when the colour is simply what the cap is.
//
// Saturated, but mid-brightness. These have to read as coloured caps from
// across the room while sitting among parts that light up - so the hue is
// unambiguous and the value is not, which keeps them from competing with the
// LEDs, where brightness is what means something.
const TACTILE_FILL = {
  CLR: '#c24f7d', // pink - the page that destroys things
  CPY: '#3d7fc9', // blue - the page that duplicates them
  MUT: '#2fa393', // teal
};

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

/* ---- what the pointer is over -------------------------------------------- */

// Two lines: what it is, in the bracket beside the panel, and what it does,
// under it.
//
// This used to be one line naming the part - "U6 - AMP / SAV / G#" - which is
// what the panel spec knows rather than what a person wants. A designator
// identifies a component on a board and a semitone only means anything on one
// of nine pages; neither answers "what happens if I press this".

const hintHelp = document.getElementById('hint-help');

const IDLE = {
  sections: [[
    'Eight channels, seven scenes',
    'Eight looping voltage sources, each locked to the beat by a ratio rather than a '
      + 'frequency, so nothing drifts out of time. Every channel\'s six parameters are '
      + 'stored per scene, and the crossfader blends the whole patch between any two of '
      + 'the seven - one hand, every channel at once. The nine control buttons tap to '
      + 'choose what the encoders edit and hold to open a page. Hover anything here to '
      + 'read what it does.',
  ]],
};

// The names of other things, boxed where a description mentions them, so a
// sentence that sends you somewhere else says where without spelling out "the
// page called". Built from the panel spec rather than listed, so a button
// renamed in the hardware cannot leave a reference behind pointing at nothing.
const REFERENCES = [...new Set(spec.buttons.flatMap(b =>
  [b.roles?.ctrl_name, b.roles?.param_name].filter(Boolean)))];
const REFERENCE_RE = new RegExp(`\\b(${REFERENCES.join('|')})\\b`, 'g');

// Latched, not tracked.
//
// Every control on the panel used to clear this on the way out, so crossing the
// board flickered between descriptions and the module's name - the text was
// unreadable precisely while you were moving towards the thing you wanted to
// read about. It now holds the last thing hovered until something else is
// hovered, and only lets go when the pointer leaves the panel altogether.
// The one place the latch lets go.
if (typeof document !== 'undefined') {
  const wrap = document.getElementById('panel-wrap');
  wrap?.addEventListener('pointerleave', () => setHover(null));
}

function setHover(target) {
  const t = target ?? IDLE;

  // The bracket keeps saying what the module is. It named the hovered part for
  // a while, which put the same words in two places a line apart - the heading
  // below says it, and says it in more detail.
  hintHelp.innerHTML = t.sections
    .map(([heading, body]) => `<h3>${heading}</h3><p>${boxRefs(body)}</p>`)
    .join('');
}

const boxRefs = text => text.replace(REFERENCE_RE, '<b class="ref">$1</b>');

// What each control button's page does, in the order the panel lays them out.
// Condensed from the Shift Modes section of the README, which is the same
// source ui_mode.c's own table was written against.
const CTRL_HELP = {
  STA: 'Scene A. Tap to choose which scene the crossfader blends from; hold to open its '
    + 'page, where the encoders set each channel\'s stepped-random pattern length.',
  STB: 'Scene B. Tap to choose which scene the crossfader blends towards; hold to open '
    + 'its page.',
  SYS: 'System. Hold to open: the first four scene buttons set what each input jack is '
    + 'for, and the encoders choose a channel\'s waveshape - wavetable, stepped random '
    + 'or PWM.',
  QNT: 'Quantizer. Hold to open: the scene buttons switch semitones in and out of the '
    + 'scale, the encoders set each channel\'s quantize mode, and pressing one assigns '
    + 'its sample trigger.',
  MIX: 'Cross modulation. Hold to open: pressing an encoder picks the input a channel '
    + 'mixes in, and turning it chooses how - off, added, or multiplied.',
  SAV: 'Save and load. Hold to open: the scene buttons become seven preset slots, each '
    + 'holding the whole module rather than one scene, and the encoders set each '
    + 'channel\'s output clamp.',
  MUT: 'Mute. Hold to open: press an encoder to mute that channel, or turn it - right '
    + 'unmutes, left mutes, so a row can be muted by feel. A muted channel keeps '
    + 'running and still feeds anything modulating from it.',
  CPY: 'Copy. Hold to open: pick a scene or channel to copy from, then one to copy to.',
  CLR: 'Clear. Hold to open: tap an encoder to clear that channel in the active scene, '
    + 'or hold it to clear the channel everywhere - every scene, its routing, its '
    + 'quantizer and its shape. Not its output clamp.',
};

// What each parameter is, for the tap rather than the hold.
const PARAM_HELP = {
  FRQ: 'a ratio against the beat rather than a level, so a channel stays locked to the '
    + 'tempo wherever it is set',
  SHP: 'the waveshape, as one continuous axis that wraps: square, sine at centre, '
    + 'triangle, and round again',
  MOD: 'what a shape does with its second dimension - wavetable skew, stepped-random '
    + 'density, or the PWM envelope',
  PHS: 'where in its cycle the channel starts, with phase 0 the rising edge in every '
    + 'shape',
  AMP: 'how far the channel swings. A module fresh out of the box is silent because '
    + 'this is zero everywhere',
  OFS: 'a fixed voltage added to the channel\'s output',
};

function describeButton(b) {
  const r = b.roles ?? {};

  // An encoder's own push. The encoder describes itself, so this only has to
  // say what pressing adds.
  if (r.channel !== undefined) {
    return {
        sections: [[
        `Channel ${r.channel} — push`,
        'Held while turning, this is fine adjust. On its own it is whatever the open page '
          + 'does with a press: picking an input under MIX, assigning a trigger under QNT, '
          + 'muting under MUT.',
      ]],
    };
  }

  // A scene button. The first four double as the input jacks on the pages that
  // configure them, which is worth saying on the button rather than leaving to
  // be discovered.
  //
  // Only the pages that act on *this scene* are named. Every shift mode gives
  // these buttons something to do - semitones under QNT, preset slots under SAV
  // - but those are seven buttons being borrowed as a row of seven, not seven
  // scenes being operated on, and listing them here said the opposite.
  if (r.scene !== undefined) {
    const isInput = r.scene < 4;
    const sections = [[
      `Scene ${r.scene}`,
      'One of the seven sets of parameters the crossfader blends between. Assign it to A '
        + 'or B under STA or STB, clear it under CLR.',
    ]];

    if (isInput) {
      sections.push([
        `Input ${r.scene}`,
        `On the pages that configure inputs this same button stands for input jack `
          + `${r.scene}: its mode under SYS, its level under MIX.`,
      ]);
    }

    return { sections };
  }

  // A control button: the page it opens, and the parameter it selects if it has
  // one. Both, because the tap and the hold are different actions on one key.
  if (r.ctrl_name) {
    // Two things on one key, and they are not variations of each other: a tap
    // chooses what the encoders edit, a hold repaints the whole panel. Saying
    // them in one paragraph made the button sound like it did one vague thing.
    const sections = [];

    if (r.param_name) {
      sections.push([
        `${r.param_name} — parameter (tap)`,
        `Tap to make ${r.param_name} what all eight encoders edit, in the active scene. `
          + `It is ${PARAM_HELP[r.param_name]}.`,
      ]);
    }

    sections.push([
      `${r.ctrl_name} — page (hold to latch)`,
      CTRL_HELP[r.ctrl_name] ?? `The ${r.ctrl_name} page.`,
    ]);

    return { sections };
  }

  return { sections: [[`Button ${b.index}`, 'No documented function.']] };
}
/* ---- interaction -------------------------------------------------------- */

function bindButton(node, b) {
  const down = ev => { ev.preventDefault(); node.setPointerCapture(ev.pointerId); input.setButton(b.index, 1); };
  const up = () => input.setButton(b.index, 0);
  node.addEventListener('pointerdown', down);
  node.addEventListener('pointerup', up);
  node.addEventListener('pointercancel', up);
  node.addEventListener('pointerenter', () => setHover(describeButton(b)));
}

// Cosmetic needle angles. The firmware's encoders are relative and endless, so
// this only shows that something turned - but it has to move by exactly what
// the module ended up at, which is why the angle is read back from it rather
// than accumulated here.
// What the panel's own parts are drawn in.
//
// The artwork underneath is a dark, mostly transparent PNG, so these are lines
// and washes over the page rather than a rendering of an aluminium panel: one
// thin light outline everywhere, and fills that only exist to say a control is
// there. The LEDs supply all the colour.
const PART = {
  // Always one device pixel, however far the panel is scaled - which is the
  // whole reason for non-scaling-stroke. A width in millimetres looked right at
  // one size and like a marker pen at another.
  line: '#8b929d',
  lineWidth: 1,

  // Enough to lift a control off the artwork, no more. A lit button gets less,
  // because its LED is about to do the work; an unlit one has nothing else to
  // make it visible.
  fillLit: 'rgba(255, 255, 255, .05)',
  fillUnlit: 'rgba(255, 255, 255, .14)',
  fillEncoder: 'rgba(255, 255, 255, .07)',
  fillCap: 'rgba(255, 255, 255, .10)',
  fillKnob: 'rgba(255, 255, 255, .16)',

  // A jack is a hole, so it is darker than the panel rather than lighter.
  jack: 'rgba(0, 0, 0, .45)',
  jackTip: 'rgba(255, 255, 255, .13)',

  // And its outline is quieter than everything else's. There are twelve of
  // them in two rows, none of them does anything on this page, and at the
  // shared line colour a dozen bright rings drew the eye to the one part of
  // the panel with nothing to say - a socket is a hole to be found when you
  // need it, not a control to be read.
  jackLine: 'rgba(139, 146, 157, .45)',

  // The parameter a control edits, and the shift mode it belongs to. The first
  // is what you read while playing; the second is reference.
  //
  // Neither is white. Pure white is the brightest thing a screen has, and on
  // this panel that belongs to the LEDs - they are the part that means
  // something by being bright, and a ring of silkscreen at full strength was
  // competing with them for it. Muted enough to sit behind the lights, still
  // well clear of the panel artwork underneath.
  label: '#b6bbc4',
  labelSecondary: '#878c96',
};

// How far an LED's spill reaches past the part it sits behind, as a multiple of
// that part's radius.
//
// Cut by about a sixth when the artwork went dark and mostly transparent. The halo is
// a soft disc that fades outwards, so on a light panel most of its reach was
// lost against the background; over a dark page all of it shows, and at the old
// radius the spill from twenty-one LEDs was the brightest thing on the screen.
//
// This is reach, not brightness - the colour and intensity still come from the
// framebuffer, so a dim LED stays dim. See leds.js.
const HALO_ENCODER = 1.22;
// Wider than the encoders', which is not symmetry lost but the parts being
// different: an illuminated switch *is* the lamp, where an encoder is a knob
// with a ring behind it, so the spill off a button reaches further on the
// hardware too.
const HALO_BUTTON = 1.5;

// Every outline on the panel, so none of them can drift from the others - and
// the jacks', which is the same line at a lower strength.
const outline = { stroke: PART.line, 'stroke-width': PART.lineWidth, 'vector-effect': 'non-scaling-stroke' };
const jackOutline = { ...outline, stroke: PART.jackLine };

const encIndicators = new Map();

// One detent of spoke rotation. Four spokes, so 12deg is a quarter turn every
// 7.5 detents - fast enough to read as movement, slow enough not to alias.
const DEG_PER_DETENT = 12;

// How long the push stays asserted after the last shift-wheel notch. Long
// enough to cover several engine ticks and the gap between two notches of one
// gesture, short enough that it is over before the next deliberate move.
const WHEEL_SHIFT_RELEASE_MS = 140;

function bindEncoder(ring, cap, e) {
  // Push is the centre cap; turning is the ring. Holding Shift while turning
  // asserts the push too, which is the press-and-turn the firmware treats as a
  // fine-adjust modifier.
  const capDown = ev => { ev.preventDefault(); cap.setPointerCapture(ev.pointerId); input.setButton(e.push_button, 1); };
  const capUp = () => input.setButton(e.push_button, 0);
  cap.addEventListener('pointerdown', capDown);
  cap.addEventListener('pointerup', capUp);
  cap.addEventListener('pointercancel', capUp);

  let dragging = false, lastY = 0, accum = 0, shifted = false;

  ring.addEventListener('pointerdown', ev => {
    ev.preventDefault();
    ring.setPointerCapture(ev.pointerId);
    dragging = true; lastY = ev.clientY; accum = 0;
    shifted = ev.shiftKey;
    if (shifted) input.setButton(e.push_button, 1);
  });
  ring.addEventListener('pointermove', ev => {
    if (!dragging) return;
    accum += (lastY - ev.clientY) / 6;  // ~6px per detent, up = clockwise
    lastY = ev.clientY;
    const steps = Math.trunc(accum);
    if (steps) { accum -= steps; input.addEncoder(e.index, steps); }
  });
  const stop = () => {
    if (!dragging) return;
    dragging = false;
    if (shifted) { input.setButton(e.push_button, 0); shifted = false; }
  };
  ring.addEventListener('pointerup', stop);
  ring.addEventListener('pointercancel', stop);

  // On the cap as well as the ring: the cap is a good half of the knob, and
  // having the wheel do nothing over the middle of it reads as a dead spot.
  //
  // Shift works here as it does on a drag, which took holding the push down
  // across the wheel gesture rather than around one event. The module reads
  // fine adjust off the button's *level* on the tick that sees the detent, and
  // a press and release inside one handler is a button that was never down when
  // a tick looked - so the press is held and released a beat after the gesture
  // stops, with each further notch pushing that release back.
  let shiftHeld = null;
  const releaseShift = () => { shiftHeld = null; input.setButton(e.push_button, 0); };

  const wheel = ev => {
    ev.preventDefault();

    if (ev.shiftKey) {
      if (shiftHeld === null) input.setButton(e.push_button, 1);
      else clearTimeout(shiftHeld);
      shiftHeld = setTimeout(releaseShift, WHEEL_SHIFT_RELEASE_MS);
    } else if (shiftHeld !== null) {
      // Let go mid-gesture: stop modifying at once rather than at the timeout.
      clearTimeout(shiftHeld);
      releaseShift();
    }

    input.addEncoder(e.index, ev.deltaY < 0 ? 1 : -1);
  };
  ring.addEventListener('wheel', wheel, { passive: false });
  cap.addEventListener('wheel', wheel, { passive: false });

  const target = {
    sections: [[
      `Channel ${e.channel} — encoder`,
      'Turning it edits whichever parameter the row under the crossfader has selected, in '
        + 'the active scene - or that page\'s own setting while a shift mode is latched. '
        + 'The centre is a button; hold it while turning for fine adjust.',
    ], [
      'On this page',
      'Drag the ring to turn it, or use the scroll wheel over either part. Click the '
        + 'centre to push, and hold Shift while turning for press-and-turn.',
    ]],
  };
  for (const n of [ring, cap]) {
    n.addEventListener('pointerenter', () => setHover(target));
  }
}

// Returns the one function that moves both the drawn knob and the engine, so
// the two cannot disagree - the panel used to draw the fader at one end while
// the engine had booted with it at the other, and scene B stayed active until
// the first drag snapped it over.
function bindSlider(hit, knob, cx, cy, travel, horizontal) {
  let dragging = false;
  hit.style.cursor = horizontal ? 'ew-resize' : 'ns-resize';

  // Drawing and publishing are separate because a physical module needs the
  // first without the second: its handle is where its owner put it, and this
  // page has no business telling the engine otherwise.
  const drawAt = pos => {
    if (horizontal) knob.setAttribute('x', cx - travel / 2 - 2 + pos * travel);
    else knob.setAttribute('y', cy + travel / 2 - 2 - pos * travel);
  };

  drawKnob = drawAt;

  const setFromPos = pos => {
    input.setSlider01(1 - pos);
    drawAt(pos);
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

  };

  hit.addEventListener('pointerdown', ev => { ev.preventDefault(); hit.setPointerCapture(ev.pointerId); dragging = true; apply(ev); });
  hit.addEventListener('pointermove', ev => { if (dragging) apply(ev); });
  hit.addEventListener('pointerup', () => { dragging = false; });
  hit.addEventListener('pointercancel', () => { dragging = false; });
  hit.addEventListener('pointerenter', () => setHover({
    sections: [[
      'Scene crossfader',
      'Blends between the two scenes assigned to A and B. Every parameter of every channel '
        + 'moves together, so one hand crossfades the whole patch. An input set to SLIDER '
        + 'mode is summed into this, so it can be driven by a cable as well as by hand.',
    ]],
  }));

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

// Jacks. Drawn as holes, and hoverable - which they were not, though they are
// the only parts of the panel a cable actually goes into.
for (const [list, kind] of [[spec.outputs, 'out'], [spec.inputs, 'in']]) {
  for (const j of list) {
    const [x, y] = px(j.pos_mm);
    el('circle', { cx: x, cy: y, r: 3, fill: PART.jack, ...jackOutline });
    el('circle', { cx: x, cy: y, r: 1.1, fill: PART.jackTip });

    // A hit area over the pair, larger than either: a 3mm circle is a small
    // target and there is nothing else nearby to catch by mistake.
    const hit = el('circle', { cx: x, cy: y, r: 4.5, fill: 'transparent', class: 'jack-hit' });
    const target = kind === 'out'
      ? {
        sections: [[
          `Channel ${j.channel} — output`,
          'What the channel puts out, after its scene blend, anything mixed into it and its '
            + 'output clamp. The scope beside this panel draws the same signal.',
        ]],
      }
      : {
        sections: [[
          `Input ${j.index} — jack`,
          'What this jack is for is set under SYS: a clock, a reset, the crossfader, or a '
            + 'plain voltage a channel can mix in. Its trace is under the outputs, and its '
            + 'mode is written beside it there.',
        ]],
      };
    hit.addEventListener('pointerenter', () => setHover(target));
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
  registerLed(e.led, el('circle', { cx: x, cy: y, r: ENC_R * HALO_ENCODER, fill: '#000', 'fill-opacity': 0 }, haloLayer), 'halo');

  const g = el('g');
  el('circle', { cx: x, cy: y, r: ENC_R, fill: PART.fillEncoder, ...outline }, g);

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
      ...outline, 'stroke-linecap': 'round',
    }, spokes);
  }
  encIndicators.set(e.index, spokes);

  const cap = el('circle', {
    cx: x, cy: y, r: ENC_CAP_R, fill: PART.fillCap, ...outline, class: 'hit',
  }, g);

  bindEncoder(ring, cap, e);
}

// Crossfader. The artwork draws the slot, so only the handle is drawn here.
let setSliderFromPos;
let drawKnob;
{
  const s = spec.slider;
  const [x, y] = px(s.pos_mm);
  const travel = s.travel_mm;
  const horizontal = (s.axis ?? 'x') === 'x';

  // The same grey the encoder bodies wear, so the two read as the same kind of
  // part rather than the fader being the brightest thing on the panel.
  const knob = horizontal
    ? el('rect', { x: x - travel / 2 - 2, y: y - 3.3, width: 4, height: 6.6, rx: 1.1, fill: PART.fillKnob, ...outline })
    : el('rect', { x: x - 4.4, y: y + travel / 2 - 2, width: 8.8, height: 4, rx: 1.2, fill: PART.fillKnob, ...outline });
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
    registerLed(b.led, el('circle', { cx: x, cy: y, r: r * HALO_BUTTON, fill: '#000', 'fill-opacity': 0 }, haloLayer), 'halo');
  }
  const capFill = TACTILE_FILL[b.roles?.ctrl_name];
  el('circle', {
    cx: x, cy: y, r,
    fill: capFill ?? (lit ? PART.fillLit : PART.fillUnlit),
    ...outline,
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
  //
  // No halo behind them. They carried a pale stroke under the fill to lift them
  // off a light artwork, which at this size read as a smudge around every word;
  // over a dark one it would be a white word inside a white outline. Light text
  // on a dark panel needs neither.
  const label = (text, dy, size, fill) => {
    const t = el('text', {
      x, y: y + dy, 'font-size': size, fill, 'text-anchor': 'middle', 'pointer-events': 'none',
      'font-weight': 700,
    }, g);
    t.textContent = text;
  };
  if (b.roles.param_name) label(b.roles.param_name, -r - 1.7, 3.1, PART.label);
  if (b.roles.ctrl_name) label(b.roles.ctrl_name, r + 3.4, 2.65, PART.labelSecondary);

  const hit = el('circle', { cx: x, cy: y, r: r + 1.4, fill: 'transparent', class: 'hit' }, g);
  bindButton(hit, b);
}

// Deliberately not called here. The engine boots with slider_raw at one end
// and has no other way to learn where the handle is drawn, so someone must
// publish it - but restoring a saved patch reboots the module, which would
// undo it. main.js calls this once startup has settled.
export { setSliderFromPos };

// Put the drawn handle where the module says the crossfade is. For probe mode:
// the fader on the board moves and this one has to follow.
//
// Not used for the local simulator, where it would be a downgrade rather than a
// refinement. What comes back is the slider *after* input_fold has summed any
// CV routed to it, so an input in slider mode would drag the drawn handle about
// while the physical one sat still. On hardware that is the closest thing to
// the truth available - the raw position never leaves the input layer - and
// here the raw position is what this page set in the first place.
export function drawSliderFromModule() {
  drawKnob(1 - sim.slider01());
}

// Drawn from the module's own encoder positions rather than from a tally of
// mouse drags. The two agree while this page is the only thing turning them,
// and part company the moment it is not: a physical module's encoders are
// wherever its owner left them, and a spoke pattern tracking drags alone would
// sit at zero however much the real knob moved.
export function drawEncoderIndicators() {
  for (const e of spec.encoders) {
    const [cx, cy] = px(e.pos_mm);
    const deg = sim.encoderPos(e.index) * DEG_PER_DETENT;
    encIndicators.get(e.index).setAttribute('transform', `rotate(${deg} ${cx} ${cy})`);
  }
}
