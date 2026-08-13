// The 21 WS2812s, driven from the real framebuffer.
//
// Drawing them as plain circles in a layer of their own does not work: the
// opaque button and knob bodies sit on top and hide them completely.
// Physically the switch cap *is* the lamp, so the control itself has to light
// up. Each LED therefore drives two things - a halo, the spill *around* a
// control, and a lamp, the control's own face - and both are transparent when
// the LED is off, so an unlit control keeps its normal colour rather than
// being painted black.

import { N_LEDS, sim } from './sim.js';

const SVG_NS = 'http://www.w3.org/2000/svg';

// led index -> { halo: [], lamp: [] }
const ledNodes = new Map();
let defs = null;

export function initLeds(defsNode) {
  defs = defsNode;
  ledNodes.clear();
  lastFill.fill(null);
}

// A radial gradient per LED rather than a flat fill: the edge fades instead of
// stopping dead, which is what an emitter behind a diffuser looks like. Done
// with gradients rather than an SVG blur filter because a paint server costs
// nothing to rasterise - a filter over a panel-sized group, re-rasterised every
// animation frame, crashed the renderer outright.
function ledGradient(id, stops) {
  const rg = document.createElementNS(SVG_NS, 'radialGradient');
  rg.setAttribute('id', id);
  const nodes = stops.map(([offset, opacity]) => {
    const st = document.createElementNS(SVG_NS, 'stop');
    st.setAttribute('offset', offset);
    st.setAttribute('stop-color', '#000');
    st.setAttribute('stop-opacity', opacity);
    rg.appendChild(st);
    return st;
  });
  defs.appendChild(rg);
  return { id, stops: nodes };
}

// Solid through the middle with the falloff only at the rim, rather than a
// bright point fading linearly all the way out.
const GRADIENT_STOPS = {
  halo: [['0%', 1], ['62%', 1], ['82%', 0.5], ['100%', 0]],
  lamp: [['0%', 1], ['76%', 1], ['92%', 0.55], ['100%', 0]],
};

// `k` is a per-node brightness weight, so one LED can drive a soft ring and a
// fainter wash across the same control.
export function registerLed(index, node, kind, k = 1) {
  if (index === null || index === undefined || index < 0) return;
  if (!ledNodes.has(index)) ledNodes.set(index, { halo: [], lamp: [] });

  const entries = ledNodes.get(index)[kind];
  const grad = ledGradient(`led-${kind}-${index}-${entries.length}`, GRADIENT_STOPS[kind]);
  node.setAttribute('fill', `url(#${grad.id})`);
  entries.push({ node, grad, k });
}

// Mirrors sim/include/led_color.h, against the constants in
// Core/Inc/Lib/led_curve.h. Both frontends have to agree or they show different
// modules - and the ramp is meant to be tuned here and then flashed, which only
// works if what is on screen is what the panel will do.
//
// The framebuffer is 8.8 fixed point duty, and the panel's own ceiling is a
// small fraction of 255 because real WS2812s are painfully bright above it - so
// the brightest thing the renderer draws is full scale here, or everything is
// invisible.
//
// Getting from duty to a colour takes two corrections, not one. The dies are
// not equally efficient, so duty is not light; and light is not hue, because
// how much a die puts out and how blue something looks are different questions.
// A blue at SAT_HIG carries a third as much green duty as blue, and green is
// nearly four times the blue die, so by light that colour is green-dominant
// while plainly being blue to look at. Dividing each die's light by what the
// matching screen primary contributes puts the hue back.
//
// Flat, one per line, because smoke.mjs reads both these and the #defines they
// mirror and checks the numbers match.
const LED_UNIT = 256;
const LED_W_RED = 0.80;
const LED_W_GREEN = 1.75;
const LED_W_BLUE = 0.45;
const LED_Y_RED = 0.2126;
const LED_Y_GREEN = 0.7152;
const LED_Y_BLUE = 0.0722;
const LED_PALETTE_REF = 1.75;
const LED_DISPLAY_FULL = 64 * LED_PALETTE_REF; // VAL_HIG, balanced
const LED_DISPLAY_GAMMA = 2.2;
const LED_SCALE = LED_DISPLAY_FULL * LED_UNIT;

const encode = (linear) =>
  linear <= 0 ? 0 : linear >= 1 ? 1 : Math.pow(linear, 1 / LED_DISPLAY_GAMMA);

// Brightness rides on opacity and hue on the fill, normalised to full range so
// a dim red still reads as red rather than as near-black.
//
// Opacity comes off the *total* light, not the brightest primary: led_set_hsv
// balances every colour to the same total for a given value, so this keeps what
// is drawn depending on the value rather than on which dies the hue used.
function ledStyle(r, g, b) {
  const sr = r * LED_W_RED / LED_Y_RED;
  const sg = g * LED_W_GREEN / LED_Y_GREEN;
  const sb = b * LED_W_BLUE / LED_Y_BLUE;
  const peak = Math.max(sr, sg, sb);
  if (peak <= 0) return { fill: '#000', level: 0 };

  const light = r * LED_W_RED + g * LED_W_GREEN + b * LED_W_BLUE;
  const ch = (s) => Math.round(encode(s / peak) * 255);
  return {
    fill: `rgb(${ch(sr)},${ch(sg)},${ch(sb)})`,
    level: encode(light / LED_SCALE),
  };
}

const BASE_OPACITY = { lamp: 0.78, halo: 0.6 };

const lastFill = new Array(N_LEDS).fill(null);

export function drawLeds() {
  const rgb = sim.leds();

  for (let i = 0; i < N_LEDS; i++) {
    const nodes = ledNodes.get(i);
    if (!nodes) continue;

    const r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
    // 16 bits per primary now, so the old 8-bit-per-channel key would collide.
    const key = `${r},${g},${b}`;
    if (key === lastFill[i]) continue;  // 40+ setAttribute calls a frame otherwise
    lastFill[i] = key;

    const { fill, level } = ledStyle(r, g, b);
    for (const kind of ['lamp', 'halo']) {
      for (const n of nodes[kind]) {
        for (const st of n.grad.stops) st.setAttribute('stop-color', fill);
        n.node.setAttribute('fill-opacity', (level * BASE_OPACITY[kind] * n.k).toFixed(3));
      }
    }
  }
}
