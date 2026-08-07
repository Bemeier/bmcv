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

// led_fb.c caps brightness at VAL_MED (32) and draws base layers at VAL_LOW
// (8), because real WS2812s are painfully bright above that. Treat VAL_MED as
// full scale and apply a perceptual curve, or everything the renderer actually
// draws is invisible here.
const LED_FULL = 32;
const LED_GAMMA = 0.45;

// Brightness rides on opacity and hue on the fill, normalised to full range so
// a dim red still reads as red rather than as near-black.
function ledStyle(r, g, b) {
  const peak = Math.max(r, g, b);
  if (peak === 0) return { fill: '#000', level: 0 };
  const k = 255 / peak;
  return {
    fill: `rgb(${Math.round(r * k)},${Math.round(g * k)},${Math.round(b * k)})`,
    level: Math.min(1, Math.pow(peak / LED_FULL, LED_GAMMA)),
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
    const key = (r << 16) | (g << 8) | b;
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
