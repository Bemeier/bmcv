// Headless check that the frontend actually loads and runs.
//
// web/smoke.mjs covers the wasm boundary; this covers everything on top of it.
// It stands up enough of a DOM to import all nine modules for real, against the
// real bmcv.wasm and the real panel.json, then drives a few frames and asserts
// the module responded. That catches the whole class of mistakes a static check
// cannot - a missing export, a module-scope reference to something not defined
// yet, an element id that does not exist in index.html, an import cycle that
// leaves a binding uninitialised - without opening a browser.
//
// It is not a rendering test. Nothing here checks what anything looks like;
// what it proves is that the frontend wires up and talks to the engine.
//
// Run with `just web-check`.

import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));

let failures = 0;
const check = (ok, what) => {
  if (!ok) { failures++; console.error(`FAIL ${what}`); }
  else console.log(`ok   ${what}`);
};

/* ---- a DOM, to the extent these modules use one -------------------------- */

// Deliberately minimal and dumb: every call is recorded or ignored, nothing is
// laid out. If a module reaches for something not here it throws, which is the
// point - the failure names the missing piece.

const listeners = [];

function makeNode(tag = 'div') {
  const node = {
    tagName: tag,
    children: [],
    style: {},
    dataset: {},
    classList: { toggle() {}, add() {}, remove() {} },
    attrs: new Map(),
    textContent: '',
    setAttribute(k, v) { this.attrs.set(k, String(v)); },
    getAttribute(k) { return this.attrs.get(k) ?? null; },
    appendChild(c) { this.children.push(c); c.parentElement = this; return c; },
    insertAdjacentHTML() {},
    addEventListener(type, fn) { listeners.push({ node: this, type, fn }); },
    setPointerCapture() {},
    // A plausible on-screen size, so the canvases size themselves and the
    // drawing code runs rather than bailing out on a zero-width rect.
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 400 }),
    querySelector(sel) { return this.byClass(sel); },
    querySelectorAll(sel) {
      // The one real query the frontend makes is for the channel table's rows,
      // which only exist because innerHTML was assigned - so that assignment
      // has to actually build something. Anything else gets a single stub.
      if (sel.startsWith('tr')) return (this._rows ?? []).filter(r => r.attrs.has('data-ch'));
      const n = this.byClass(sel);
      return n ? [n] : [];
    },
    byClass(sel) {
      // Only ever called with a class or tag selector by these modules. The
      // result is parented on the node it was found in, as a real one would be.
      if (!this._stubs) this._stubs = new Map();
      if (!this._stubs.has(sel)) {
        const n = makeNode(sel);
        n.parentElement = this;
        this._stubs.set(sel, n);
      }
      return this._stubs.get(sel);
    },
    getContext: () => ctx2d,
  };
  node.parentElement = null;

  // Just enough HTML parsing to build the channel table: a row per <tr>, a
  // cell per <td>/<th>, carrying data-ch through. Without this the table's rows
  // would not exist and drawReadouts would never be exercised.
  let html = '';
  Object.defineProperty(node, 'innerHTML', {
    get: () => html,
    set(value) {
      html = value;
      node._rows = [];
      for (const chunk of value.split('</tr>')) {
        const open = chunk.match(/<tr([^>]*)>/);
        if (!open) continue;
        const row = makeNode('tr');
        const id = open[1].match(/data-ch="(\d+)"/);
        if (id) row.attrs.set('data-ch', id[1]);
        row.children = [...chunk.matchAll(/<t[dh][^>]*>/g)].map(() => makeNode('td'));
        node._rows.push(row);
      }
    },
  });

  return node;
}

// Every 2D canvas call the scopes make, doing nothing.
const ctx2d = new Proxy({}, {
  get: (_t, prop) => {
    if (prop === 'font' || prop === 'fillStyle' || prop === 'strokeStyle') return '';
    return () => {};
  },
  set: () => true,
});

const byId = new Map();
const el = id => {
  if (!byId.has(id)) {
    const n = makeNode(id);
    // The canvases need width/height to be real, settable numbers.
    n.width = 0; n.height = 0;
    n.parentElement = makeNode('parent');
    byId.set(id, n);
  }
  return byId.get(id);
};

// The ids index.html actually provides. Asking for one that is not here is a
// failure, because it means the page would hand the module a null.
const PAGE_IDS = [
  'panel', 'hint', 'scope', 'inscope', 'in-overlay', 'params',
  'r-shift', 'r-param', 'r-scene', 'r-bpm', 'status', 'reset', 'reset-fram',
];
const missingIds = [];

globalThis.document = {
  getElementById(id) {
    if (!PAGE_IDS.includes(id) && !id.startsWith('clock-')) missingIds.push(id);
    return el(id);
  },
  createElement: tag => makeNode(tag),
  createElementNS: (_ns, tag) => makeNode(tag),
};

globalThis.window = { devicePixelRatio: 2 };
globalThis.ResizeObserver = class { observe() {} disconnect() {} };
globalThis.requestAnimationFrame = () => 0;

const store = new Map();
globalThis.localStorage = {
  getItem: k => store.get(k) ?? null,
  setItem: (k, v) => store.set(k, v),
  removeItem: k => store.delete(k),
};

// Node has these, but only on the Buffer path in older versions.
globalThis.btoa ??= s => Buffer.from(s, 'binary').toString('base64');
globalThis.atob ??= s => Buffer.from(s, 'base64').toString('binary');

/* ---- serve the module's own files off disk -------------------------------- */

// The wasm build is -sENVIRONMENT=web, so it fetches bmcv.wasm; spec.js fetches
// panel.json. Point fetch at the directory rather than widening the build for
// the sake of a test.
globalThis.fetch = async (url) => {
  const name = String(url).split('/').pop();
  const body = readFileSync(join(here, name));
  return new Response(body, {
    headers: { 'content-type': name.endsWith('.wasm') ? 'application/wasm' : 'application/json' },
  });
};

/* ---- load the frontend ---------------------------------------------------- */

let loadError = null;
try {
  await import('./main.js');
} catch (e) {
  loadError = e;
}

check(!loadError, `every module loads${loadError ? ` (${loadError.message})` : ''}`);
if (loadError) {
  console.error(loadError.stack);
  process.exit(1);
}

check(missingIds.length === 0,
  `every element id exists in index.html${missingIds.length ? ` (missing: ${[...new Set(missingIds)]})` : ''}`);

/* ---- and it is wired to the engine ---------------------------------------- */

const { sim, SHIFT_NAMES } = await import('./sim.js');
const { spec } = await import('./spec.js');
const { drawScopes } = await import('./scope.js');
const { drawLeds } = await import('./leds.js');
const { drawReadouts } = await import('./readouts.js');
const { runTicks } = await import('./inputs.js');

check(spec.buttons.length === 24 && spec.encoders.length === 8, 'the panel spec loaded through fetch');
check(SHIFT_NAMES[0] === 'STA' && SHIFT_NAMES.at(-1) === '---', `shift names came from the firmware (${SHIFT_NAMES.join(',')})`);

// The panel binds a handler to every control. If a selector or an id had gone
// wrong in the split, this is where the count would drop.
check(listeners.length > 100, `panel bound ${listeners.length} event handlers`);

// Drive it the way the frame loop does, and check the engine actually moved.
runTicks(4000);
const outs = sim.outputs();
check(outs.length === 8, 'outputs read back as eight channels');
check(outs.every(v => Math.abs(v) <= 10.001), 'every output is inside +/-10V');

// A channel with amplitude dialled in should be producing something. Turn the
// first encoder with AMP selected, then let it run.
const ampButton = spec.buttons.find(b => b.roles?.param_name === 'AMP');
check(!!ampButton, 'found the AMP button in the spec');
sim.setButton(ampButton.index, 1);
runTicks(200);
sim.setButton(ampButton.index, 0);
runTicks(200);
sim.addEncoder(spec.encoders[0].index, 40);
runTicks(8000);

const moved = sim.scope();
const ch = spec.encoders[0].channel;
let min = Infinity, max = -Infinity;
for (let i = 0; i < 4096; i++) {
  const v = moved[ch * 4096 + i];
  if (v < min) min = v;
  if (v > max) max = v;
}
check(max - min > 0.5, `channel ${ch} is oscillating after the encoder turn (${(max - min).toFixed(2)}Vpp)`);

// The draw path has to survive being called - it reads heap views and the
// canvas stubs, which is where a wrong array length would blow up.
let drawError = null;
try {
  drawLeds();
  drawScopes();
  drawReadouts();
} catch (e) {
  drawError = e;
}
check(!drawError, `the draw path runs${drawError ? ` (${drawError.message})` : ''}`);
if (drawError) console.error(drawError.stack);

// ...and actually wrote something, rather than quietly doing nothing.
check(el('r-shift').textContent === '---', `the mode readout is populated (${el('r-shift').textContent})`);
check(el('r-bpm').textContent !== '', `the bpm readout is populated (${el('r-bpm').textContent})`);

/* ---- persistence round-trips ---------------------------------------------- */

const { persist, restore, forget } = await import('./storage.js');

persist();
check(store.size === 1, 'persist wrote a blob to localStorage');
check(restore(), 'restore accepted its own blob');

// A truncated blob must be refused rather than fed to the module.
store.set([...store.keys()][0], btoa('nonsense'));
check(!restore(), 'a wrong-sized blob is rejected');

forget();
check(store.size === 0, 'forget clears the browser copy');

console.log(failures ? `\n${failures} failed` : '\nall passed');
process.exit(failures ? 1 : 0);
