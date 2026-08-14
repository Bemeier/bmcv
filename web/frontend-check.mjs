// Headless check that the frontend actually loads and runs.
//
// web/smoke.mjs covers the wasm boundary; this covers everything on top of it.
// It stands up enough of a DOM to import all ten modules for real, against the
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
    // A real one, backed by a Set. It was three no-ops, which meant a check
    // could set a class and read it back as absent - and the module box shows
    // one half or the other off exactly such a class.
    classList: (() => {
      const set = new Set();
      return {
        add: c => set.add(c),
        remove: c => set.delete(c),
        contains: c => set.has(c),
        toggle: (c, on) => ((on ?? !set.has(c)) ? set.add(c) : set.delete(c)),
      };
    })(),
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
      // The real queries the frontend makes are for table rows, which only
      // exist because innerHTML was assigned - so that assignment has to
      // actually build something. Anything else gets a single stub.
      // Table rows and meters alike: both are repeated blocks that only exist
      // because innerHTML was assigned, and both are what the draw functions
      // walk. Matched by the data-* attribute in the selector, or by class.
      if (sel.startsWith('tr') || sel.startsWith('.')) {
        const attr = sel.match(/\[(data-[\w-]+)\]/)?.[1];
        const cls = sel.match(/^\.([\w-]+)/)?.[1];
        const rows = (this._rows ?? []).filter(r =>
          (!attr || r.attrs.has(attr)) && (!cls || r.classList.contains(cls)));
        if (rows.length || attr || cls) return rows;
      }
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

  // Just enough HTML parsing to build the repeated blocks the draw functions
  // walk: a row per <tr> with a cell per <td>/<th>, and a node per top-level
  // <div class="..." data-...> with its class carried through. Without this
  // they would not exist and the draw code would never be exercised.
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
        for (const [, k, v] of open[1].matchAll(/(data-[\w-]+)="([^"]*)"/g)) row.attrs.set(k, v);
        row.children = [...chunk.matchAll(/<t[dh][^>]*>/g)].map(() => makeNode('td'));
        node._rows.push(row);
      }

      for (const [, attrs] of value.matchAll(/<div\s+([^>]*data-[\w-]+="[^"]*"[^>]*)>/g)) {
        const block = makeNode('div');
        for (const [, k, v] of attrs.matchAll(/(data-[\w-]+)="([^"]*)"/g)) block.attrs.set(k, v);
        const cls = attrs.match(/class="([^"]*)"/)?.[1] ?? '';
        for (const c of cls.split(/\s+/).filter(Boolean)) block.classList.add(c);
        node._rows.push(block);
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

// The ids index.html actually provides, read out of the page rather than
// listed here: a hand-kept copy of the list is one more thing that can drift,
// and drifting the other way - an id in the list that the page no longer has -
// is exactly what this check exists to catch. Asking for an id the page does
// not provide is a failure, because the module would be handed a null.
const PAGE_IDS = [...readFileSync(new URL('index.html', import.meta.url), 'utf8')
  .matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
const missingIds = [];

globalThis.document = {
  getElementById(id) {
    if (!PAGE_IDS.includes(id) && !id.startsWith('clock-')) missingIds.push(id);
    return el(id);
  },
  createElement: tag => makeNode(tag),
  createElementNS: (_ns, tag) => makeNode(tag),

  // A real page always has one, and mode.js puts a class on it to say a
  // physical module is driving the panel. Without it here, importing the
  // frontend throws on the first connect rather than at load - which is exactly
  // the kind of thing this file exists to catch before a browser does.
  body: makeNode('body'),

  // The probe stops polling while the tab is hidden, so it listens for that at
  // construction. Nothing here fires the event; what is checked is that
  // subscribing to it does not throw.
  addEventListener() {},
  hidden: false,
};

// The probe hands the device back on pagehide, so it subscribes at load. As
// with document's, nothing here fires the event - what is checked is that
// subscribing does not throw.
globalThis.window = { devicePixelRatio: 2, addEventListener() {} };
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

const { sim, SHAPE_NAMES, SHIFT_NAMES } = await import('./sim.js');
const { spec } = await import('./spec.js');
const { drawScopes } = await import('./scope.js');
const { drawLeds } = await import('./leds.js');
const { drawReadouts } = await import('./readouts.js');
const { runTicks } = await import('./inputs.js');
const { drawMidi, pumpMidi } = await import('./midi.js');

check(spec.buttons.length === 24 && spec.encoders.length === 8, 'the panel spec loaded through fetch');

/* ---- the scopes show a span of time, not a count of samples --------------- */

// The buffer is filled by the engine at 4kHz and by a debug probe at ~30Hz, and
// a cell that draws a fixed number of samples therefore shows two spans that
// differ by a factor of a hundred - which is what it used to do, silently.
{
  const { mode } = await import('./mode.js');
  const { drawScopes, spanSamples } = await import('./scope.js');

  mode.drivenBy(null);
  check(spanSamples() === 1500, `the simulator draws 375ms of ticks (${spanSamples()} samples)`);

  mode.drivenBy(30);
  check(spanSamples() === 60, `a probe at 30/s draws two seconds (${spanSamples()} samples)`);

  // The rate is measured, so the window has to follow it rather than a constant.
  mode.drivenBy(10);
  check(spanSamples() === 20, `and follows the measured rate (${spanSamples()} samples)`);

  mode.drivenBy(null);
  check(spanSamples() === 1500, 'and disconnecting puts the span back');
  drawScopes();

  // The probe hands its sample count over as it disconnects, and the simulator
  // has no gaps to account for - so the count has to be discarded, not carried.
  // Carried, it capped how much of the buffer the scopes would draw: after a
  // session of ~900 snapshots the simulator drew 900 of its 1500 samples and
  // the trace stopped 60% of the way across every cell.
  mode.drivenBy(30, 900);
  check(mode.contiguous === 900, 'a live probe reports its own sample count');
  mode.drivenBy(null, 900);
  check(mode.contiguous === Infinity, 'and disconnecting discards it, whatever is passed');

  // The aliasing marker is drawn straight onto the canvas, so what is checked
  // here is that asking the question does not throw with a probe's sample rate
  // in effect - the branch only runs when live, and only there reads effective
  // frequencies out of the wasm heap.
  mode.drivenBy(30);
  let aliasThrew = null;
  try { drawScopes(); } catch (e) { aliasThrew = e; }
  check(!aliasThrew, `the aliasing check runs${aliasThrew ? ` (${aliasThrew.message})` : ''}`);

  // Coming back to a backgrounded tab: the buffer still holds whatever was in
  // it, but only a few samples were taken since. Drawing the rest would put
  // history from minutes ago on an axis that claims to be two seconds wide.
  mode.drivenBy(30, 3);
  check(mode.contiguous === 3, 'a resumed probe reports how little it has sampled');
  let gapThrew = null;
  try { drawScopes(); } catch (e) { gapThrew = e; }
  check(!gapThrew, `and the scopes draw only that${gapThrew ? ` (${gapThrew.message})` : ''}`);

  mode.drivenBy(null);
  check(mode.contiguous === Infinity, 'the simulator has no such gap');
}

/* ---- the page reserves its layout before anything loads ------------------- */

// The panel SVG has no size until panel.json has been fetched and a viewBox set
// from it, and the scope canvases hold their attribute size until the first
// resize callback. Without a declared ratio on each, the page lays itself out
// once at nothing and again a moment later, and everything below jumps.
{
  const css = readFileSync(new URL('style.css', import.meta.url), 'utf8');
  for (const sel of ['#panel', '#scope', '.scope-stack canvas']) {
    const rule = css.slice(css.indexOf(sel + ' {'), css.indexOf('}', css.indexOf(sel + ' {')));
    check(/aspect-ratio:/.test(rule), `${sel} reserves its height`);
  }
}

/* ---- the scope columns keep the 2:1 that makes cells match ---------------- */

// The output grid is 4 cells wide and the input grid 2, over the same height -
// so a cell is the same rectangle in either only while the output column is
// exactly twice the input column. That ratio moved from the canvases to the
// columns when the bracket labels went in, and it is the kind of thing that
// survives a refactor by luck.
{
  const css = readFileSync(new URL('style.css', import.meta.url), 'utf8');
  check(/\.scope-col:first-child\s*\{\s*flex:\s*2 1 0/.test(css), 'the output column is 2fr');
  check(/\.scope-col:last-child\s*\{\s*flex:\s*1 1 0/.test(css), 'the input column is 1fr');
}

/* ---- the MIDI table appears only with a port ------------------------------ */

// Twelve rows of 64 is what an idle CC table looks like, and it is also what a
// CC table with nowhere to send looks like. The class is what tells them apart.
{
  const midi = document.getElementById('midi');
  check(!midi.classList.contains('sending'), 'the CC table is hidden with no port selected');
}

/* ---- the two halves of the module box are exclusive ----------------------- */

// MIDI and the probe rates share one box and are shown one at a time, off the
// body class mode.js sets. The reset buttons stay visible but stop working,
// because a page showing hardware would reset a simulation it is not running.
{
  const { mode } = await import('./mode.js');
  const cls = document.body.classList;
  const reset = document.getElementById('reset');
  const resetFram = document.getElementById('reset-fram');

  check(!cls.contains('live'), 'the simulator is the default source');
  check(!reset.disabled && !resetFram.disabled, 'and both resets work');

  mode.drivenBy(30);
  check(cls.contains('live'), 'a connected module marks the page live');
  check(reset.disabled && resetFram.disabled, 'and takes the resets out of service');

  // The rest of it is CSS, and the bug was CSS: an id selector outranked
  // `button:disabled:hover`, so Reset FRAM went on lighting up amber while it
  // was refusing to do anything. Every hover rule has to exclude disabled
  // buttons itself, which is a thing about the stylesheet rather than the DOM.
  const css = readFileSync(new URL('style.css', import.meta.url), 'utf8');
  const bareHovers = [...css.matchAll(/^\s*([^{\n]*button[^{\n]*:hover[^{\n]*)\{/gm)]
    .map(m => m[1].trim())
    .filter(sel => !sel.includes(':not(:disabled)'));
  check(bareHovers.length === 0,
    `every button hover rule excludes disabled${bareHovers.length ? ` (${bareHovers.join(' / ')})` : ''}`);

  mode.drivenBy(null);
  check(!cls.contains('live'), 'disconnecting hands the page back');
  check(!reset.disabled && !resetFram.disabled, 'and the resets with it');
}

/* ---- the rate readouts are wired ----------------------------------------- */

// Three of the four come out of the wasm, and an export missing from
// sim/CMakeLists.txt is undefined here rather than an error - so it only shows
// up when something calls it. This calls it.
{
  const { drawProbeRates } = await import('./probe/ui.js');
  runTicks(400);
  drawProbeRates();

  const val = id => document.getElementById(id).textContent;
  check(val('r-engine-fps') !== '—', `the engine rate reaches the page (${val('r-engine-fps')})`);

  // engine_fps is measured in engine.c and so exists in every host; the other
  // two measure peripherals only the board has - the DAC service pass and the
  // WS2811 flush. Blank is the correct reading here, and a number would mean
  // something had started inventing one.
  check(val('r-dac-fps') === '—', 'the dac rate is blank without a DAC loop to measure');
  check(val('r-led-fps') === '—', 'as is the led rate without an LED driver to flush to');
  check(val('r-probe-hz') === '—', 'and the probe rate with nothing connected');
}

/* ---- no transfer can hang the connection --------------------------------- */

// WebUSB transfers have no timeout of their own. A probe left holding an unread
// reply from a previous page answers the next session's first command with the
// wrong data and the one after that with nothing - and the page sat in
// "connecting" forever, because nothing was ever going to reject.
{
  const src = readFileSync(new URL('probe/stlink.js', import.meta.url), 'utf8');

  const raw = [...src.matchAll(/await\s+(?:this\.)?(?:device\.)?transfer(?:In|Out)\(/g)];
  check(raw.length === 0, `every transfer is wrapped${raw.length ? ` (${raw.length} bare)` : ''}`);
  check(/device\.reset\(\)/.test(src), 'and a failed connection retries with a port reset');

  // The endpoint toggles are what a refresh leaves out of step, and clearHalt
  // is the only thing short of re-enumeration that puts them back - so losing
  // this call turns "refresh the page" back into "unplug the probe".
  check(/clearHalt\(/.test(src), 'and connecting resynchronises both pipes');
  // Connecting drains whatever the last session abandoned, and that means
  // reading an endpoint that may be empty. Such a read never completes and
  // cannot be cancelled, so left pending it swallows this session's first real
  // reply - which broke every healthy first connect once already. The only
  // thing that cancels it is closing the device, so the deadline branch has to
  // do exactly that.
  check(/#drain\b/.test(src), 'and drains what the last session abandoned');
  check(/=== QUIET[\s\S]{0,240}#reopen\(\)/.test(src),
    'and a read that finds nothing is cancelled by dropping the handle');
}

/* ---- the poll loop yields without a clamped timer ------------------------- */

// The loop's pacing is a MessageChannel message rather than a timer, because
// nested setTimeout is clamped to 4ms and that was a sixth of the budget. What
// is checked here is that the mechanism exists and resolves - if it silently
// never fired, polling would stop dead after the first snapshot.
{
  check(typeof MessageChannel === 'function', 'MessageChannel is available to pace the loop');

  const ch = new MessageChannel();
  const fired = await new Promise(resolve => {
    ch.port1.onmessage = () => resolve(true);
    ch.port2.postMessage(0);
    setTimeout(() => resolve(false), 100);
  });
  check(fired, 'and a message posted to it comes back');
}

/* ---- the probe descriptor decodes ---------------------------------------- */

// These 28 bytes are what `arm-none-eabi-objdump -s -j .probe_info` prints for
// a real build - the firmware saying where its state lives, which is the first
// thing the browser reads off a module. Frozen here rather than read from the
// ELF so the check runs on a fresh checkout, where no firmware has been built.
//
// Worth testing because a wrong offset here does not fail: it yields a
// plausible-looking address, reads 2368 bytes of whatever is there, and shows a
// module made of noise.
{
  const { decodeInfo } = await import('./probe/probe.js');

  const bytes = new Uint8Array([
    0x42, 0x4d, 0x43, 0x56, // "BMCV"
    0x01, 0x00,             // descriptor version 1
    0x40, 0x09,             // instance size, 0x0940 = 2368
    0x70, 0x0f, 0x00, 0x20, // &bmcv = 0x20000f70
    0x30, 0x2e, 0x31, 0x30, 0x2e, 0x30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // "0.10.0"
  ]);

  const info = decodeInfo(bytes);
  check(info.instanceAddr === 0x20000f70, `reads the instance address (0x${info.instanceAddr.toString(16)})`);
  check(info.instanceSize === 2368, `reads the instance size (${info.instanceSize})`);
  check(info.version === '0.10.0', `reads the firmware version (${info.version})`);

  // The size the descriptor reports is checked against this build's before a
  // byte of state is believed, so it has to be the same number the wasm knows.
  check(info.instanceSize === sim.instanceSize, 'and it matches what this build decodes');

  const refuses = (mutate, what) => {
    const bad = bytes.slice();
    mutate(bad);
    let threw = false;
    try { decodeInfo(bad); } catch { threw = true; }
    check(threw, what);
  };
  refuses(b => { b[0] = 0x41; }, 'a wrong magic is refused, not decoded');
  refuses(b => { b[4] = 2; }, 'an unknown descriptor version is refused');
}

/* ---- the remote input mailbox -------------------------------------------- */

// The write direction: the bytes this page pushes into a module's RAM so its
// input layer merges them with the panel someone may have their hands on.
//
// Worth checking without hardware because none of it fails loudly. The offset
// is a plain number added to an address, so a wrong one writes a valid-looking
// mailbox over whatever else is there; and the sequence number is what the far
// end reads as a heartbeat, so one that stopped moving would look like a page
// that had gone away rather than like a bug here.
{
  check(sim.remoteSize > 0 && sim.remoteSize % 4 === 0, `the mailbox is ${sim.remoteSize} bytes, a whole number of words`);
  check(sim.remoteOffset % 4 === 0, `it starts on a word (offset ${sim.remoteOffset})`);
  check(sim.remoteOffset + sim.remoteSize <= sim.instanceSize, 'and lies inside the instance');

  const seqOf = b => new DataView(b.buffer, b.byteOffset, b.byteLength).getUint32(b.length - 4, true);

  sim.remoteClear();
  const first = sim.remoteBlob();
  check(first.length === sim.remoteSize, 'the blob is the size it declares');

  // Every call, not only the ones that changed something - probe.js writes on
  // every poll precisely so the far end keeps hearing from this page.
  const second = sim.remoteBlob();
  check(seqOf(second) !== seqOf(first), `each blob carries a fresh sequence number (${seqOf(first)} -> ${seqOf(second)})`);
  check(seqOf(first) !== 0 && seqOf(second) !== 0, 'and never zero, which means never written');

  // The panel's gestures have to reach it. A button is the one field that is a
  // plain level, so it shows up in the bytes exactly where the struct says.
  const b = spec.buttons[0].index;
  sim.remoteButton(b, 1);
  const held = sim.remoteBlob();
  check(held[b] === 1, `a remote press lands in the blob (button ${b})`);

  sim.remoteButton(b, 0);
  check(sim.remoteBlob()[b] === 0, 'and releasing it clears the byte');

  // Clearing is what connect and disconnect do, and it must not restart the
  // count - a repeated sequence number reads as no update at all.
  sim.remoteButton(b, 1);
  const before = seqOf(sim.remoteBlob());
  sim.remoteClear();
  const cleared = sim.remoteBlob();
  check(cleared[b] === 0, 'clearing lets go of everything');
  check(seqOf(cleared) > before, 'and is itself an update, not a silence');

  // The two-transfer split probe.js does: fields first, sequence number after,
  // so a mailbox that half-arrives is not acted on until it is whole.
  const blob = sim.remoteBlob();
  check(blob.subarray(0, blob.length - 4).length === sim.remoteSize - 4, 'the body is everything but the last word');
  check(blob.subarray(blob.length - 4).length === 4, 'and the sequence number is that word');
}

/* ---- the seven-bit codec, for the MIDI transport ------------------------- */

// The firmware and this page have to agree about this exactly, and they do it
// by being the same compiled source - sysex.c, in the wasm. What is checked
// here is that the binding reaches it and that nothing is lost on the way
// through, since a codec that is subtly wrong produces a module made of noise
// rather than an error.
{
  check(sim.sysex7EncodedLen(48) === 55, 'the remote input mailbox encodes to 55 bytes');
  check(sim.sysex7EncodedLen(sim.instanceSize) === 2707, `an instance encodes to ${sim.sysex7EncodedLen(sim.instanceSize)} bytes`);
  check(sim.sysex7DecodedLen(sim.sysex7EncodedLen(sim.instanceSize)) === sim.instanceSize, 'and decodes back to its own size');

  // Every byte value, including the high-bit ones that are the entire reason
  // this encoding exists.
  const raw = new Uint8Array(256).map((_, i) => i);
  const enc = sim.sysex7Encode(raw);
  check(enc.length === sim.sysex7EncodedLen(256), 'encoding 256 bytes gives the declared length');
  check(enc.every(b => b < 0x80), 'nothing encoded has its high bit set');

  const back = sim.sysex7Decode(enc);
  check(back.length === 256 && back.every((b, i) => b === i), 'and it round-trips byte for byte');

  // A whole snapshot through the same path a module's would take.
  const snapshot = sim.exportInstance();
  const wire = sim.sysex7Encode(snapshot);
  const recovered = sim.sysex7Decode(wire);
  check(recovered.length === snapshot.length, 'a snapshot survives the encoding at full size');
  check(recovered.every((b, i) => b === snapshot[i]), 'and byte for byte');
  check(sim.importInstance(recovered), 'and is still an instance this build accepts');
}
check(SHIFT_NAMES[0] === 'STA' && SHIFT_NAMES.at(-1) === '---', `shift names came from the firmware (${SHIFT_NAMES.join(',')})`);
check(SHAPE_NAMES.length > 1 && SHAPE_NAMES[0] === 'LFO', `shape names came from the firmware (${SHAPE_NAMES.join(',')})`);

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
  pumpMidi();
  drawMidi();
} catch (e) {
  drawError = e;
}
check(!drawError, `the draw path runs${drawError ? ` (${drawError.message})` : ''}`);
if (drawError) console.error(drawError.stack);

// ...and actually wrote something, rather than quietly doing nothing.
check(el('r-shift').textContent === '---', `the mode readout is populated (${el('r-shift').textContent})`);
check(el('r-bpm').textContent !== '', `the bpm readout is populated (${el('r-bpm').textContent})`);

// The MIDI meters are fed by draining the module's own queue, so a meter that
// has moved proves the whole path: midi_out published, the wasm boundary handed
// the bytes over, and the frontend parsed them back into a CC value.
const midiMeters = el('midi-cc')._rows.filter(r => r.attrs.has('data-cc'));
check(midiMeters.length === 12, `the midi readout has 12 meters (got ${midiMeters.length})`);

// Every one of them, not merely some: the first publish slot states the whole
// block, so a meter still marked silent means a CC never crossed the boundary.
const silent = midiMeters.filter(m => m.classList.contains('silent')).length;
check(silent === 0, `all 12 midi values reached the readout (${silent} still silent)`);

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
