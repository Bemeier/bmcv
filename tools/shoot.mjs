// The screenshots in docs/images/ and web/manual/, taken from the simulator
// itself rather than by hand.
//
// They were hand-captured until the LED palette moved and every one of them
// quietly became a picture of colours the module no longer has. A screenshot is
// generated output like any other file under tools/ - the state it shows is the
// interesting part, and that belongs in a script where it can be read and
// re-run, not in someone's browser at one moment.
//
// No dependencies, deliberately, like the rest of web/ and scripts/: Chrome's
// DevTools Protocol is a WebSocket, and node has had one built in since 22.
// Needs a Chrome or Chromium - pass it in CHROME, and under WSL that can be the
// Windows one, though a Linux build avoids a round trip through wslpath.
//
//   node tools/shoot.mjs --url=http://localhost:8123 [--only=panel]
import { spawn } from 'node:child_process';
import { mkdtemp, readFile, writeFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

const arg = (name, fallback) => {
  const hit = process.argv.slice(2).find(a => a.startsWith(`--${name}=`));
  return hit ? hit.slice(name.length + 3) : fallback;
};

const URL_BASE = arg('url', 'http://localhost:8123');
const CHROME = process.env.CHROME || 'chromium';
const ONLY = arg('only', '');

// The backdrop is a photograph behind the panel artwork, which it shows
// through: a flat screenshot of it comes out as a heavy photographic one. Every
// shot of the panel hides it first, which is what the note in manual/index.html
// used to ask a human to do by hand.
const HIDE_BACKDROP = `document.head.insertAdjacentHTML('beforeend',
  '<style>body::before{display:none!important}</style>')`;

// Where a panel element is on screen, from the same panel.json the page draws
// itself from - so a moved control moves the click with it. mm are board
// millimetres; spec.px puts them in the SVG's own viewBox.
const POINT_FN = `async (mm) => {
  const { px } = await import('./spec.js');
  const svg = document.getElementById('panel');
  const p = svg.createSVGPoint();
  const [x, y] = px(mm);
  p.x = x; p.y = y;
  const s = p.matrixTransform(svg.getScreenCTM());
  return { x: s.x, y: s.y };
}`;

/* ---- CDP ---------------------------------------------------------------- */

async function launch(width, height) {
  const dir = await mkdtemp(join(tmpdir(), 'bmcv-shot-'));
  const proc = spawn(CHROME, [
    '--headless=new', '--disable-gpu', '--hide-scrollbars', '--mute-audio',
    '--no-first-run', '--no-default-browser-check', '--disable-extensions',
    `--user-data-dir=${dir}`, `--window-size=${width},${height}`,
    '--remote-debugging-port=0', 'about:blank',
  ], { stdio: 'ignore' });

  // Chrome writes the port it settled on into the profile once it is listening,
  // which is the only race-free way to ask for "any free port".
  let port = null;
  for (let i = 0; i < 100 && port === null; i++) {
    await new Promise(r => setTimeout(r, 100));
    try {
      port = (await readFile(join(dir, 'DevToolsActivePort'), 'utf8')).split('\n')[0].trim();
    } catch { /* not up yet */ }
  }
  if (!port) throw new Error(`${CHROME} did not start - is CHROME set to a real browser?`);

  const targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  const page = targets.find(t => t.type === 'page');
  return { proc, dir, wsUrl: page.webSocketDebuggerUrl };
}

function connect(wsUrl) {
  const ws = new WebSocket(wsUrl);
  const pending = new Map();
  const waiters = [];
  let nextId = 1;

  ws.addEventListener('message', ev => {
    const msg = JSON.parse(ev.data);
    if (msg.id && pending.has(msg.id)) {
      const { resolve, reject } = pending.get(msg.id);
      pending.delete(msg.id);
      msg.error ? reject(new Error(msg.error.message)) : resolve(msg.result);
    }
    for (let i = waiters.length - 1; i >= 0; i--) {
      if (waiters[i].method === msg.method) waiters.splice(i, 1)[0].resolve(msg.params);
    }
  });

  const open = new Promise((resolve, reject) => {
    ws.addEventListener('open', resolve);
    ws.addEventListener('error', () => reject(new Error('could not attach to the browser')));
  });

  const send = (method, params = {}) => new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject });
    ws.send(JSON.stringify({ id, method, params }));
  });

  const until = method => new Promise(resolve => waiters.push({ method, resolve }));

  return { ws, open, send, until };
}

/* ---- the page ----------------------------------------------------------- */

const sleep = ms => new Promise(r => setTimeout(r, ms));

async function evaluate(cdp, expression) {
  const r = await cdp.send('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true });
  if (r.exceptionDetails) throw new Error(r.exceptionDetails.exception?.description ?? 'page threw');
  return r.result.value;
}

// Real input events through the browser, not synthetic ones dispatched at a
// node: the panel's handlers take a pointer capture, and a PointerEvent made in
// page script carries no live pointer id to capture.
async function click(cdp, page, mm, holdMs = 60) {
  const p = await evaluate(cdp, `(${POINT_FN})(${JSON.stringify(mm)})`);
  const base = { x: p.x, y: p.y, button: 'left', clickCount: 1, buttons: 1, pointerType: 'mouse' };
  await cdp.send('Input.dispatchMouseEvent', { type: 'mouseMoved', ...base, buttons: 0 });
  await cdp.send('Input.dispatchMouseEvent', { type: 'mousePressed', ...base });
  await sleep(holdMs);
  await cdp.send('Input.dispatchMouseEvent', { type: 'mouseReleased', ...base, buttons: 0 });
  await sleep(40);
}

async function wheel(cdp, page, mm, notches) {
  const p = await evaluate(cdp, `(${POINT_FN})(${JSON.stringify(mm)})`);
  const step = notches > 0 ? -100 : 100; // wheel up is negative deltaY, and is clockwise
  for (let i = 0; i < Math.abs(notches); i++) {
    await cdp.send('Input.dispatchMouseEvent', {
      type: 'mouseWheel', x: p.x, y: p.y, deltaX: 0, deltaY: step, pointerType: 'mouse',
    });
    await sleep(6);
  }
}

// Always clipped, even for a whole-page shot, where the clip is the viewport.
// captureBeyondViewport otherwise returns the entire scroll height, which is
// how the overview came out 1261px tall instead of the 1000 it was framed for.
async function capture(cdp, selector, out, scale, width, height) {
  const clip = selector
    ? {
        ...(await evaluate(cdp, `(() => {
          const el = document.querySelector(${JSON.stringify(selector)});
          const b = el.getBoundingClientRect();
          return { x: b.x, y: b.y, width: b.width, height: b.height };
        })()`)),
        scale,
      }
    : { x: 0, y: 0, width, height, scale };

  const shot = await cdp.send('Page.captureScreenshot', {
    format: 'png', captureBeyondViewport: true, clip,
  });
  await writeFile(out, Buffer.from(shot.data, 'base64'));
  console.log(`wrote ${out}`);
}

/* ---- the shots ---------------------------------------------------------- */

const CTRL = { STA: [11.25, 92.5], SYS: [22.75, 92.5], QNT: [34.25, 92.5], MIX: [45.75, 92.5], SAV: [57.25, 92.5], STB: [68.75, 92.5] };
const ENC = [[7.5, 38.0], [25.5, 38.0], [54.5, 38.0], [72.5, 38.0], [7.5, 56.0], [25.5, 56.0], [54.5, 56.0], [72.5, 56.0]];

// A spread of offsets across the row, so every ring sits at a different level
// and the picture shows the whole voltage ramp at once: the warm shift at 10V,
// full red and green either side of 5V, and the dark crossing in the middle.
// Static rather than a running LFO, which is what makes the shot the same every
// time it is taken - no clock, so no phase to land on. UI_EDIT_DISPLAY then has
// to expire before the shot, or the rings would still be showing the parameter
// rather than what it did.
//
// One notch per wheel event, whatever the delta - see the handler in panel.js -
// and 256 parameter units per notch, so 128 of them is the full 10V.
const OFFSET_NOTCHES = [-128, -64, -26, -8, 8, 26, 64, 128];

// Amplitudes for the overview, in the same notches: a spread of a few volts so
// the eight scopes are visibly different sizes rather than one wave drawn
// eight times.
const AMP_NOTCHES = [55, 48, 62, 69, 41, 34, 83, 60];

// A clock on the first input, every channel at a ratio of it, and a spread of
// amplitudes to draw. What the two shots of a *running* module both need.
const livePatch = async (cdp, page) => {
  await evaluate(cdp, `document.querySelector('[data-clock-on="0"]').click()`);
  await click(cdp, page, CTRL.STA); // FRQ
  for (const e of ENC) await wheel(cdp, page, e, 1); // one ratio up the grid
  await click(cdp, page, CTRL.SAV); // AMP
  for (let i = 0; i < ENC.length; i++) await wheel(cdp, page, ENC[i], AMP_NOTCHES[i]);
  await click(cdp, page, CTRL.STA); // back to FRQ, as the old shots had it
  await sleep(2500); // and let the rings fall back to what they are putting out
};

// Caught with the channels swinging positive, so the rings read as a module
// putting something out rather than as a row of warnings, and the traces are
// caught mid-rise. The last column of the readout is the live output voltage,
// which is the page's own measurement of exactly that.
const SWINGING_POSITIVE = `parseFloat(document.querySelector('tr[data-ch="0"]').lastElementChild.textContent) > 2.5`;

const SHOTS = {
  panel: {
    // The module as it boots: nothing patched, rings dark, the parameter row
    // lit in the six colours that name it.
    out: 'web/manual/panel.png',
    selector: '#panel-wrap',
    scale: 2,
    setup: async () => {},
  },
  'led-language': {
    // The same panel with something to show: every ring at its own level.
    out: 'docs/images/led-language.png',
    selector: '#panel-wrap',
    scale: 2,
    setup: async (cdp, page) => {
      await click(cdp, page, CTRL.STB); // OFS
      for (let i = 0; i < ENC.length; i++) await wheel(cdp, page, ENC[i], OFFSET_NOTCHES[i]);
      await click(cdp, page, CTRL.STA); // leave FRQ selected, as the old shot had it
      await sleep(2500); // and let the parameter display decay back to the levels
    },
  },
  'web-overview': {
    // The whole frontend, viewport-sized, with something actually running -
    // a clock on the first input, every channel at a ratio of it and an
    // amplitude to draw. The backdrop stays: this one is a picture of the page,
    // not of the panel.
    //
    // The only shot that is not identical run to run: the scopes are drawing a
    // live oscillator, so the phase they are caught at is whatever it is. That
    // is the picture - a still of a running module.
    out: 'docs/images/web-overview.png',
    selector: null,
    scale: 1,
    width: 1500,
    height: 1000,
    keepBackdrop: true,
    readyWhen: SWINGING_POSITIVE,
    setup: livePatch,
  },

  scopes: {
    // The eight output traces alone, for the manual. Same patch as the
    // overview: the point of the picture is eight channels at one ratio and
    // different amplitudes, which is what the paragraph beside it says.
    out: 'web/manual/scopes.png',
    selector: '#scope',
    scale: 2,
    width: 1500,
    height: 1000,
    // The backdrop goes, as it does for the panel: the scope cells are part
    // transparent and the photograph reads straight through them once the shot
    // is cropped away from the page around it.
    readyWhen: SWINGING_POSITIVE,
    setup: livePatch,
  },
};

const shots = Object.entries(SHOTS).filter(([name]) => !ONLY || ONLY === name);

for (const [name, shot] of shots) {
  const width = shot.width ?? 900;
  const height = shot.height ?? 1300;
  const { proc, dir, wsUrl } = await launch(width, height);
  const cdp = connect(wsUrl);
  await cdp.open;

  await cdp.send('Page.enable');
  await cdp.send('Runtime.enable');
  const loaded = cdp.until('Page.loadEventFired');
  await cdp.send('Page.navigate', { url: URL_BASE + '/' });
  await loaded;
  await sleep(2500); // the wasm, and the first frames it draws

  if (!shot.keepBackdrop) await evaluate(cdp, HIDE_BACKDROP);
  await shot.setup(cdp, null);

  // Park the pointer off the panel before the shutter. Whatever was clicked
  // last is still under the cursor otherwise, and the page answers a hover with
  // an explanation of that control - which is a fine thing for the page to do
  // and not a thing a screenshot of the module should contain.
  await cdp.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: 2, y: 2, buttons: 0, pointerType: 'mouse' });
  await sleep(200);

  // A shot of something moving has to say when to press the shutter, or it gets
  // whatever phase the oscillator happened to be at - which for eight channels
  // sharing one ratio means a whole panel of negative levels as easily as a
  // whole panel of positive ones.
  if (shot.readyWhen) {
    for (let i = 0; i < 200; i++) {
      if (await evaluate(cdp, shot.readyWhen)) break;
      await sleep(25);
    }
  }

  await capture(cdp, shot.selector, shot.out, shot.scale, width, height);

  cdp.ws.close();
  proc.kill();
  // Chrome is still tidying its profile as it goes, so the directory can be
  // half gone by the time we get to it. Not worth failing a screenshot over.
  await sleep(300);
  await rm(dir, { recursive: true, force: true }).catch(() => {});
}
