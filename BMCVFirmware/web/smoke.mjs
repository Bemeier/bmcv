// Headless check that the wasm module loads and its exports behave.
//
// The browser frontend is not testable here, but everything it depends on is:
// that the export list in sim/CMakeLists.txt is complete, that heap views line
// up with the C structs, and that driving the module through the flat API
// actually moves the engine. Run with `just wasm-check`.

import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));

let failures = 0;
const check = (ok, what) => {
  if (!ok) { failures++; console.error(`FAIL ${what}`); }
  else console.log(`ok   ${what}`);
};

// The build is -sENVIRONMENT=web, so the module only knows how to fetch its
// wasm over HTTP - and node's fetch will not do file:// URLs. Rather than
// widening the build for the sake of a test, hand it an already-instantiated
// module through the instantiateWasm hook. The web path stays untouched.
const wasmBinary = readFileSync(join(here, 'bmcv.wasm'));
const createBmcv = (await import('./bmcv.js')).default;
const Module = await createBmcv({
  instantiateWasm(imports, receive) {
    WebAssembly.instantiate(wasmBinary, imports).then(r => receive(r.instance));
    return {};
  },
});

/* ---- the export list is complete ---------------------------------------- */

const NAMES = [
  'create', 'destroy', 'reset', 'set_button', 'add_encoder', 'set_slider01',
  'set_cv', 'fire_gate', 'run', 'now_us', 'outputs_v', 'leds_rgb', 'scope',
  'scope_head', 'shift_state', 'selected_param', 'active_scene', 'bpm',
  'have_beat', 'scene_contribution', 'channel_muted', 'error_flags',
  'channel_param', 'storage_size', 'storage_get', 'storage_set',
  'input_scope', 'effective', 'mode_name', 'mode_count', 'engine_fps', 'dac_fps',
];
const missing = NAMES.filter(n => typeof Module[`_bmcv_sim_${n}`] !== 'function');
check(missing.length === 0, `all ${NAMES.length} exports present${missing.length ? ` (missing: ${missing})` : ''}`);
if (missing.length) process.exit(1);

const F = Object.fromEntries(NAMES.map(n => [n, Module[`_bmcv_sim_${n}`]]));

const f32 = (ptr, len) => new Float32Array(Module.HEAPF32.buffer, ptr, len);
const u8 = (ptr, len) => new Uint8Array(Module.HEAPU8.buffer, ptr, len);

/* ---- a module boots and runs -------------------------------------------- */

const sim = F.create();
check(sim !== 0, 'created an instance');

// A module that has never been used is not in an error state: no stored config
// is its normal condition, and the defaults have already been validated.
check(F.error_flags(sim) === 0, 'a fresh module boots clean, with no error display');
check(F.shift_state(sim) === 9, 'boots with no shift mode');

// The frontend builds its shift-mode labels from these, so an export that
// returned a null or an empty string would show up as blank UI, not an error.
check(F.mode_count() === 10, 'ten shift modes, including "none"');
const modeNames = Array.from({ length: F.mode_count() }, (_, i) => Module.UTF8ToString(F.mode_name(i)));
check(modeNames[0] === 'STA' && modeNames[9] === '---', `mode names come from the firmware table (${modeNames.join(',')})`);

F.set_slider01(sim, 0);
F.run(sim, 250, 400); // 100ms
check(F.now_us(sim) === 100000, 'time advances by dt * ticks');

/* ---- run() cannot be made to hang -------------------------------------- */

// A negative count used to arrive as ~4.29 billion unsigned ticks and lock the
// main thread solid - a frozen browser tab with no error anywhere.
{
  const before = F.now_us(sim);
  F.run(sim, 250, -20);
  check(F.now_us(sim) === before, 'a negative tick count is a no-op, not 4.29 billion ticks');
  F.run(sim, 250, 0);
  check(F.now_us(sim) === before, 'a zero tick count is a no-op');
  F.run(sim, -250, 10);
  check(F.now_us(sim) === before, 'a negative dt is a no-op');

  // Absurdly large counts are capped rather than run.
  const t0 = Date.now();
  F.run(sim, 250, 2_000_000_000);
  check(Date.now() - t0 < 5000, `an absurd tick count is capped (took ${Date.now() - t0}ms)`);
  F.reset(sim, 1);
  F.set_slider01(sim, 0);
}

/* ---- the panel spec agrees with the module ------------------------------ */

const spec = JSON.parse(readFileSync(join(here, 'panel.json'), 'utf8'));
check(spec.leds.length === 21 && spec.buttons.length === 24 && spec.encoders.length === 8,
  'panel spec has 24 buttons, 21 leds, 8 encoders');

const leds = u8(F.leds_rgb(sim), 21 * 3);
check(leds.length === 21 * 3, 'led framebuffer view is 63 bytes');

/* ---- driving it changes what comes out ---------------------------------- */

// Tap the ctrl button that selects AMP (ctrl ids 0..5 = FRQ SHP MOD PHS AMP OFS),
// then wind channel 0's encoder up and check the output starts moving.
const sav = spec.buttons.find(b => b.roles.ctrl_name === 'SAV');
const ch0 = spec.encoders.find(e => e.channel === 0);
check(!!sav && !!ch0, 'found the SAV button and channel 0 in the spec');

F.set_button(sim, sav.index, 1);
F.run(sim, 250, 200);  // 50ms - a tap, under UI_T_HOLD
F.set_button(sim, sav.index, 0);
F.run(sim, 250, 200);
check(F.selected_param(sim) === 4, 'tapping SAV selects AMP');

F.add_encoder(sim, ch0.index, 400);
F.run(sim, 250, 4000); // 1s

const outs = f32(F.outputs_v(sim), 8);
check(Math.abs(outs[0]) > 0.5, `channel 0 is producing output (${outs[0].toFixed(2)}V)`);
check(outs.slice(1).every(v => v === 0), 'the other channels stayed silent');
check(outs.every(v => Math.abs(v) <= 10.01), 'every output is inside +/-10V');

// The scope ring must be filling.
const head = F.scope_head(sim);
const scope = f32(F.scope(sim), 8 * 4096);
check(head > 0, 'scope head advanced');
check(scope.slice(0, 4096).some(v => v !== 0), 'scope captured channel 0');

/* ---- the LED curve is one curve ----------------------------------------- */

// web/leds.js and sim/include/led_color.h turn framebuffer bytes into a colour
// the same way, so that the browser and the VCV Rack module show the same
// module. Nothing links them, so this reads the C constants out of the header
// and compares. Two frontends that disagree about brightness is the kind of
// difference nobody notices until they are side by side.
{
  const header = readFileSync(join(here, '..', 'sim', 'include', 'led_color.h'), 'utf8');
  const define = name => {
    const m = header.match(new RegExp(`^#define ${name}\\s+(\\S+)`, 'm'));
    return m ? m[1] : null;
  };
  const js = readFileSync(join(here, 'leds.js'), 'utf8');
  const jsConst = name => {
    const m = js.match(new RegExp(`^const ${name} = ([0-9.]+)`, 'm'));
    return m ? m[1] : null;
  };

  // The header says VAL_MED rather than a number, which is the point - it is
  // the renderer's own brightness cap, not a value picked for the screen.
  check(define('LED_FULL_VALUE') === 'VAL_MED', 'led_color.h caps at VAL_MED');
  check(jsConst('LED_FULL') === '32', 'web/leds.js agrees (VAL_MED is 32)');
  check(define('LED_GAMMA') === `${jsConst('LED_GAMMA')}f`, 'both use the same gamma');
}

/* ---- a hold latches a shift mode ---------------------------------------- */

const qnt = spec.buttons.find(b => b.roles.ctrl_name === 'QNT');
F.set_button(sim, qnt.index, 1);
// 600ms. UI_T_HOLD is not exported, so this is a duration long enough that
// no plausible value of it fails - 300ms used to say "well past UI_T_HOLD"
// and stopped being true the moment the threshold moved.
F.run(sim, 250, 2400);
check(F.shift_state(sim) === 2, 'holding QNT latches QNT mode');
F.set_button(sim, qnt.index, 0);
F.run(sim, 250, 200);
check(F.shift_state(sim) === 2, 'the mode persists after release');

/* ---- the clock locks ---------------------------------------------------- */

const sim2 = F.create();
F.set_slider01(sim2, 0);
// Input 0 defaults to INPUT_CLOCK; 120bpm at 4 pulses/beat is a pulse every 125ms.
for (let i = 0; i < 40; i++) {
  F.fire_gate(sim2, 0);
  F.run(sim2, 250, 500); // 125ms
}
check(Math.abs(F.bpm(sim2) - 120) < 1, `clock locked to 120bpm (got ${F.bpm(sim2).toFixed(1)})`);
check(F.have_beat(sim2) === 1, 'clock reports a beat');

// Two instances must not interfere - the whole point of phase 1b.
check(F.bpm(sim) === 0, 'the first instance has its own clock');

/* ---- persistence round-trips -------------------------------------------- */

const size = F.storage_size();
check(size > 0, `storage blob is ${size} bytes`);
const buf = Module._malloc(size);
F.storage_get(sim, buf);
check(F.storage_set(sim, buf, size) === 1, 'storage round-trips');
check(F.storage_set(sim, buf, size - 1) === 0, 'a wrong-sized blob is rejected');
Module._free(buf);

F.destroy(sim);
F.destroy(sim2);

console.log(failures ? `\n${failures} failed` : '\nall passed');
process.exit(failures ? 1 : 0);
