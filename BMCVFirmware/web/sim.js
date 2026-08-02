// The wasm module, behind names a reader can follow.
//
// Everything past this file talks to the module through `sim` and never
// touches an underscore-prefixed Emscripten symbol or a raw heap pointer. The
// flat C API is sim/include/bmcv_sim.h; this is a thin transliteration of it,
// not a layer with opinions of its own.

import createBmcv from './bmcv.js';

const Module = await createBmcv();

// Direct exports: cwrap is not in the build, so these are the underscore
// symbols Emscripten installs on the module.
const _ = name => Module[`_bmcv_sim_${name}`];

const handle = _('create')();

// ALLOW_MEMORY_GROWTH can detach the heap's ArrayBuffer, which silently
// invalidates any view held across a call. Re-derive views on each read; they
// are just windows onto the heap, so this costs nothing.
const f32 = (ptr, len) => new Float32Array(Module.HEAPF32.buffer, ptr, len);
const u8 = (ptr, len) => new Uint8Array(Module.HEAPU8.buffer, ptr, len);

// Mirrors of the counts in bmcv_sim.h. The wasm smoke test asserts the heap
// views line up, so a mismatch is caught without opening a browser.
export const N_CH = 8;
export const N_IN = 4;
export const N_LEDS = 21;
export const SCOPE_LEN = 4096;

// Effective-value fields, matching the enum in bmcv_sim.h.
export const EFF = { FREQ_HZ: 0, FREQ_RATIO: 1, PHASE: 2, SHAPE: 3, MOD: 4, AMP_V: 5, OFFSET_V: 6, GCD: 7, PHASE_OFS: 8, COUNT: 9 };

// The shift-mode names, read out of the firmware's own ui_mode.c table rather
// than retyped. This list used to exist in four places - the mode table, the
// CLI, the panel generator and the frontend - and four copies can disagree.
export const SHIFT_NAMES = Array.from({ length: _('mode_count')() }, (_i, i) => {
  const ptr = _('mode_name')(i);
  return ptr ? Module.UTF8ToString(ptr) : '—';
});

// Same idea for the waveshape modes: the firmware names them, the table here
// only looks them up.
export const SHAPE_NAMES = (() => {
  const names = [];
  for (let i = 0; ; i++) {
    const ptr = _('shape_mode_name')(i);
    if (!ptr) return names;
    names.push(Module.UTF8ToString(ptr));
  }
})();

const storageSize = _('storage_size')();

export const sim = {
  /* ---- input ------------------------------------------------------------ */

  setButton: (index, down) => _('set_button')(handle, index, down ? 1 : 0),
  addEncoder: (index, detents) => _('add_encoder')(handle, index, detents),

  // 0.0 = bottom of travel, 1.0 = top.
  setSlider01: pos => _('set_slider01')(handle, pos),

  // Volts on an input *jack*, not a converter channel.
  setCv: (input, volts) => _('set_cv')(handle, input, volts),

  /* ---- running ---------------------------------------------------------- */

  // Negative and zero counts are rejected inside, and the count is capped -
  // this is the boundary where a bad frame-timer result used to reach C as
  // ~4.29 billion ticks and freeze the tab for good.
  run: (dtUs, ticks) => _('run')(handle, dtUs, ticks),

  // Back to power-on. Pass true to wipe the stored presets as well.
  reset: wipeStorage => _('reset')(handle, wipeStorage ? 1 : 0),

  /* ---- output ----------------------------------------------------------- */

  outputs: () => f32(_('outputs_v')(handle), N_CH),
  leds: () => u8(_('leds_rgb')(handle), N_LEDS * 3),
  scope: () => f32(_('scope')(handle), N_CH * SCOPE_LEN),
  inputScope: () => f32(_('input_scope')(handle), N_IN * SCOPE_LEN),
  effective: () => f32(_('effective')(handle), N_CH * EFF.COUNT),
  scopeHead: () => _('scope_head')(handle),

  /* ---- introspection ---------------------------------------------------- */

  shiftState: () => _('shift_state')(handle),
  selectedParam: () => _('selected_param')(handle),
  activeScene: () => _('active_scene')(handle),

  // Two tempos: what the clock input measured, and what the oscillators are
  // running against. They only agree while a clock is locked - see haveBeat.
  bpm: () => _('bpm')(handle),
  activeBpm: () => _('active_bpm')(handle),
  haveBeat: () => _('have_beat')(handle) === 1,

  muted: c => _('channel_muted')(handle, c),
  shapeMode: c => _('channel_shape_mode')(handle, c),

  /* ---- persistence ------------------------------------------------------ */
  //
  // The whole preset store as an opaque blob. Copied in and out through a
  // scratch allocation rather than exposing the pointer, so no caller can hold
  // a view across a heap resize.

  storageSize,

  readStorage() {
    const buf = Module._malloc(storageSize);
    _('storage_get')(handle, buf);
    const bytes = u8(buf, storageSize).slice();
    Module._free(buf);
    return bytes;
  },

  writeStorage(bytes) {
    if (bytes.length !== storageSize) return false;
    const buf = Module._malloc(storageSize);
    u8(buf, storageSize).set(bytes);
    const ok = _('storage_set')(handle, buf, storageSize) === 1;
    Module._free(buf);
    return ok;
  },
};
