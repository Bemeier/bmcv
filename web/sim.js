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
// From the byte heap, not a HEAPU16: the emscripten glue only publishes the 8,
// 32 and f32 views. The byteOffset is the pointer as-is, and every uint16 array
// the API hands out is aligned by virtue of being one.
const u16 = (ptr, len) => new Uint16Array(Module.HEAPU8.buffer, ptr, len);

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

// One BmcvInstance, and a buffer to move one through. Allocated at load and
// reused, because on a probe-backed page this carries a snapshot every frame.
const instanceSize = _('instance_size')();
const snapshotBuf = Module._malloc(instanceSize);

// One drained MIDI message, matching MidiMsg in Core/Inc/Lib/midi_out.h.
const MIDI_MSG_BYTES = 4;

// The module's queue is 32 deep, so nothing is ever left behind by draining
// this many. Allocated once, at load, and reused every frame - the alternative
// is a malloc/free pair per frame for a buffer that never changes size.
const MIDI_DRAIN_MAX = 32;
const midiBuf = Module._malloc(MIDI_DRAIN_MAX * MIDI_MSG_BYTES);

export const sim = {
  /* ---- input ------------------------------------------------------------ */

  setButton: (index, down) => _('set_button')(handle, index, down ? 1 : 0),
  addEncoder: (index, detents) => _('add_encoder')(handle, index, detents),

  // 0.0 = bottom of travel, 1.0 = top.
  setSlider01: pos => _('set_slider01')(handle, pos),

  // Volts on an input *jack*, not a converter channel.
  setCv: (input, volts) => _('set_cv')(handle, input, volts),

  /* ---- driving another module ------------------------------------------- */
  //
  // The mailbox a physical module reads its remote panel out of. These do not
  // touch this instance: they fill a small struct that the probe writes into
  // the board's RAM, where its input layer merges it with the panel someone
  // may also have their hands on. See web/input.js for who calls which, and
  // Core/Inc/Lib/input_fold.h for the merge.
  //
  // The bytes are laid out by the wasm, never here - the same rule the read
  // direction follows, and for the same reason: a copy of the struct written
  // out in JS would be a second definition of it, free to drift.

  remoteButton: (index, down) => _('remote_button')(handle, index, down ? 1 : 0),
  remoteEncoder: (index, detents) => _('remote_encoder')(handle, index, detents),

  // 0..1 to take the module's crossfader, negative to hand it back.
  remoteSlider01: pos => _('remote_slider01')(handle, pos),
  remoteClear: () => _('remote_clear')(handle),

  remoteOffset: _('remote_offset')(),
  remoteSize: _('remote_size')(),

  // Stamped with a fresh sequence number on every call - the far end reads it
  // as a heartbeat, so a write that changes nothing still has to happen.
  //
  // Copied out rather than returned as a view: this is handed to WebUSB and
  // awaited, and a view into the wasm heap does not survive it growing.
  remoteBlob: () => u8(_('remote_blob')(handle), sim.remoteSize).slice(),

  /* ---- running ---------------------------------------------------------- */

  // Negative and zero counts are rejected inside, and the count is capped -
  // this is the boundary where a bad frame-timer result used to reach C as
  // ~4.29 billion ticks and freeze the tab for good.
  run: (dtUs, ticks) => _('run')(handle, dtUs, ticks),

  // Back to power-on. Pass true to wipe the stored presets as well.
  reset: wipeStorage => _('reset')(handle, wipeStorage ? 1 : 0),

  /* ---- output ----------------------------------------------------------- */

  outputs: () => f32(_('outputs_v')(handle), N_CH),
  // 8.8 fixed point duty per primary, not bytes - see led_curve.h.
  leds: () => u16(_('leds_rgb')(handle), N_LEDS * 3),
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

  // The module's own measured loop rates. ledFps is hardware-only - it counts
  // flushes to the WS2811, which the simulator has none of - and reads 0 here.
  engineFps: () => _('engine_fps')(handle),
  dacFps: () => _('dac_fps')(handle),
  ledFps: () => _('led_fps')(handle),

  // Where the panel is, as the engine last read it - so a drawn control follows
  // whichever module is driving the page rather than only this one's mouse.
  encoderPos: e => _('encoder_pos')(handle, e),
  slider01: () => _('slider01')(handle),

  /* ---- midi ------------------------------------------------------------- */

  // Take what the module has queued for the MIDI bus, as an array of
  // Uint8Arrays ready to hand to a MIDIOutput. The scratch buffer is allocated
  // once and reused - this runs every frame.
  //
  // Each record is four bytes (status, d1, d2, len), and `len` says how many of
  // the first three are real: 1 for a clock byte, 3 for a control change. So a
  // caller needs no MIDI parser, only the slice.
  drainMidi() {
    const n = _('midi_drain')(handle, midiBuf, MIDI_DRAIN_MAX);
    if (n === 0) return [];

    const raw = u8(midiBuf, n * MIDI_MSG_BYTES);
    const out = [];
    for (let i = 0; i < n; i++) {
      const at = i * MIDI_MSG_BYTES;
      out.push(raw.slice(at, at + raw[at + 3]));
    }
    return out;
  },

  /* ---- snapshots -------------------------------------------------------- */
  //
  // The whole running module as bytes, which is how a physical one gets onto
  // this page: its firmware keeps its instance in a single global, so a debug
  // probe reads those bytes out of RAM and `importInstance` makes every reading
  // above report the hardware. See docs/plans/probe-bridge.md.
  //
  // The scratch buffer is allocated once, not per import: this runs on every
  // frame that a probe is connected.

  instanceSize,

  importInstance(bytes) {
    if (bytes.length !== instanceSize) return false;
    u8(snapshotBuf, instanceSize).set(bytes);
    return _('import')(handle, snapshotBuf, instanceSize) === 1;
  },

  exportInstance() {
    _('export')(handle, snapshotBuf);
    return u8(snapshotBuf, instanceSize).slice();
  },

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
