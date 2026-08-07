// The generated panel spec, and the things every module derives from it.
//
// panel.json comes from tools/gen_panel_spec.py, which merges the hardware's
// KiCad placement data with the firmware's own index tables. Nothing in the
// frontend knows where a control is; it all comes from here, so the picture
// cannot drift from the board.

export const spec = await (await fetch('panel.json')).json();

const [OX, OY] = spec.panel.board_offset_mm;

// Board millimetres -> panel millimetres, which is the SVG's viewBox.
export const px = ([x, y]) => [x + OX, y + OY];

// Parameter names ride along in the spec's button roles, which the generator
// derives from the firmware's ctrl-button ordering - so they are not retyped
// here either. Shift-mode names come from the wasm module (sim.js).
export const PARAM_NAMES = [];
for (const b of spec.buttons ?? []) {
  const r = b.roles ?? {};
  if (r.param !== undefined && r.param_name) PARAM_NAMES[r.param] = r.param_name;
}

// Reading order for a grid of cells laid out like the panel: top row left to
// right, then the next row down.
const byPosition = list => [...list].sort(
  (a, b) => (a.pos_mm[1] - b.pos_mm[1]) || (a.pos_mm[0] - b.pos_mm[0]));

// Input jacks in panel order. The input scope's cells and the fader overlay
// grid share this, so a control sits on the trace it belongs to - they have to
// agree, which is why it is derived once here rather than in each of them.
export const IN_ORDER = byPosition(spec.inputs).map(j => j.index);

// Channels in the order their encoders sit on the panel, so a scope trace is
// where its knob is.
export const SCOPE_ORDER = byPosition(spec.encoders).map(e => e.channel);
