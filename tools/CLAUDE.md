# Tools

Generators and offline harnesses. **All generated output is checked in** — so a
reader gets it without a toolchain and a reviewer sees it change as a diff.

**Never hand-edit a generated file.** Change the generator, re-run its recipe,
review the diff, commit generator and output together. Then `just test` — the
properties these guarantee are asserted, not assumed.

## Generators

| Recipe | Generator | Writes |
|---|---|---|
| `just wavetable` | `gen_wavetable.py` | `Core/{Inc,Src}/Lib/wavetables.{h,c}` + `docs/images/wavetable-*.svg` |
| `just stepped-table` | `gen_stepped_table.c` | `Core/Inc/Lib/stepped_table.h` |
| `just panel` | `gen_panel_spec.py` (+ `dump_hw_setup.c`) | `panel/`, `web/panel.json`, `vcv/res/` |
| `just layout-check` | `dump_layout.py` + `gen_layout_asserts.py` | `sim/include/layout_target.h` |
| `just vcv-compdb` | `gen_compdb.py` | `vcv/compile_commands.json` |
| `just shape-figures` | `dump_shapes.c` + `gen_shape_figures.py` | `docs/images/shape-*.svg` |
| `just docs-shots` | `shoot.mjs` + headless Chrome | `docs/images/{web-overview,led-language}.png`, `web/manual/{panel,scopes}.png` |

Notes that matter:

- **`gen_wavetable.py` emits the header, the source and the plots in one run**,
  so the docs cannot show a shape the firmware does not have. It is generated
  rather than drawn because the guarantees — full swing and no DC at *every*
  setting, canonical shapes exact, the axis closing on itself — are true of the
  whole family by construction and were not true of a table edited slice by
  slice. `just wavetable --report` prints the axis as sparklines, writing
  nothing.
- **`gen_stepped_table.c` includes `Core/Inc/Lib/stepped_pattern.h`**, the
  same value path the firmware runs, so the normalisation cannot be aimed at a
  pattern the module no longer plays. It used to carry its own copy; do not
  reintroduce one.
- **`gen_shape_figures.py` draws from the module's own shape functions**, via
  `dump_shapes.c`, for the same reason: a picture of a shape the firmware does
  not have is worse than no picture. One SVG per mode, SHP down the rows and MOD
  across the columns.
- **`shoot.mjs` drives the panel to the state each picture shows**, through the
  browser's own input events and `panel.json`'s coordinates — so a screenshot is
  generated output like everything else here, and what it is a picture of is
  readable rather than being whatever someone had on screen. They were
  hand-taken until the LED palette moved and every one of them quietly became a
  picture of colours the module no longer has. No dependencies: the DevTools
  protocol is a WebSocket and node has had one built in since 22. The two shots
  of a *running* module wait for the channels to swing positive before the
  shutter, so a still of a live oscillator is not a matter of luck.
- **`gen_panel_spec.py` merges KiCad output with the firmware's own `HwSetup`
  tables**, so positions and roles derive from board and firmware together.
  `tests/test_panel_spec.c` asserts they agree. What genuinely cannot be
  derived goes in `panel/overrides.json` and is listed under `assumptions` in
  the output, so an assumption cannot hide.
- **`just layout-check` needs a current `build-rel`.** It reads the firmware
  ELF's debug info, so stale build, stale assertions. Run `just build-rel`
  first.

## Offline harnesses

**`stepped_explore/`** — the measurement harness `stepped_shape()`'s two knobs were
tuned with. Keep it: it is the only thing that can judge a change to that
algorithm, and the obvious metrics actively mislead (the old sweep contained
400 distinct patterns at 32 steps and still read as nothing happening).
`stmodel.py` replicates the shipping C to 1.8e-6; `verify_c.py` checks they
still agree; `invariants.py` mirrors `tests/test_stepped.c`. Read
`stepped_explore/README.md` before touching it — `character.scale()` must run before
the first variant or the numbers are not comparable.

**`render_channel.c`** (`just render`) — renders a channel to a WAV. Built by
`tests/CMakeLists.txt`, uses `wav_writer.h`.

## Dead weight

- **`delta_probe.c`** — measured how much of a `BmcvInstance` changes between
  snapshots, to decide whether the MIDI transport could send deltas. That
  transport was deleted (see `docs/live-module.md`). It is in no build and
  answers a question nobody has. Safe to remove.
- **`bmcv.mcuvproj`** — an MCUViewer project for watching variables over
  ST-Link. Superseded by `web/probe/` and `web/diagnostics/`, which do the same
  job in a browser. Keep only if MCUViewer is still in use.
