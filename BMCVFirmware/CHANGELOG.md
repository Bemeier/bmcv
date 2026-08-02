# Changelog

## Frontend split

`web/main.js` was 926 lines doing wasm binding, base64 `localStorage`, SVG panel
construction, LED gradients, encoder and slider gestures, two canvas scopes, CV
and clock generators, and the parameter table - the least modular thing in a
tree whose whole theme is modularity. It is now nine modules, none over 300
lines, with `main.js` down to wiring and the frame loop:

| module | what it owns |
|---|---|
| `sim.js` | the wasm module behind readable names; nothing else touches an `_bmcv_sim_*` symbol or a heap pointer |
| `spec.js` | `panel.json` and everything derived from it |
| `panel.js` | the SVG panel and its pointer handling |
| `leds.js` | the 21 WS2812s and their gradients |
| `scope.js` | both scopes |
| `inputs.js` | input faders, gate buttons, clock generator |
| `readouts.js` | the readouts and the channel table |
| `storage.js` | the `localStorage` mirror |
| `const.js` | the numbers more than one of them needs |

### Added

- **`web/frontend-check.mjs`** (`just web-check`): stands up enough of a DOM to
  import all nine modules for real, against the real `bmcv.wasm` and the real
  `panel.json`, then drives frames and asserts the module responded. It catches
  what a static check cannot - a missing export, a module-scope reference to
  something not yet defined, an element id that is not in `index.html`, an
  import cycle leaving a binding uninitialised - without opening a browser.
  `web/smoke.mjs` covers the wasm boundary; this covers everything on top of it.
- `just check` runs everything checkable without hardware or a browser: unit
  tests, golden flows, wasm smoke, frontend check.

### Fixed

- **The drawn crossfader position was published before the saved patch loaded.**
  Restoring reboots the module, which puts `slider_raw` back at one end, so the
  panel and the engine disagreed about where the handle was until the first
  drag. Publishing is now an explicit step in `main.js`, after `restore()`,
  rather than a side effect of building the panel.
- The input scope called `getBoundingClientRect()` every frame to decide
  whether to resize, which forces a layout for a size that only changes when
  the window does. Both canvases now size off a `ResizeObserver`.

### Changed

- The two scope draw functions were near-identical copies; cell drawing (grid,
  decimated trace, border) is shared, and the two differ only in their
  dimensions and whether cells are labelled.
- `IN_ORDER` is derived once in `spec.js`. The fader overlay and the input
  scope both lay out cells in panel order and have to agree - they each
  computed it before.

## Layering cleanup

A review pass over the UI/engine split and the simulator. Behaviour-preserving
except where noted: all four golden flows in `sim/flows/` still match byte for
byte. See `docs/review-ui-engine-split.md` for what was found and what is left.

### Fixed

- **Four test files never failed the build.** Their `main` called
  `TESTKIT_SUMMARY()` without `return`, so it fell off the end and exited 0 -
  ctest reported a green run no matter how many checks failed
  (`test_instance`, `test_panel_spec`, `test_sim_rt`, `test_input_state`).
  Fixed, and `testkit_summary` is now `[[nodiscard]]` so dropping the value is
  a compiler warning rather than a silent pass.
- **A channel counted as triggering another channel from 0.31V.**
  `CHANNEL_TRIG_THRESH` was the input-side value 1024 copied across without
  rescaling: identical numbers, but ADC counts are a quarter of DAC counts at
  the same voltage. Both domains now derive from one documented pair
  (`TRIG_THRESH`/`TRIG_THRESH_DAC`, ~1.25V rising with ~0.98V hysteresis), so a
  trigger means the same voltage wherever it is measured.
- **A save that failed still flashed green.** `SCN_PRESET` ignored
  `ux_preset_store`'s return and emitted the confirmation unconditionally,
  while the adjacent load path checked and raised an error. Store is now
  symmetric (`ERR_PRESET_STORE`).
- **A failed autosave was never retried.** `last_crc` was updated whether or
  not the write succeeded, so the next interval saw no change and did nothing
  until the user edited something else.
- **`preset_store`/`preset_load` did not guard a negative slot**, which would
  wrap the `uint16_t` address and read or write an arbitrary part of FRAM. Not
  reachable from any current caller; the simulator's equivalents already
  checked, and the two backends should not disagree about who validates.
- **Scene buttons 4-6 blinked as candidates in MON and QNT**, where the scene
  row addresses the four input jacks and those three buttons do nothing. The
  renderer drew them dark and then let the context layer paint over it; the
  handler had the guard and the renderer did not.

### Changed

- **Mute is applied by the engine, not by whoever reads the output.**
  `channel_output_level()` advanced the ramp as a side effect and carried a
  prose contract that any host reading outputs must call it - so a host that
  read `channels_output_level[]` directly got ungated audio and a mute ramp
  frozen in place. `engine_tick` now calls `channel_apply_mute` once per tick
  and publishes `EngineState.channels_gated_level[]`; the firmware and the
  simulator both just read it.
- **The DSP takes what it touches.** `channel_compute`, `channel_detect_trigger`,
  `channel_apply_mute`, `channel_init`, `channel_reset*` and `channel_take_trig`
  took `UxState*` - the whole module, interaction layer and preset vtable
  included - and used only a channel index from it. They now take
  `(uint8_t ch, EngineState*, const EngineConfig*, const HwState*)` as needed,
  which is what makes the layering structural rather than a convention, and is
  the shape an audio thread in a plugin host wants.
- **`input_fold.c` (was `input_state.c`) is a transducer again.** It drove the
  clock, reset every channel's phase, cleared error flags and ran the config
  autosave. It now latches `HwState.clock_pulse`/`clock_reset` and stops;
  `engine_tick` acts on them with the same timestamp, and the autosave moved to
  `ux_state.c`, which owns the preset io. `InputFrames` is the frame ring and
  nothing else.
- **The mode table drives the scene LEDs and the keyboard overlay.**
  `render_scene` branched on the descriptor and then re-checked `shift_state`
  three more times for SYS, SAV and STA/STB; `quantizer.c` and
  `render_quantizer` each named `SHIFT_STATE_QNT` themselves. New descriptor
  fields `scene_btn_base`, `xfade_end` and `keyboard_overlay` cover all of it,
  and `SCN_SET_A`/`SCN_SET_B` collapse into one `SCN_SET_XFADE`. Adding a mode
  is a row in `ui_mode.c`.
- **`ui_select.c` no longer names a shift mode.** Arming QNT's trigger mode and
  clearing MON's routing were written inline in the generic "pick a source"
  path. They are `assign_trig_arm_channel` / `assign_input_clear`, dispatched on
  `UiAction` - the vocabulary that file does own.
- **`channel.c` and `scene.c` split by layer.** `channel.c` was 510 lines of
  DSP, UX dispatch, config mutation, presentation tables and a driver call;
  the interaction half is `ui_channel.c` and the scene buttons are `ui_scene.c`.
  The one remaining driver call moved to `bmcv.c`, so nothing in the core
  touches a peripheral.
- **`state.h` split into `config.h`, `hw_state.h` and `engine_state.h`** -
  persisted record, hardware frame, running state - with `ShiftStates` moved to
  `ui_mode.h`, the file that enumerates what each mode does. No umbrella header:
  every file now includes only the layer it uses.
- Names follow the file they live in: `channel_*` for the signal path,
  `ui_channel_*`/`ui_scene_*`/`ui_ctrl_*` for the handlers, `ux_update` for the
  pass, `input_fold` for the sample-to-frame step.
- **First-boot defaults are `config_defaults()`**, next to the validation that
  has to accept them, rather than inline in the composition root.
- `bmcv_sim_mode_name()`/`bmcv_sim_mode_count()` expose the firmware's own mode
  names, and `dump_hw_setup` emits them as `ctrl_names`. The list
  STA/SYS/QNT/... existed in four places - the mode table, the CLI,
  `gen_panel_spec.py` and `web/main.js` - and four copies can disagree. The
  other three are now derived.
- `engine_fps` is measured inside `engine_tick`, so every host reports it the
  same way; both it and `dac_fps` are readable through the sim API.
- Ctrl-button ids mean three things at once (a `ShiftStates`, a
  `ChannelParameters`, an index into `ctrl_button_color[]`) and nothing said so.
  Two `_Static_assert`s in `ui_mode.c` now do.
- `TRIG_SRC_INPUT()`/`TRIG_SRC_CHANNEL()` name the composite trigger-source
  index space that was open-coded as `N_INPUTS + c`. Channel-indexed arrays say
  `N_CHANNELS` rather than `N_ENCODERS`, with a `_Static_assert` where the two
  being equal is relied on.
- `SLIDER_MIN_VALUE`/`SLIDER_MAX_VALUE` moved to `hw_setup.h` (analog
  calibration), the blink periods to `ui_state.h` (presentation),
  `FRAM_CONFIG_SLOTS` to `config.h` so a core file no longer includes a driver
  header for it.
- Both native builds compile with `-Wmissing-prototypes`; `led_set_adcr` and
  `led_set_dac` are wrappers over one `led_set_bipolar`.

### Removed

- **`envelope.c` / `envelope.h`**, an old experiment nothing referenced. It was
  in `BMCV_CORE_SOURCES`, so it compiled into all three builds; the linker was
  already dead-stripping it, which is why the firmware is the same size without
  it.
- `compute_channel_scene()` (empty, no callers, no prototype), `CTRL_DEFAULT`,
  `UxState.last_ux_update` (write-only), `UxState.dt` (a copy of `ui->in.dt`),
  and the commented-out level tables in `channel.c`.
- `ui_render_arm_all_edits` - arming a UI timer is `ui_state.h`'s business, not
  the renderer's (`ui_show_channel_edit` / `ui_show_all_channel_edits`).

## Virtual BMCV

Work toward running the real firmware logic off-target - headless, then in a
browser, later as a VCV Rack module. See `docs/plans/virtual-bmcv.md`.

### Added

- **Generated panel spec** (`panel/bmcv_panel.json`, `panel/panel_layout.h`,
  `panel/bmcv_panel.svg`, `web/panel.json`) built by `just panel` from three
  sources that cannot disagree: `BMCV.kicad_pcb` (board outline, placements and
  body sizes), `netlist.ipc` (WS2812 chain order, jack-to-converter paths), and
  the firmware's own `HwSetup`/`UxSetup` tables via `tools/dump_hw_setup.c`.
  Nothing in the layout is hand-typed - every button, LED, encoder and jack
  position is derived, because each WS2811 is mounted directly behind the
  control it lights, which resolves LED index to physical part by co-location.
- `tools/gen_panel_spec.py`, `tools/dump_hw_setup.c`, `panel/overrides.json`
  (the three facts the CAD data does not carry, listed as `assumptions` in the
  generated spec).
- `tests/test_panel_spec.c`: asserts every `UxSetup` pairing is physically
  co-located, that exactly three buttons have no LED, that no two controls
  share a position, and that everything lands inside the board outline.
  Swapping two entries in any `hw_setup.c` index table fails it.
- **`input_state.c`**: the step between raw hardware and `HwState`, hoisted out
  of `bmcv.c`. Takes a plain `InputSample` (slider, CV levels, latched gate
  edges, button levels, free-running encoder positions) and does the frame ring
  buffer, clock trigger/reset dispatch, slider CV summing, CV scaling, encoder
  deltas and the periodic config autosave. `input_trig_step()` exposes the
  ADC driver's gate hysteresis so a host can reproduce the same edges.
- `tests/test_input_state.c`: 77 checks over paths that previously had none -
  clock dispatch, reset-before-clock ordering, `INPUT_SLIDER` CV summing,
  autosave, encoder int16 wraparound, gate hysteresis.
- **`ChannelEffective`** in `EngineState`: per channel, what it is actually
  doing after the scene crossfade and the parameter maths - frequency in Hz and
  as a ratio of the beat, phase, shape, mod, peak amplitude and DC offset. This
  replaces the ad hoc `cgcd`/`cphsc`/`csphs`/`cfrm`/`cshp`/`cmod` fields, which
  were written every tick and read by nothing. Exposed by
  `bmcv_sim_effective()` and shown in the web frontend's channel table, which
  now reports what the module is doing rather than what was dialled into the
  active scene.
- **`instance.c`**: `BmcvInstance` is one module in one struct - config, signal
  path, interaction state, input layer and the wiring between them.
  `bmcv_instance_init()` is the power-on sequence; `bmcv_instance_tick()` is
  input fold + engine tick. The firmware holds one static instance; a
  simulator or VCV Rack patch holds one per module.
- `tests/test_instance.c`: two instances must not share a clock, error flags,
  input/UI state, preset storage or the LED framebuffer.
- **`sim/`**: the firmware's core behind a flat C API (`sim/include/bmcv_sim.h`)
  that speaks volts, 0..1 slider positions and encoder detents rather than
  converter counts, so a frontend needs none of the firmware's types.
  `bmcv_sim_run()` drives the real `engine_tick`; outputs, the 21 LEDs and an
  8-channel scope ring are readable as flat arrays.
- **`sim/src/sim_rt.c`**: host-side glue shared by every frontend - unit
  conversion, tick decimation and gate latching. Time accumulates in Q32
  microseconds: at 44.1kHz a rounded integer step would run the engine 0.17%
  slow, putting every LFO permanently flat.
- **The simulator's preset store persists across browser sessions.** The
  module autosaves its config to its last slot every couple of seconds; the
  frontend mirrors that blob into `localStorage` on the same cadence and
  restores it on load, so a patch survives a reload. Reset clears both. A blob
  whose length does not match `bmcv_sim_storage_size()` is rejected, and
  anything past that is still validated by `config_validate()` on boot.
- **Input panel rebuilt** as one canvas on a 2x2 grid laid out like the input
  jacks, matching the output scopes, with the controls overlaid on the cells
  rather than wrapped in boxes and labels of their own. Each cell carries a
  vertical fader, the trace of what the engine actually latched, and a pulse
  button; the clock generator sits on input 0's cell as a bpm number field and
  disables that fader while it runs. The fader fills from the centre outward,
  so the bar means "offset from 0V" - a native range input fills from its
  minimum and put a solid bar at -10V. `bmcv_sim_input_scope()` exposes the
  per-input history.
- The panel controls are restyled for the light artwork: dark outlines and
  spokes, a nearly transparent knob fill, lighter switch caps and a stronger,
  wider LED glow, since a wash that read well on a dark mock-up disappeared on
  a silver panel. The slider draws only its handle - the artwork has the slot -
  and the wiper travel (49mm) is now measured off that slot rather than guessed
  from the potentiometer body.
- The encoder cap is sized to the 7.03mm cutout in the panel artwork, leaving a
  hairline of it showing, with a correspondingly larger knob body. Legends are
  bold; the crossfader handle is shorter, its hit area uses a horizontal resize
  cursor, and the switch caps are lighter.
- Button legends are drawn rather than left to the artwork, so they stay correct
  if a button is ever reassigned: the parameter above (FRQ, SHP, ...) and the
  shift mode below (STA, SYS, ...), with CLR/CPY/MUT taking the lower slot
  alone since latching a mode is all they do.
- **The panel is drawn like the parts.** Buttons are round: 6mm for the
  illuminated switches, 7mm for the three unlit tactiles. Only the ctrl
  functions carry a legend and it sits above the button - the scene numbers and
  semitone names are gone, since a 6mm cap has no room and the layout already
  says which is which. Encoders have a smaller body (the cap is unchanged) and
  four spokes rather than one, because a single mark reads as an absolute
  pointer and these are endless relative encoders. Their LED is a ring around
  the body rather than a tint on the face: the WS2812 is behind the encoder, so
  its light spills around opaque plastic rather than through it.
- Output-jack and channel labels dropped, along with the slider's SCENE label.
  Separate **Reset module** (reboot, keep presets) and **Reset FRAM** (wipe
  presets and the browser copy) buttons.
- The input scope's cells are the same size as the output scope's, at the same
  pixel density and the same time per pixel, so a trace means the same thing in
  both. It reads at half the canvas width because it is 2 columns to the output
  scope's 4.
- **The exported panel artwork (`web/bmcv_panel.png`) is the panel background**,
  drawn 1:1 over the 81 x 128.5mm panel rectangle with the live controls on
  top. It lands exactly on the generated geometry, which is a useful check on
  the whole KiCad-derived pipeline. The drawn ctrl legends are gone with it -
  the artwork carries them, and it names the *parameter* (FRQ, SHP, MOD, ...)
  rather than the shift mode. The spec still exposes both as roles, for the
  hover hint and for anyone rendering without the artwork.
- LED edges are soft: each one paints through its own radial gradient rather
  than a flat fill - solid through the middle with the falloff only at the rim,
  rather than a bright point fading linearly all the way out. Done with
  gradients rather than an SVG blur filter because a paint server costs nothing
  to rasterise. Encoder bodies are slightly translucent so the colour bleeds
  through the knob as well as around it.
- The channel parameter table uses `table-layout: fixed` with tabular figures;
  it used to resize its columns on every value change and jitter constantly.
- **`bmcv_sim_cli`**: drives the module from a scripted input timeline
  (`sim/flows/*.txt`) and prints outputs / LEDs / UI state as CSV. Scripts name
  controls by role (`ctrl QNT 1`, `scene 2 1`, `chbtn 0 1`) rather than by
  button index.
- **`sim/flows/`** with `just flows` / `just flows-bless`: golden-file coverage
  of whole interactions - shift-mode entry and exit, mute, clock lock, param
  selection - which unit tests cannot express.
- `tests/test_sim_rt.c`: 125 checks over the shared runtime glue, including
  no-drift-over-ten-minutes at 44.1k and 48k, and a gate shorter than one tick
  still being seen.
- **WebAssembly build** (`just wasm`) of the same `sim/` project via emscripten,
  producing `web/bmcv.js` + `web/bmcv.wasm` (~105KB). The export list lives in
  `sim/CMakeLists.txt` and is explicit: `bmcv_sim` is a static library, so
  anything unnamed is dead-stripped before it can be exported.
- **`web/`**: a browser frontend that builds its whole panel from
  `web/panel.json` - nothing in the JS knows where any control is. Encoders
  turn by dragging the ring or scrolling and push via the centre cap (Shift
  while turning gives press-and-turn); the 21 LEDs are driven from the real
  framebuffer with a gamma lift so they read like WS2812s rather than flat
  fills. An 8-lane scope, scene contribution bars, a live parameter table and
  CV/clock generators fill in what a panel photo cannot show.
- **`web/smoke.mjs`** (`just wasm-check`): headless node check that the module
  loads, that all 26 exports are present, that heap views line up, and that
  driving it through the flat API actually moves the engine - so a broken wasm
  build is caught without opening a browser.

### Changed

- **`bmcv_state_update` is now only peripheral reads** (~25 lines): it fills an
  `InputSample` and hands it to the core. Every `bmcv_*` entry point keeps its
  signature, so `main.c` and the HAL callbacks are untouched.
- **Presets go through a `PresetIo` vtable on `UxState`** rather than direct
  `preset_store`/`preset_load` calls from `scene.c`. The backing store is FRAM
  on the module and will be patch JSON / browser storage off it. A NULL
  `PresetIo` is valid: store is a no-op, load reports "nothing stored".
- `ui_render.c` no longer calls `get_adc` for the MON scene LED - it reads the
  same value from `hw_state->input_state[]`, which makes that path assertable
  and removes the last driver dependency from the render layer.
- `TRIG_THRESH`/`TRIG_THRESH_LOW` moved from `dac_adc.h` to `hw_setup.h`,
  alongside the CV range constants and for the same stated reason: a simulator
  needs them without pulling in a driver header.
- `tests/fakes/fake_drivers.c` is down from five stubs to one (`dacadc_write`).
- **`channel_output_level()` split out of `write_channel_dac()`.** Mute is an
  output-stage gain, not a zeroed `channels_output_level`, so what actually
  leaves the module was only computed inside the function that talks to the
  DAC. Any host reading `channels_output_level[]` directly would show a muted
  channel as live - the simulator did, until this split.
- **The clock is per instance.** `g_clk` is gone; `ClockState` is a field of
  `EngineState` and every `Clock_*` takes a `ClockState*`.
- **Error flags are per instance.** The `error.c` file static is gone;
  `error_flags` is a field of `EngineState` and every `error_*` takes an
  `EngineState*`.
- **`bmcv.c` holds a single `BmcvInstance`** instead of six file statics, and
  `bmcv_init` is now pin setup plus one `bmcv_instance_init` call. Every
  `bmcv_*` entry point keeps its signature.

### Fixed

- **Panel geometry came from the wrong file.** It was read from
  `production/positions.csv`, which is a *pick-and-place centroid* export: for a
  footprint whose pads are not symmetric about its origin the centroid sits to
  one side, and the offset flips sign with the footprint's rotation. The jacks
  are on a uniform 13mm grid but alternate 90/270 degrees, so they came out at
  14.03/11.97mm alternating - visibly wrong. Geometry now comes from the
  footprint placements in `BMCV.kicad_pcb`, which also yields the board outline
  from Edge.Cuts, so the board size and the panel offset are derived instead of
  typed into the overrides.
- **The simulator booted showing scene 6 until the slider was touched.** The
  frontend drew the crossfader at one end but never told the engine, which had
  come up with `slider_raw` at the other. The drawn position is now published at
  startup, and there is one function that moves both.
- **The browser tab could freeze solid on load.** `bmcv_sim_run` took an
  unsigned `n_ticks`, and the frontend computed it from `rAF` timestamp minus
  `performance.now()` without a lower bound. rAF hands a callback the *current
  frame's* start time, which can be earlier than a `performance.now()` sampled
  just before scheduling it, so the first frame sometimes produced a negative
  count - which arrived in C as ~4.29 billion ticks and blocked the main thread
  for good. No JS error, no crash event, just a dead page, and intermittent
  because it depended on frame timing.
  `bmcv_sim_run` now takes signed counts, rejects zero or negative and caps at
  `BMCV_SIM_MAX_TICKS`; the frontend clamps elapsed time at zero. Covered by
  `web/smoke.mjs`. Eight consecutive page loads: 8/8 alive, previously about one
  in three froze.
- **A fresh module no longer boots into an error display.** Finding no stored
  config raised error bit 6, which `render_error` draws by blanking every LED
  and blinking scene 6 until the next interaction - a startup screen announcing
  that nothing is wrong. An unused module having no saved config is its normal
  state, and `config_validate()` has already made the defaults safe to index. A
  slot the user *explicitly* asks for and cannot read still reports (`scene.c`,
  error bit 5).
- **The 2s config autosave is silent.** It flashed the whole scene row green on
  every write, so idling with a knob just moved produced a confirmation nobody
  asked for. Confirmations are for committed actions; background housekeeping
  on a timer is not one. An explicit save still flashes.
- **The slider was drawn vertical with a guessed 45mm travel.** RV13's
  footprint is 73 x 16.8mm with its pads at x = +/-34, so it is *horizontal*
  with roughly 60mm of travel. The generator now reads the footprint extent
  out of `BMCV.kicad_pcb`, so the axis and body size are derived rather than
  assumed and only the wiper stroke remains an override.
- **The web panel's LEDs were invisible.** They were drawn as circles in a
  layer beneath the controls, and the opaque button and knob bodies covered
  them completely. Physically the switch cap *is* the lamp, so each control now
  carries a tint plus a halo, both transparent when the LED is off.
- **LED brightness was calibrated against the wrong ceiling.** `led_fb.c` caps
  at `VAL_MED` (32) and draws base layers at `VAL_LOW` (8); normalising against
  255 left everything the renderer actually draws invisible. Brightness now
  rides on opacity against a `VAL_MED` full scale with a perceptual curve, and
  hue on a fill normalised to full range so a dim red still reads as red.
- An SVG blur filter over a panel-sized group, re-rasterised every animation
  frame, crashed the renderer outright; the glow is a translucent oversized
  shape instead. The scope polyline is decimated to the canvas width and the
  readout table updates at 10Hz rather than 60Hz - it was the largest source of
  DOM churn on the page.
- **The scope was a fixed 1000x440 bitmap stretched to fit**, so it was visibly
  soft on a wide or HiDPI display. Its backing store now tracks the element's
  real size times `devicePixelRatio`, via a `ResizeObserver`.
- **Slider direction.** Scene A anchors at `SLIDER_MAX_VALUE` (`scene.c`) and
  sits on the left of the panel, so the leftmost position has to map to full
  scale, not zero.
- **The scope stops growing past ~1100px** and the side panels move alongside
  it on wide screens rather than the traces stretching forever. Two jacks
  abreast collapses the range inputs in the narrower column, so they go one per
  row there.

### Notes

- **There is no channel-to-jack crossing.** An earlier revision of the
  generator reported one; that was a tracing error, not a hardware or firmware
  fault. The two AD5754Rs are daisy-chained (MCU -> U34 -> U35) behind a single
  shared `DAC_1_SYNC`, so in each 6-byte transfer the *first* command shifts
  all the way through U34 into U35 and the second stays in U34 - even buffer
  indices are U35, odd are U34, the opposite of what `dac_init()`'s ordering
  suggests at a glance. With that corrected, `channel_dac_idx` puts channels
  0-3 on J1-J4 (left block) and 4-7 on J5-J8 (right block), each in the same
  reading order as their encoders. `hw_setup.c` is correct as shipped.
- **The ADC ADDR phase is resolved.** `input_adc_idx = {2,3,0,1}` only reads as
  a sensible panel if the A2/B2 converter slots land in `adc_i[0..1]`, which
  puts inputs 0-3 on J9-J12 in reading order. The other phase would number the
  jacks 2,3,0,1 across the panel, which nothing else on the module does.

## UI layer restructure

Separated the interaction layer from the engine and gave the recurring UI
concepts one implementation each.

### Added

- **MUT shift mode.** Channel button toggles that channel's mute on release.
  Gates the DAC write only, so a muted channel still feeds cross-modulation and
  still works as a trigger source. Ramps over 5ms so it does not click. Renders
  dim purple in every mode. Not persisted - clears on power cycle.
- **Confirmation feedback** (`ui_feedback.c`) for copy, clear, save, load,
  assign and scene A/B - same flash, same timing, colour by action class. Most
  of these previously gave no feedback at all.
- **Semantic palette** in `color_presets.h` plus `HUE_PURPLE`. The renderer
  names meanings, not colours. Brightness capped at `VAL_MED`.
- **Candidate highlighting** driven by the same predicate the dispatcher uses,
  so what blinks and what responds cannot disagree.
- Tests: `test_ui_input`, `test_ui_gestures`, `test_ui_select`,
  `test_ui_feedback`, `test_ui_mode`. The button path had no coverage before.

### Changed

- **Button gestures derived once** (`ui_input.c`) as edge events, with the
  press durations defined in one place. Events accumulate across ticks where
  the UX layer does not run, so a hold crossing its threshold is never missed.
- **`EngineState` split**: signal path only; interaction and view state moved
  to `UiState`. `compute_scenes_contribution` is now pure.
- **Selection model** (`ui_select.c`) replaces the assign type/target matrix
  and the mode-specific exceptions that lived in its callers.
- **Layered renderer** (`ui_render.c`) replaces the four `write_*_led`
  switches: base, context, transient value, confirmation. The base layer always
  writes, so stale pixels are impossible.
- **Channel LEDs** show the output level as their base in every mode; the value
  being edited surfaces on touch and decays. Entering a mode reveals every
  channel's state for that mode briefly.
- **Mode behaviour is a table** (`ui_mode.c`) read by both dispatch and render.
- **Shift-mode exit**: a mode's own button always exits; other ctrl buttons
  exit unless the mode opts out (only QNT does, because its buttons are a
  semitone keyboard). The exit tap no longer also changes the selected param.
- All UI timers age in one place against one dt.

### Fixed

- `scene.c` compared a hold against 10 microseconds where 10 milliseconds was
  meant.
- SAV drew its LED from the press timer but acted on the release timer, so the
  two disagreed at the boundary. Store now fires on the hold crossing.
- The QNT scene-button arm wrote no LED unless a trigger source was pending,
  leaving the framebuffer holding the previous frame.
- The MON channel arm fell through into the default, letting the value overlay
  clobber the mode colour.
- An error display returned early from the input update, suppressing the whole
  UX pass while it showed.
- `momentary_scene` was zero-initialised on hardware, reading as "scene 0 held"
  until the first UX pass corrected it.

### Removed

- `HwState.button_pressed_t` / `button_released_t`, `AssignType`, and the three
  ad hoc indicator timers they fed.
