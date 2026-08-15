# Web frontend

Four pages, one stylesheet, plain ES modules. **No build step, no framework, no
dependencies** — `web/` is served as-is. Keep it that way; do not add npm.

```
just web         # build wasm + serve at localhost:8000
just wasm-check  # headless: the wasm module loads and its exports behave
just web-check   # headless: the whole frontend imports and drives the engine
just dfu-check   # the DfuSe client against a fake device
just docs-page   # serve only the pages that need no wasm
```

`just check` runs all four. Run `web-check` after touching any module — it
catches a missing export, a bad element id or an import cycle without a
browser.

Serve through `scripts/serve.py`, not `python3 -m http.server`: it sends
`no-store`, without which a browser reuses a cached ES module and an edit
simply does not appear.

## The pages

| Path | What it is | Needs wasm |
|---|---|---|
| `/` | simulator + live module view | yes |
| `/manual/` | the user-facing manual | no |
| `/update/` | firmware updater over DFU | no |
| `/diagnostics/` | USB link instruments | no |

The three no-wasm pages are separate deliberately: a broken simulator build
must not take down the thing that flashes the module.

## Rules

**Nothing on this page ever looks inside the instance blob.** The wasm decodes
it — `bmcv_sim_import()` takes the bytes, re-points the pointers at itself and
republishes every reading. So there is no struct parser in JS, no duplicated
unit conversion and no second copy of the LED curve. This is the property that
let three transports be swapped underneath without touching the page. If you
need a new reading, export it from `sim/`, do not parse it here.

**The panel is built entirely from `panel.json`**, which is generated from
KiCad placement plus the firmware's index tables. Nothing in `panel.js` knows
where anything is, so the picture cannot drift from the board. Do not hardcode
a coordinate.

**Constants that the firmware also has come from the firmware.** `const.js`
holds only numbers more than one module needs; anything with a C counterpart is
checked against it (the LED curve, in `smoke.mjs`).

**Every call to a USB device must be bounded.** WebUSB transfers have no
timeout: a read for data that never comes never settles, with no error and
nothing to retry — which during a connect locks every button on the page.
`frontend-check.mjs` holds this, because it is easy to lose one call at a time.

## Module map

`main.js` is wiring and the frame loop only. Substance lives in `sim.js` (the
wasm behind readable names), `panel.js`, `leds.js`, `scope.js`, `inputs.js`,
`readouts.js`, `storage.js`, `midi.js`, `mode.js`, `spec.js`, `probe/`.

`probe/wire.js` holds the USB IDs and the vendor protocol; `update/dfuse.js`
holds the DFU ones. `scripts/flash-usb.sh` repeats those two constants — keep
them in step.

## Style

Match the surrounding file. Comments explain *why* — the wrong turn taken
first, the measurement behind a number — not what the line does. The manual
(`manual/index.html`) uses `dl`/`dt`/`dd` and `p.note` and no lists; there are
no `ul` styles in `style.css`.
