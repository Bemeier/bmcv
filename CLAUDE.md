# BMCV

Eurorack module: eight clock-locked LFOs, seven scenes, one crossfader.
STM32G474. Four builds share one C core — firmware, native tests, wasm/CLI
simulator, VCV Rack plugin.

## Commands

`just` runs everything. `just --list` for the rest; the recipes carry their own
reasoning in comments.

```
just check          # ALWAYS run before saying done: fmt, tests, flows, wasm, web, dfu
just test           # ctest over the core, host-compiled (~10s)
just build          # ARM firmware -> build/BMCVFirmware.{elf,bin}
just flash          # over ST-Link (module out of the rack; can halt/breakpoint)
just flash-usb      # over the module's USB port via ROM DFU (module stays in the rack)
just web            # wasm + serve the frontend at localhost:8000
just fmt            # clang-format 18; fmt-check to verify
```

Slower or situational: `just check-all` (adds firmware + Rack plugin builds),
`just test-san` (ASan/UBSan), `just flows-bless`, `just where` (halt a hung
module over SWD), `just sim-run`, `just render`.

First run on a fresh checkout: `just arm-sdk` (firmware), `just wasm-sdk` (web).
Needs GCC 13+ / recent Clang — see [docs/setup.md](docs/setup.md).

## Layout

```
Core/Inc/Lib, Core/Src/Lib   the module. No host, no peripherals. See Core/CLAUDE.md
Core/Src/*, USB_Device/      STM32 glue: HAL, DMA, SPI, USB stack
sim/                         host support + flat C API + headless CLI + golden flows
web/                         browser frontend (wasm + plain ES modules)
vcv/                         VCV Rack plugin
tests/                       native unit tests
tools/                       generators; output is checked in
panel/                       generated panel geometry
docs/                        long-form reference (see below)
```

## Rules

**The seam is absolute.** `Core/Src/Lib` has no peripheral access, no globals,
and no idea what runs it. A behaviour that exists in a host and not in the core
is a behaviour the hardware does not have — widen the core's interface rather
than reimplementing on top of it.

**Generated files are never hand-edited.** Regenerate, review the diff, commit
both. See `tools/CLAUDE.md` for which recipe owns which file.

**Never hand-edit `CHANGELOG.md` or `VERSION`.** Commitizen writes both from
commit messages on `just bump`. Commits must be conventional-commit format
(`feat(scope):`, `fix(scope):`, …) — CI rejects otherwise. `just commit` for a
wizard.

**Add a test with the behaviour, not after it.** Every fix in this repo lands
with a case that fails without it. Prefer the gesture-level fixture over poking
state directly — see `tests/CLAUDE.md`.

**Plans live in `docs/plans/` and are deleted once implemented.**

### Avoid

- Do not move a parameter from the scene-indexed array onto the channel where
  it "looks like it belongs". Scenes are the whole point; this is the recurring
  wrong turn.
- Do not add a source file to a build without adding it to
  `cmake/core_sources.cmake` — all four builds read that one list.
- Do not add npm/pip dependencies to `web/` or the check scripts. Everything
  there runs on bare `node` and bare `python3`, deliberately.
- Do not parse the instance blob in JavaScript. The wasm decodes it; there is
  no struct parser, no duplicated unit conversion, no second copy of the LED
  curve on the JS side. See `web/CLAUDE.md`.
- Do not use enum types as struct fields in anything shared across builds — see
  `Core/CLAUDE.md`.
- Do not use `git rebase -i` / `git add -i`; not supported here.

## Style

Follow the surrounding code; it is consistent and deliberate.

- C: clang-format 18 (`.clang-format`, Allman, 140 cols, 2-space indent).
  Formatting is enforced by `just check`, so do not hand-align.
- **Comments say why, not what.** This codebase's comments carry the reasoning,
  the measurement, and the wrong turn that was taken first. Match that. A
  comment restating the code is noise; a comment recording why a constant is 6
  and not 4 is the point.
- JS: ES modules, no build step, no framework, no semicolon-free style — match
  the file.
- Prose in docs/comments: prefer concision over grammar, technical over
  stylised.

## Where the long-form docs are

Keep these current when the behaviour they describe changes.

| Doc | What it is for |
|---|---|
| [architecture.md](docs/architecture.md) | how the four builds share one core, and why `sim/` exists |
| [setup.md](docs/setup.md) | what to install per build target; both flash routes |
| [live-module.md](docs/live-module.md) | WebUSB + ST-Link probe, snapshots, `RemoteInput`, the update page |
| [pll.md](docs/pll.md) | the sync loop, and the measurement baseline to diff against |
| [wavetable.md](docs/wavetable.md) | the SHP axis and its generated guarantees |
| [led-language.md](docs/led-language.md) | one fact per LED; the colour vocabulary |
| [midi.md](docs/midi.md) | CC map, clock in/out, what is deliberately not sent |

The user-facing manual is `web/manual/index.html`, not a doc here.
