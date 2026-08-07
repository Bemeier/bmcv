# How this repository is arranged

Four things are built from it: the firmware, a headless simulator, a browser
frontend and a VCV Rack module. All four run the same C, and the arrangement
below exists to keep that true rather than approximately true. For how to
build each - what to install and in what order - see
[setup.md](setup.md).

```
Core/Inc/Lib  Core/Src/Lib   the module. No host, no peripherals.
Core/Src/*, USB_Device/      the STM32: HAL glue, DMA, SPI, the USB stack.
sim/                         host-side glue, plus a headless CLI.
web/                         the browser frontend (wasm + plain ES modules).
vcv/                         the VCV Rack plugin.
tests/                       native unit tests for everything above.
tools/                       generators. Their output is checked in.
panel/                       generated panel geometry.
```

The HAL itself is not in the tree. `cmake/stm32cubemx/CMakeLists.txt` compiles it
out of an STM32Cube FW package located by `STM32CUBE_FW_PATH`, which is what
`toolchain.cmake` sets - written by `just arm-sdk` (see setup.md), or by hand
from `toolchain.default.cmake` if you already have STM32CubeCLT. CI fetches
the same version from ST's public mirror the same way `arm-sdk` does; the two
submodules it needs are named in `../.github/workflows/ci.yml` - that
workflow lives at the monorepo root, alongside the KiCad hardware files, not
under this directory.

## The seam

`Core/Src/Lib` is the module and nothing else. It has no peripheral access, no
globals, and no idea what is running it. The contract is one struct and two
calls:

```c
BmcvInstance m;                                  // instance.h
bmcv_instance_init(&m, &preset_io, now_us);
bmcv_instance_tick(&m, &input_sample, now_us);   // input_fold.h
```

A host fills an `InputSample` - button levels, encoder positions, slider, CV -
and reads `EngineState` back: `channels_gated_level[]` for its outputs,
`leds[]` for its lights. That is the whole interface. Everything a host has to
decide for itself, it decides outside this line.

Which files are in the core is not a matter of opinion:
`cmake/core_sources.cmake` lists them, and all four builds read that one list.
A source that is not on it does not compile into the firmware either.

**The rule that keeps this honest:** a behaviour that exists in a host and not
in `Core/Src/Lib` is a behaviour the hardware does not have. When a host needs
something the core does not expose, the answer is to widen the core's
interface, not to reimplement the behaviour on top of it.

## Why there is a `sim/` at all

Three hosts need the same handful of fiddly things, and each is easy to get
subtly wrong:

- **Unit conversion.** Volts to ADC counts and DAC counts to volts, with the
  converters' real ranges and the 14-bit converter's off-by-one.
- **Tick decimation.** The engine is dt-driven and correct at any rate, but a
  host samples audio far faster than the engine needs. `SimTickDiv` accumulates
  time in Q32 microseconds so that a 44.1kHz divider of 11 does not run the
  clock 0.17% slow.
- **Gate latching.** Hardware catches edges in the ADC's DMA callback, faster
  than the engine loop. A host that samples per frame and ticks at 4kHz has to
  latch the same way or it drops short triggers.
- **Filling an `InputSample`.** A host counts jacks the way the panel does;
  `InputSample` counts converter channels.
- **The preset slots**, which on the module are FRAM and on a host are bytes to
  put in a patch file or in browser storage.

`sim/src/sim_rt.c`, `slot_store.c` and `led_color.h` are those, in plain C with
no host's types in them, so all three hosts share one implementation and
`tests/test_sim_rt.c` and `test_slot_store.c` can exercise it with no host at
all. `sim/src/bmcv_sim.c` is a fourth host - the flat C API the wasm build and
the CLI use.

The directory is named for the simulator that came first, but its contents are
host support generally.

## The four builds

| Target | Entry point | Why it is separate |
|---|---|---|
| firmware | `CMakeLists.txt` | ARM toolchain, `cmake/gcc-arm-none-eabi.cmake` |
| tests + tools | `tests/CMakeLists.txt` | host compiler, `ctest` |
| sim + wasm | `sim/CMakeLists.txt` | same sources under `emcmake` for the web |
| Rack plugin | `vcv/Makefile` | Rack supplies `plugin.mk`; a plugin must include it |

They are four projects rather than one because their toolchains genuinely
differ, not because the code does. Each includes `cmake/core_sources.cmake` (or
parses it, in the Rack plugin's case) so the source list cannot drift.

`just check` runs everything that needs neither hardware nor a browser: the
unit tests, the golden flows, a headless load of the wasm, and a headless
import of the whole web frontend.

## Generated, not written

Anything derived is generated and checked in, so that a reader gets it without
the hardware repository to hand and a reviewer sees it change as a diff:

- `panel/` and `web/panel.json` and `vcv/res/` come from `tools/gen_panel_spec.py`,
  which merges the hardware repository's KiCad output with the firmware's own
  `HwSetup` tables. Positions, LED assignments and button roles are therefore
  derived from the board and the firmware together, and `tests/test_panel_spec.c`
  asserts the two agree. `just panel`.
- `Core/Inc/Lib/stepped_random_table.h` comes from `tools/gen_sr_table.c`.
  `just sr-table`.
- `Core/Inc/Lib/wavetables.h`, `Core/Src/Lib/wavetables.c` and the plots in
  `docs/images/wavetable-*.svg` all come from one run of
  `tools/gen_wavetable.py`, so the documentation cannot show a shape the
  firmware does not have. `just wavetable`, and `just wavetable --report` for
  the axis as sparklines. Generated rather than drawn because the properties
  that matter - full swing and no DC at *every* setting, the canonical shapes
  exact, the axis closing on itself - are true of the whole family by
  construction and were not true of a table edited slice by slice.
  `docs/wavetable.md` is the long version.
- `docs/images/` comes from `tools/`-driven screenshots of the real frontend.
  `just docs-shots`.

What genuinely cannot be derived lives in `panel/overrides.json` and is listed
under `assumptions` in the generated spec, so an assumption cannot hide.

## Testing

`tests/` holds ordinary unit tests plus two things worth naming:

- **`tests/fixtures/fixture.c`** drives a whole `BmcvInstance` from button and
  encoder events, so a test can express a gesture ("hold STA, tap scene 3")
  rather than a state.
- **`sim/flows/`** are scripted input timelines replayed through the CLI and
  diffed against committed CSV output. They cover interactions across time that
  a unit test cannot state - a clock locking, a mute ramping, a mode latching.
  Review the diff, then `just flows-bless`.
- **`tests/pll_metrics.{c,h}`** turns a run of the sync loop into numbers -
  settling, ringing, frequency pull, phase continuity, long-run alignment - so
  that a change to it reads as a trade rather than as a feeling.
  `tests/test_pll.c` prints the table; `docs/pll.md` keeps the baseline.

Two tests exist purely to stop the four builds drifting apart:
`test_panel_spec.c` (the generated geometry against `hw_setup.c`) and the LED
curve check in `web/smoke.mjs` (the C header against the JS constants).
