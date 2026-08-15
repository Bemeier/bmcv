# sim

Host support for the core, plus a headless CLI. Named for the simulator that
came first, but its contents are host support generally: the wasm build, the
Rack plugin and the CLI all sit on it.

```
just sim                    # native -> build-sim/bmcv_sim_cli
just wasm                   # emscripten -> web/bmcv.js + web/bmcv.wasm
just flows                  # replay every flow, diff against its golden
just flows-bless            # accept the new output (review the diff first)
just sim-run --script=sim/flows/demo_amp.txt --emit=all
```

## What lives here and why

Three hosts need the same fiddly things, each easy to get subtly wrong, so they
are written once in plain C with no host's types in them:

- `sim_rt.c` — volts↔converter counts; tick decimation (`SimTickDiv` accumulates
  Q32 microseconds so a 44.1kHz divider of 11 does not run the clock 0.17%
  slow); gate latching, because hardware catches edges in the ADC's DMA
  callback and a host sampling per frame would drop short triggers; filling an
  `InputSample`, which counts converter channels where a panel counts jacks.
- `slot_store.c` — the preset slots: FRAM on the module, bytes in a patch file
  or `localStorage` on a host.
- `led_color.h` — the LED curve, so no host reimplements it.
- `bmcv_sim.c` — the flat C API the wasm and the CLI use, and
  `bmcv_sim_import()`, which adopts a real module's snapshot bytes.

`tests/test_sim_rt.c` and `test_slot_store.c` exercise these with no host at
all.

## Rules

**`layout_target.h` is generated — never hand-edit it.** It asserts the offset
and size of every leaf field of `BmcvInstance`, read from the firmware ELF's
debug info, and is compiled into the wasm. That is what lets a probe's raw
snapshot be decoded by a differently-compiled build. If it fails, the two
compilers have stopped agreeing about the struct — fix the struct, do not relax
the assertion. Regenerate with `just build-rel && just layout-check`.

**New readings get exported from here**, not parsed in JavaScript. See
`web/CLAUDE.md`.

## Golden flows

`flows/*.txt` are scripted input timelines; `flows/golden/*.csv` is what the
module did. They cover interactions across time that a unit test cannot state —
a clock locking, a mute ramping, a mode latching.

Script format is one command per line, times non-decreasing; a `#!args` line
sets that flow's CLI arguments. Full grammar at the top of
`src/bmcv_sim_cli.c`.

```
0ms     slider 0.0
100ms   ctrl QNT 1        # hold the QNT shift button
1s      enc 3 +4          # 4 detents clockwise on encoder 3
2s      end
```

**A failing flow is a report, not a nuisance.** Read the diff and decide
whether the change was intended before `just flows-bless`. When the behaviour
genuinely changed, update the flow's header comment in the same commit so the
file still says what it is demonstrating.
