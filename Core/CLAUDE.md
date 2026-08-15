# Core

`Inc/Lib` + `Src/Lib` are **the module**: portable C, compiled unchanged into
the firmware, the tests, the wasm simulator and the Rack plugin.
`Src/*.c` and `USB_Device/` outside `Lib/` are the STM32 half — HAL, DMA, SPI,
USB — and are ARM-only.

Which files are core is not opinion: `cmake/core_sources.cmake` lists them,
split into `BMCV_CORE_SOURCES` (hardware-free) and `BMCV_DRIVER_SOURCES`.
A file not on that list compiles into nothing.

## The interface

```c
BmcvInstance m;                                  // instance.h
bmcv_instance_init(&m, &preset_io, now_us);
bmcv_instance_tick(&m, &input_sample, now_us);   // input_fold.h
```

A host fills an `InputSample` and reads `EngineState` back —
`channels_gated_level[]` for outputs, `leds[]` for lights. That is all of it.

## Rules

**State shared with an interrupt uses `IsrFlag`** (`helpers.h`), never a plain
`uint8_t`. A plain global is one the compiler may keep in a register, and
`if (flag) { work(); flag = 0; }` also drops any interrupt that lands during
`work()`. Take the flag *before* the work it asks for:

```c
if (isr_flag_take(&mcp_poll)) { if (!mcp_read()) isr_flag_set(&mcp_poll); }
```

A value an interrupt *produces* rather than requests — a MIDI clock byte — is a
count, not a flag, and needs `__atomic` on both sides. Single-writer-per-field
is what keeps that sound; see `usblink.c`'s two credit counters.

**No file-statics that hold module state.** Everything lives in `BmcvInstance`.
The clock and the error flags were the last two globals and both moved into
`EngineState`; that is what lets a host run several modules, a test build one
per case, and a probe ship the whole module to a browser as bytes.

**Never use an enum type as a struct field.** `arm-none-eabi-gcc` defaults to
`-fshort-enums` and clang does not, so an enum field is a different width in
the firmware than in the wasm build and `sim/include/layout_target.h` fails.
Declare `int8_t` and name the enum in a trailing comment:

```c
int8_t shape_mode;     // ChannelShapeMode
```

**`ChannelConfig`/`EngineConfig` are the packed FRAM record.** Changing a field
type, order or size invalidates every saved preset. If it must change, bump
`CONFIG_STATE_VERSION` (config.h) and add a migration in `config_migrate.c`
with a case in `tests/test_config_migrate.c`.

**Parameters are scene-indexed** (`params[N_SCENES][CH_PARAM_COUNT]`), because
the crossfader blends all forty-eight at once. Do not move one onto the channel.

**The UI reads gestures, never raw buttons.** `ui_input.h` owns every
press-duration threshold (`UI_T_DEBOUNCE/HOLD/LONG/VLONG`) and derives
DOWN/UP/TAP/HOLD/LONG/VLONG edges. Five scattered thresholds is how the render
path and the input path came to disagree about what a long press was.
`ui_input_update` must run every engine tick; the UX layer runs slower.

**Modes are data, not switch arms.** A new shift mode should be one row in
`ui_mode.c`'s table. If it needs an edit to `ui_render.c`, `scene.c` or
`channel.c`, the descriptor is missing something and that is where it goes.

**The engine is dt-driven and must stay correct at any tick rate.** Hosts tick
at whatever their audio callback gives them.

**No allocation, no `printf`, no `float` formatting in the hot path.** The tick
runs at 4kHz across eight channels.

## Gotchas

- `held_us` is measured from the press tick and includes the release tick, so
  `hw_state->time - held_us` is the instant of the press on every tick. Code
  comparing a press against an event timestamp depends on that.
- Pattern length and the PLL alignment period are both latched and re-taken
  only on a cycle wrap. Changing either mid-cycle jumps the output.
- `stepped_random` is stateless and deterministic by contract — same arguments,
  same value — which is what lets the caller re-derive phase from the PLL every
  tick and blend scenes without drift.
