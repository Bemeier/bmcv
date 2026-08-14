# Plan: drive the web frontend from a physical module over SWD

Status: working. A physical module drives the page over an ST-Link at around 70
snapshots a second. Everything except the USB layer itself is covered by
`just check`.

Goal: point the existing web frontend at a running board instead of at the wasm
sim, so the scopes, LED panel, channel table and readouts show the hardware.
Read first; write later.

## Verdict

Feasible, and cheaper than it looks. The seam this needs is already built:

- **The whole module is one struct at one address.** `nm build-rel/BMCVFirmware.elf`
  says `bmcv` is `0x900` = **2304 bytes** at `0x20000f70`. `bmcv.h` already says
  it is external "purely so a live debugger has something to attach to".
- **Everything the frontend draws is inside those 2304 bytes.** LED framebuffer,
  `channels_effective`, `channels_gated_level`, clock, `active_scene`, `error_flags`,
  `engine_config`, `ui_state`, the folded input frame.
- **`bmcv_sim.c` already turns that struct into the flat API the frontend uses.**
  Its `capture()` is the only thing deriving display state from `s->m`. Give it a
  snapshot instead of a tick and every accessor works unchanged.

So the wasm module stops being "a simulated module" and becomes **the decoder for
a real one**. No struct parser in JS, no duplicated unit conversions, no second
copy of the LED curve. That is the whole trick.

2304 B at 30 Hz is 69 KB/s over SWD; at 60 Hz, 138 KB/s. Within an ST-Link V2's
block-read budget, comfortable on a V3.

Firmware cost for read-only: **zero lines.**

## Read path

The browser talks to the ST-Link itself, over WebUSB. No local process:

```
board --SWD--> ST-Link --WebUSB--> browser
                                      |
                        memcpy into wasm heap
                                      |
             existing bmcv_sim_* accessors -> UI
```

**Decode** is done: `bmcv_sim_import()` takes the blob, re-points the instance's
pointers at itself, and republishes every reading. See "What is built" below.

**Reads** are two USB transactions: an ST-Link `READMEM_32BIT` command out, the
data in. A Cortex-M's memory is readable over the AHB-AP without halting the
core, so the module keeps playing while it is watched. 2304 bytes is one
transfer — the limit is around 6KB — so a snapshot costs about as much as a
mouse move, and 30-60Hz is unremarkable. `WRITEMEM_32BIT` is the same shape,
so the write direction later costs nothing in transport.

The protocol subset needed is small: identify the probe, enter SWD mode, read
memory, write memory. It is stable — openocd's `stlink_usb.c` is the reference
and it has not moved in years — and it has been done in a browser before
(devanlai's `webstlink` is exactly this). Call it ~300 lines we own.

### Why not a local bridge process

The alternative was openocd behind a WebSocket. It is less code for us and
someone else maintains the probe support, but:

- **It needs a process running.** WebUSB needs a page and a click. That is the
  difference between "open the page, plug in the module" and "install openocd,
  run `just probe`, then open the page" — and it means the *hosted* frontend can
  drive real hardware, which is a different product rather than a nicer setup.
- **It solves this machine specifically.** The tooling here is on the Windows
  side (`scripts/flash.sh` shells out through `cmd.exe`), so a Linux-side bridge
  would need `usbipd-win` to attach the probe into WSL first. Windows Chrome
  talking to a Windows ST-Link skips the problem rather than working around it.

What it costs: **WebUSB is Chromium only** — narrower than the Web MIDI caveat
the page already carries, which at least covers Firefox. And on Windows the
device has to be bound to WinUSB for Chrome to claim it, which is the one step
that may need Zadig. Neither is fatal, and the bridge stays available as a
second implementation of the same three calls (`open`, `readMem`, `writeMem`)
if Firefox or a stubborn machine ever makes it worth writing.

### The one thing WebUSB cannot do: find the address

With a bridge, `nm -S <elf> | grep ' bmcv$'` gives the address and size. A
browser has no ELF and no `nm`, and the address is not stable — `bmcv` is at
`0x20000f70` in today's build and moves whenever anything before it in `.bss`
changes size. **Built.** The firmware says where it is, at an address that does not move:
`Core/Inc/Lib/bmcv_probe.h` declares a 28-byte `const` and the linker script
pins it to `0x08000200`, just past the vector table's `0x080001D8`.

```
8000200  424d4356 01000009 700f0020 302e3130   BMCV....p.. 0.10
8000210  2e300000 00000000 00000000            .0..........
         magic    v1  2304 &bmcv     "0.10.0"
```

The browser reads 28 bytes from one known address and has everything: whether
this is a BMCV at all, where the instance is, how many bytes to take, and which
firmware wrote it. Nothing on any execution path - a const in flash, the same
bargain `bmcv.h` already makes by keeping the instance `extern` "purely so a
live debugger has something to attach to". Two linker `ASSERT`s hold the address
and the size to what the header declares, so the two cannot drift, and the
second also catches the vector table ever growing past `0x200`.

That the address genuinely moves is not hypothetical: the same source at two
optimisation levels puts `bmcv` at `0x20000ea8` (Debug) and `0x20000f70`
(RelWithDebInfo). Anything that hardcoded one would read the other's `.bss`.

The alternatives were the same descriptor found by scanning flash for the magic
(no linker change, ~60ms at connect) and uploading the ELF to parse its symbol
table in JS (no firmware at all, but friction every session and nothing stops
someone picking an ELF that is not what is flashed - precisely the failure the
descriptor exists to catch).

### The frontend, and the abstraction that was not needed

The plan called for a `source.js` that picks between a local and a probe-backed
implementation of the whole `sim` interface, with every module importing from it
instead of `sim.js`. That turned out to be the wrong shape.

The wasm instance *is* the abstraction. `bmcv_sim_import()` writes hardware state
into it, so `leds.js`, `scope.js`, `readouts.js` and the rest keep reading `sim`
and get the module without knowing anything changed. Not one of them was touched.
What decides the source is only who fills the instance: the frame loop's
`runTicks`, or the probe's timer. So instead of a source abstraction there is
`web/mode.js` - a boolean, and the two questions that hang off it:

- **The frame loop does not tick a module that is already running**, and does not
  drain its MIDI queue (the hardware is sending that itself, over its own USB
  port) or mirror its patch into browser storage.
- **The panel's inputs stand down.** Anything typed in would be overwritten
  before it was drawn. The cursor says so.

Two things did have to stop being drawn from the frontend's own bookkeeping,
because that bookkeeping only tracks this page's mouse: the encoder spokes
(`encAngle` is gone; `bmcv_sim_encoder_pos` replaces it, in both modes) and the
crossfader handle (`bmcv_sim_slider01`, in probe mode only - what comes back is
the slider *after* input_fold sums CV into it, which on hardware is the closest
to the truth available and locally would be a downgrade).

Leaving a probe session restores the simulation that was running before it, via
`bmcv_sim_export` on connect. Without that, disconnecting left the simulator
holding the module's patch and autosaving it over the browser's copy two seconds
later.

### What is built

`bmcv_instance_wire()` (`instance.c`) is the list of every pointer in the
instance, extracted from `bmcv_instance_init` and called by it. `bmcv_sim_import`
calls the same function on a blob that arrived from somewhere else.

```c
int32_t bmcv_sim_instance_size(void);
void    bmcv_sim_export(const BmcvSim* s, void* dst);
int32_t bmcv_sim_import(BmcvSim* s, const void* src, int32_t len);
```

`import` = length check, `memcpy`, re-wire, adopt the snapshot's clock and panel
positions, `capture()`. Exposed through `web/sim.js` as `importInstance` /
`exportInstance` over a buffer allocated once at load.

One thing the first draft of this plan did not anticipate: **an `InputSample` is
the host's, not the instance's.** It survives the copy and then disagrees with
the hardware frame that arrived, and `input_fold` reads the difference as input
— import a module whose ENC 3 sits forty detents from this host's and the first
tick applies forty detents of edit to a patch nobody touched. `sim_input_adopt()`
re-baselines the panel positions onto the ones that came in. Encoders and buttons
come back exactly; the slider is off by whatever slider CV is patched, and
self-corrects on the next tick. Volts on the input jacks stay the host's, because
those are its patch rather than the module's.

That only matters for a host that imports *and then runs*, which probe mode does
not — but the API allows it, it is how a module gets cloned into the simulator to
carry on playing, and it was silently wrong.

`tests/test_sim_import.c` covers it in two halves: what a snapshot publishes
without a tick, and whether it still matches the original after both are run on.
The second half is the one that catches a pointer left aimed at the module that
produced the blob, since every one of them is followed during a tick.

### Layout equivalence: measured, and it does not hold today

**Done.** The blob is written by `arm-none-eabi-gcc` and read by wasm32 clang.
Both are little-endian ILP32 with 4-byte pointers, so the layouts look like they
should match. They do not:

| build | `sizeof(BmcvInstance)` |
|---|---|
| ARM (`build-rel/BMCVFirmware.elf`, authoritative) | **2304** |
| wasm32 (emcc, as built today) | **2360** |
| native x86-64 | 2400 — not a valid stand-in either way, 8-byte pointers |

**One cause: enum width.** `arm-none-eabi-gcc` defaults to `-fshort-enums`, so an
enum whose values fit in a byte *is* a byte. Clang gives it four. Every one of
the 19 divergences traces back to three struct members:

```
EngineConfig.input_mode[N_INPUTS]        InputMode            4 bytes -> 1
ChannelConfig.input_amp_mode             ChannelInputAmpMode  4 bytes -> 1
ChannelConfig.quantize_mode              ChannelQuantizeMode  4 bytes -> 1
```

`ChannelConfig` is 91 bytes on ARM and 97 on wasm; everything after
`engine_config` shifts. Nothing else in the instance diverges — `EngineState`,
`UiState`, `InputFrames` and `MidiOut` are already identical byte for byte, and
`UxState` matches once pointers are 4 bytes.

#### How the check was made, and how to repeat it

- `tools/dump_layout.py` runs under `arm-none-eabi-gdb-py3` and walks
  `BmcvInstance` through the firmware ELF's debug info, printing all 172 leaf
  fields as `path / offset / size`. The ELF is the authority: it is the layout
  the probe actually reads out of RAM.
- `tools/gen_layout_asserts.py` turns that into 345 `_Static_assert`s over
  `offsetof`/`sizeof`. Compile the header in another build and a divergence is a
  compile error naming the field.

```
just layout-check     # after `just build-rel`
```

The generated `sim/include/layout_target.h` is checked in and included by
`sim/src/bmcv_sim.c`, so the wasm build - and therefore `just check` - is where
the agreement is enforced. It is guarded to targets with four-byte pointers, so
the native build ignores it.

Control: compiled against native x86-64 it fails 181 assertions, so the check
has teeth rather than passing vacuously.

#### Two fixes, both verified to give a clean compile — option 2 landed

1. **`-fshort-enums` on the sim build.** One line in `sim/CMakeLists.txt`. All
   345 assertions pass. But it fixes only the build it is added to, so the wasm
   and Rack builds would then disagree with each other about `EngineConfig`.
2. **Make the three fields `int8_t`, with the enum named in a comment**
   (recommended). All 345 assertions pass *with no compiler flag*. This is
   already the codebase's own idiom — `shape_mode` and `clamp_mode` sit in the
   same struct as `int8_t` with exactly this comment, and `config.h` already
   explains why: "Persisted as a plain int8_t in ChannelConfig, so appending
   modes here does NOT change the FRAM layout".

Option 2 changes nothing on ARM — those fields are already one byte there — so
**the FRAM record format is untouched and `CONFIG_STATE_VERSION` does not move**.
It only brings the other three builds into line, and it fixes all of them at
once. Cost is casts at the call sites that assign or switch on those fields.

#### One consequence either way

`SlotStore` holds raw `EngineConfig`s, so the sim's storage blob shrinks from
6400 to 5920 bytes. `storage.js` rejects a blob of the wrong length, so **saved
browser state is dropped once**. Acceptable, and it should be a deliberate
one-line note in the changelog rather than a surprise.

A smaller bonus than it first looked: a single `EngineConfig` is now byte-identical
across all four builds, so one FRAM record's payload could be transplanted into a
host. `SlotStore` is still not a chip dump - it has no record headers - so
`slot_store.h`'s "not interchangeable" stands as written.

#### Still worth building

A `bmcv_layout_id` — a hash over the same offset table, compiled into the
firmware as a `const uint32_t` and exposed from the wasm as
`bmcv_sim_layout_id()`. The bridge reads the target's on connect and refuses a
mismatch. Static assertions catch a *compiler* divergence at build time; this
catches the common runtime one, a board flashed from a different commit than the
page was built from.

Wire the assertion header into the wasm build and CI as `just layout-check` so
the agreement cannot rot. If it ever does break for a reason that is not fixable
in a struct, the fallback is field-wise transfer driven by the ELF's DWARF, which
costs the bridge a DWARF reader — worth avoiding, worth knowing is there.

## Frontend

`sim` is already the single interface object — `panel.js`, `leds.js`, `scope.js`,
`readouts.js`, `midi.js` all import it and touch nothing else. So the change is
to make its *source* swappable, not to invent an abstraction:

```
sim.js       the wasm module + accessors        (unchanged)
local.js     inputs -> sim; run() ticks         (today's behaviour)
probe.js     inputs -> bridge; run() no-op; snapshots imported
source.js    picks one, exports `sim`
```

`probe.js` shares the same wasm instance for decoding, so it is thin: replace
`run()` with `importSnapshot(bytes)` and route the input setters.

Two things need widening, both improvements in their own right:

- **Panel state must come from the module, not from JS.** `panel.js` accumulates
  encoder angle in its own `encAngle` map and `main.js` pushes the slider position
  in. On hardware those live in `bmcv.input.curr`. Add `bmcv_sim_slider01()`,
  `bmcv_sim_button_down(i)`, `bmcv_sim_encoder_pos(i)` reading `hw_state`, and
  have `panel.js` draw from them in both modes. One source of truth, which is what
  `docs/architecture.md` asks for anyway.
- **Scope capture must be callable without a tick.** `import` calls `capture()`,
  so the scope ring fills at the poll rate.

## What does not survive the wire

Say these in the UI rather than letting someone misread a graph:

- **Scope resolution.** The sim's scope is one sample per 4kHz tick. Over a probe
  it is one sample per poll — 30–60 Hz. Fine for envelopes and slow LFOs,
  useless for anything near audio. Fixing it properly means a ring buffer in the
  firmware that the bridge drains, which is firmware code for one feature; not in
  scope for v1.
- **A snapshot is not atomic.** The core keeps running during the read, so a blob
  can straddle a tick. At 2.3 KB the window is short and nothing on screen is
  sensitive to it; do not build anything that assumes cross-field consistency.
- **Timestamps** are the target's TIM2 microseconds, unrelated to the page's
  clock. In probe mode nothing ticks the wasm, so this never matters — but it
  does mean the two modes must not be mixed in one instance.

## Write path

Read-only is the deliverable. For writing, three options, worst to best:

1. **Poke `InputSample`.** Useless — the firmware refills it from real hardware
   every tick.
2. **Poke `engine_config`.** Works: the engine re-reads config every tick, and a
   2-byte param write is atomic enough. But it bypasses the UX layer, so no
   sparkle feedback, no `channels_last_delta`, no selected-param follow, and
   `config_validate` only runs at boot — the browser could write a value the
   module would never produce. Cheap, and permanently a bit of a lie.
3. **A remote-input mailbox in the core** (recommended). A small struct in
   `InputFrames` that `input_fold` merges into the sample: button levels to OR in,
   encoder detents to add, an optional slider override. Host writes the fields
   then bumps a sequence counter; `input_fold` acts only on a changed counter and
   copies before acting, so a non-atomic SWD write cannot be half-applied.

   ~20 lines in `input_fold.c`, testable natively, and it exists in every host —
   so it is a core capability, not a firmware special case. The module then reacts
   to the browser exactly as it reacts to a finger: UX feedback, LED response,
   autosave, all of it. It also gives the VCV module and the flows a scripted
   input path they do not have today.

Option 3 is the only one that satisfies `architecture.md`'s rule — "a behaviour
that exists in a host and not in `Core/Src/Lib` is a behaviour the hardware does
not have".

## Picking this up later

### 1. The write direction

Everything under it is already there. `writeMem()` works and is exercised by the
same round trips reads use; `#calibrateBlocks` proves the transfer size; the
frontend already knows when a module is driving the page (`mode.live`). What is
missing is entirely core-side.

The three options were weighed above; the conclusion has not changed, and the
reasoning is worth keeping because the cheap option is genuinely tempting:

- **Poking `engine_config` directly** would work today, with no firmware change
  at all, and is the wrong answer. The engine re-reads config every tick so a
  2-byte parameter write lands - but it bypasses `ux_note_channel_edit`, so a
  stepped channel's pattern length changes under the playhead; it bypasses the
  UX layer, so no sparkle, no selected-param follow, no LED feedback; and
  `config_validate` only runs at boot, so the browser could write a value the
  module itself would never produce. It would look right in the simulator and be
  subtly wrong on hardware, which is the worst failure mode this project has.
- **A remote-input mailbox in `input_fold.c`** is the one to build. A small
  struct in `InputFrames`: button levels to OR in, encoder detents to add, an
  optional slider override, and a sequence counter. The host writes the fields
  and then bumps the counter; `input_fold` acts only on a changed counter and
  copies before acting, so a non-atomic SWD write cannot be half-applied. About
  20 lines, testable natively, and present in every host - so it is a core
  capability rather than a firmware special case, and the VCV module and the
  flows get a scripted input path they do not have today.

Two things learned since that will matter:

- `sim_input_adopt()` already solves the mirror-image problem on the way in. The
  same reasoning applies going out: the panel positions a host holds and the
  ones the module holds are different numbers, and whichever direction they
  cross in, the delta is what gets read as a gesture.
- A write is one command plus data on the OUT pipe with no reply, so it is
  cheaper than a read. It can ride in the same poll loop without measurably
  moving the ~70/s snapshot rate.

### 2. MIDI as the transport instead

Worth considering, and not a drop-in. The module already enumerates as USB MIDI
and already speaks SysEx (`Core/Inc/Lib/sysex.h`), so the physical path exists
and needs no probe, no WinUSB, no Chromium - Web MIDI is in Firefox too.

What makes it a different design rather than a swap:

- **It is not free.** The probe reads RAM with no firmware involvement at all;
  the whole reason this cost zero firmware lines is that nobody had to serialise
  anything. Over MIDI the module has to assemble and send its own state, on the
  same core that is running the engine, at whatever rate is asked of it.
- **Bandwidth.** SysEx is 7-bit, so 2304 bytes becomes ~2633 bytes on the wire
  plus framing. USB-MIDI packs 3 data bytes per 4-byte packet, and the endpoint
  is polled once per frame - order 10-20 KB/s in practice. That is a handful of
  full snapshots a second against the probe's 70, so it would want to send
  deltas or a subset rather than the whole instance.
- **It changes what the decoder can assume.** The wasm side currently takes a
  byte-identical `BmcvInstance` and re-points nine pointers. A MIDI transport
  would almost certainly send a chosen subset, which means a second definition
  of what that subset is - the exact duplication `bmcv_sim_import` was designed
  to avoid. If it is built, send the raw struct bytes and keep `import` as the
  only decoder.
- **What it buys** is every browser, no probe, and a path that works on a module
  that is already plugged into a DAW. That is a real product difference, not
  just a convenience.

A sensible shape if it happens: keep the probe as the high-rate path, add MIDI
as a low-rate one behind the same `readMem`-shaped interface, and let the
frontend show whichever is connected. `mode.captureHz` already exists precisely
so the scopes follow whatever rate the transport manages - a 5Hz MIDI link would
draw correctly without touching `scope.js`.

### 3. Smaller things left on the floor

- The scopes over a probe are one sample per snapshot. A capture ring in the
  firmware that the probe drains in bulk is the only way to see a real waveform;
  it is firmware code on the hot path for one feature, which is why it was left
  out.
- `probe.js` polls from the main thread, so rendering and polling compete. A
  worker would decouple them, but `navigator.usb` in workers has patchy support.
- The rate tops out around 70/s, which is WebUSB's per-transfer overhead rather
  than anything in this code. Six USB round trips per snapshot became one; there
  is no further arithmetic to win.

## Phases

1. ~~Layout equivalence experiment (ARM vs wasm32 offsets).~~ **Done** — they
   differ, and both fixes are verified. See above.
2. ~~Land the fix (three fields to `int8_t` + call-site casts), regenerate the
   header, wire it into the wasm build and `just layout-check`.~~ **Done** - see
   the section above. `just check` passes; the ARM build is byte-identical to
   before (`bmcv` still 0x900 at 0x20000f70).
3. ~~`bmcv_instance_wire` extraction + `bmcv_sim_import` / `_export` /
   `_instance_size`, native test, wasm exports, `web/sim.js`.~~ **Done.**
4. ~~The flash descriptor: `bmcv_probe.h`, the `.probe_info` section and its
   linker asserts.~~ **Done.**
5. ~~`web/probe/stlink.js` — ST-Link over WebUSB behind `open`/`readMem`/`writeMem`.~~
   **Done, and working on hardware** at ~70 snapshots a second.
6. ~~`probe.js`, connect button, `hw_state` accessors, panel drawing from the
   module.~~ **Done.**
7. (Later) the remote-input mailbox and the write direction. `writeMem` is
   already there, so this is core-side work rather than transport.

## Reconnecting after a refresh

Worth writing down, because it cost three wrong theories and the right answer is
not guessable from the symptom.

A page that goes away without disconnecting - a refresh, a closed tab - does so
with a read outstanding: the poll loop asks for 2304 bytes seventy times a
second, so there is almost always one in flight. The ST-Link has begun handing
that reply over and is waiting for the rest of it to be collected. **Its
firmware will not look at the command pipe until that finishes.** So the next
session's first command is not answered wrongly, it is not answered at all - and
nothing on the target end changes that, which is why power-cycling the module
did nothing and unplugging the probe fixed it.

The symptom is a hang, and the two things that look like the cause are not:

- *A stale reply in the buffer.* Would return six bytes of nonsense to the
  version read, not silence. Ruled out by the error naming a timeout.
- *A data-toggle mismatch.* Plausible, and `clearHalt` is kept for it, but it
  was not the cause - fixing only that changed nothing.

What works is collecting the abandoned reply. One 64-byte packet at a time,
because 2304 is an exact multiple of 64 and a bulk read asking for more waits
forever for a terminating short packet that never comes.

The trap in doing so, which broke every *healthy* connect for a round: a read
issued against an empty pipe never completes, and a WebUSB transfer cannot be
cancelled. Left pending it swallows the first real reply of the session - the
exact failure being undone. The only thing that cancels it is closing the device
it belongs to, so the drain drops the handle and retakes it when a read comes
back empty. `web/frontend-check.mjs` asserts that branch still does.

## Environment note

This machine is WSL2 and the ST-Link tooling is on the Windows side —
`scripts/flash.sh` and `where.sh` both shell out through `cmd.exe`. Per-invocation
`STM32_Programmer_CLI` startup is ~1s, so it cannot poll. Two ways round it:
`usbipd-win attach` the ST-Link into WSL and run openocd natively, or run the
bridge as a Windows process and connect to `ws://localhost` (WSL2 shares
localhost). The first is tidier and should be tried first.

## Open questions

1. Read-only v1, or hold v1 until the mailbox lands so the page is never a
   look-but-don't-touch mode?
2. Is the low-rate scope acceptable, or is a firmware-side capture ring buffer
   in scope after all? It is the one thing probe mode is visibly worse at.
3. Should probe mode be the same page with a toggle, or a separate entry point?
   Same page argues for side-by-side comparison (board vs sim, same patch) later;
   separate keeps `main.js`'s frame loop from growing a mode branch.
4. Chromium-only acceptable? WebUSB is not in Firefox or Safari. The page
   already warns about Web MIDI; this is a narrower version of the same note.
5. Should `web/update/` show the version out of `.probe_info` when a probe is
   attached? It reads the same string over SysEx today, and the two must agree.
