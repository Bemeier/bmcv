# Plan: the write direction — driving a physical module from the page

Status: **built, untested on hardware.** Everything except the USB layer itself
is covered by `just check` (29 native tests, 138 checks in `test_input_fold`
alone, plus the frontend checks). Supersedes "Picking this up later / 1. The
write direction" in `probe-bridge.md`, which called for a mailbox but not this
shape.

Goal: a click on the web panel presses the button on the module in front of you.

## Verdict

Clean. **No new abstraction in the core, and one 30-line file on the web side.**
The same argument that killed the planned `source.js` mostly killed the planned
input-sink abstraction too:

- **Core.** Everything downstream of the fold reads `HwState` and nothing else
  (`ui_input.c`, `engine.c`, `bmcv_sim.c`, `sim_rt.c` — checked, that is the
  complete list). So a second input merged in `input_fold` is invisible to all
  of it. The merge is **three expressions inside loops that already existed.**
- **Web.** `panel.js` *lost* code: six `if (mode.live) return` guards and a
  wrapper function are gone, and it no longer imports `mode` at all. What
  replaces them is `web/input.js`, which answers "whose button was that" in
  three one-line dispatches.

## The idea: levels, not events

The mailbox is **a second panel, wherever it is** — not a queue of gestures.
Every field is a level or a free-running position, never a delta to be consumed.

That one decision removes most of what made this look expensive:

| problem the old plan anticipated | why it does not arise |
|---|---|
| torn SWD writes half-applying an edit | a torn level costs one tick of a wrong value and self-corrects; nothing accumulates, so nothing is applied twice or lost |
| the mirror-image of `sim_input_adopt` — the two sides holding different baselines | the writer picks its own encoder origin and never learns the module's. See "the one thing that was not free" |
| the firmware consuming the mailbox, so the host must not resend | the firmware never writes to it; its bookkeeping is outside |
| a dropped or repeated write | idempotent by construction |

```c
// Core/Inc/Lib/input_fold.h — 48 bytes, at offset 2152 in a BmcvInstance
typedef struct
{
  uint8_t  button_down[N_BUTTONS];   // OR'd with the physical panel's
  int16_t  encoder_pos[N_ENCODERS];  // summed with it; free-running, may wrap
  int16_t  slider_raw;               // overrides while >= 0; REMOTE_SLIDER_NONE hands back
  uint32_t seq;                      // bumped on every update. Last, deliberately
} RemoteInput;
```

`InputFrames` gains `remote` plus six bytes of bookkeeping that live *outside*
the mailbox, so the writer never sees its own struct change under it.

### The one thing that was not free

A mailbox arriving or leaving adds or removes its whole encoder contribution at
once — a step, which the existing delta would read as a turn of that size.
Connecting a page whose encoder happened to sit at 1000 would have applied a
thousand detents of edit to a patch nobody touched: `sim_input_adopt`'s exact
failure, in a new place.

Fixed in `remote_refresh()` by moving `in->prev.encoder_state[]` with the step
on the tick liveness changes, so the delta comes out zero. Eight lines, and it
is what buys the writer its free choice of origin — the alternative was every
host having to learn and track the module's absolute encoder positions.
`remote_encoder_origin_is_not_a_turn` covers both directions.

### Why `seq` is required rather than optional

Its second job. Levels are safe against tearing on their own, but a writer that
*stops* leaves its levels standing — a page refreshed with a button held would
strand the module holding SHIFT with no way back but a power cycle. After
`REMOTE_TIMEOUT_US` (250ms) without a new `seq` the whole struct stops being
read. The probe writes on every poll, changed or not.

### The slider: last mover wins

The only control where the two panels can genuinely disagree, because it is an
absolute position rather than a level to OR or a movement to sum.

A new value in the mailbox takes the fader and remembers where the physical one
was sitting; the physical one takes it back by moving `REMOTE_SLIDER_RELEASE_RAW`
(120 raw, ~1.6% of travel) away from there. The threshold is what makes it work
on hardware — without it the ADC's own noise would hand control back within a
tick or two of every remote move. CV still sums into whichever fader is being
obeyed, since a patched slider input is a property of the module rather than of
the hand on the panel.

## The transport was already there

A write is one command plus data on the OUT pipe with **no reply**, so it is
cheaper than a read and rides the existing poll loop. Two transfers, split at
the last word, so `seq` lands after the fields it describes:

```
writeMem(instanceAddr + remoteOffset,      body)  // 44 bytes
writeMem(instanceAddr + remoteOffset + 44, seq)   // 4 bytes
```

**The offset comes from the wasm, for free.** `layout_target.h` asserts every
leaf field of `BmcvInstance` against the firmware ELF and its generator emits
nested-struct leaves, so regenerating covered `input.remote.*` automatically —
369 assertions now, up from 345. `offsetof(BmcvInstance, input.remote)` in wasm
*is* the ARM offset, held by the same machinery the read path relies on.

So **no change to `bmcv_probe.h` and no `PROBE_INFO_VERSION` bump.** A board
running older firmware is refused by the existing `instanceSize` check, which
already says what is wrong. Three `_Static_assert`s in `bmcv_sim.c` hold what
the transport needs: `seq` last, whole words, word-aligned start.

## The sim: a separate outgoing mailbox

`BmcvSim` gains `remote_out`, and `bmcv_sim_remote_*` fills it. The local
setters are untouched.

The tempting alternative was to route the local setters through the mailbox too,
so there would be one input path and every local session would exercise the
merge. It does not survive contact with `bmcv_sim_import`: probe mode imports
~70 times a second and each import overwrites `m.input.remote`, so the staging
copy has to be separate anyway — and re-baselining a summed encoder across an
import gets genuinely delicate, in exactly the area this codebase has been bitten
before. The merge is covered by nine native test cases instead, which is the
better place for it.

`sim_input_adopt()` is unchanged for the same reason: local input still lands in
`InputSample`, so what it re-baselines is still what needs re-baselining.

## What is deliberately not in v1

**Remote CV and gates** — decided against. The module's jacks are physical, and
overriding an ADC reading means something different from pressing a button. If
it is ever wanted it is additive: `int16_t cv_raw[N_INPUTS]` plus an override
mask, and `input_trig_step()` (which exists for exactly this) run over the
overridden channels at fold rate.

**MIDI as the transport** — unchanged from `probe-bridge.md`: a different design,
not a swap.

## What it cost

- `sizeof(BmcvInstance)` 2304 → 2368 (+64). Snapshot reads grow 2.8%.
- Firmware RAM 9680 → 9744 bytes, 7.43% of 128KB. Nothing on the hot path but
  three expressions and a branch.
- `bmcv` stayed at `0x20000f70`; the flash descriptor followed `sizeof`
  automatically (`01004009` — size 0x0940).
- Browser storage untouched: `SlotStore` holds `EngineConfig`s, not
  `InputFrames`. No saved state is dropped.
- `web/frontend-check.mjs`'s frozen descriptor fixture needed the new size. It
  caught this itself, which is what it is for.

## Left to do

**Try it on hardware.** Press a button on the page, watch the module's LED
answer. Then check the two things only a board can show:

1. Whether the write measurably moves the ~70/s snapshot rate. It should not —
   no reply to wait for — but nothing has measured it.
2. Whether `REMOTE_SLIDER_RELEASE_RAW` is set right against the real ADC's
   noise. Too low and the physical fader steals control back on its own; too
   high and taking it back needs a visible sweep rather than a nudge. 120 is
   reasoned, not measured.

## Unresolved questions

1. **Should the module show that it is being driven remotely?** It has no idea
   today, and a module reacting to nobody is a confusing sight on a bench. A LED
   tell costs UX-layer code for a debug feature; leaving it out is defensible.
2. **Does the VCV module want a scripted input path?** It gets one for free now
   — `RemoteInput` is core — but it has real ports and knobs and may never use
   it. Listed because "it exists in every host" was part of the argument for
   putting the mailbox in core rather than in the firmware.
3. **`REMOTE_TIMEOUT_US` at 250ms** — long enough to survive a hitched frame,
   short enough that a pulled cable is not felt. Only a bench session will say
   whether that is the right trade.
