# Spike: the module's own USB port as the transport

Status: **step 1 built, waiting on a bench run.** Nothing here is a feature. The
branch is `spike/midi-transport` and is meant to be measured and then largely
thrown away.

Question: can the module publish its state over the USB MIDI port it already
enumerates, instead of over a debug probe on the programming header? That would
mean no probe, no WinUSB binding, no Chromium-only restriction, and it would
work on a module already plugged into a DAW.

## What changed since `probe-bridge.md` said no

That doc rejected MIDI partly on this reasoning:

> It changes what the decoder can assume. [...] A MIDI transport would almost
> certainly send a chosen subset, which means a second definition of what that
> subset is - the exact duplication `bmcv_sim_import` was designed to avoid.

**That does not follow, and it was the load-bearing objection.** A byte-level
chunked diff is not a subset: the host reassembles a byte-identical 2368-byte
image and hands it to `bmcv_sim_import` unchanged. Neither end names a field, so
there is no second definition of anything. The rule that killed "send a subset"
simply does not apply to "send which bytes moved".

That reopens the question, and a measurement settles the rest of it.

## Measurement 1: how much of the instance moves

`bmcv_sim_export` on a running sim, snapshotted at a rate and diffed against the
previous one. (The harness reports 2408 bytes because it is native x86-64 with
8-byte pointers; the ARM and wasm figure is 2368. The percentages carry over.)

| state | changed bytes | 32 B chunks touched, of 76 |
|---|---|---|
| idle, 30 Hz | 86 (3.6%) | 17 |
| idle, 15 Hz | 89 (3.7%) | 17 |
| running, 30 Hz | 122 (5.1%) | 20 |
| running, 15 Hz | 118 (4.9%) | 20 |
| running, 5 Hz | 128 (5.3%) | 21 |

**About 5% of the instance changes between frames, and the figure barely moves
with the rate.** The changes are scattered rather than contiguous - 122 changed
bytes touch 20 different 32-byte chunks - so chunking wins less than the raw
percentage suggests, but it still wins by a lot:

| | payload | on the wire | 64 B transfers |
|---|---|---|---|
| whole instance | 2368 B | 3608 B | **57** |
| 32 B chunks that changed | 680 B | 1036 B | **17** |

Both after 7-bit SysEx encoding (×8/7) and USB-MIDI packing (3 payload bytes per
4-byte event packet).

## Measurement 2: what the endpoint is actually worth — NOT YET DONE

This is what the branch is for, and it is the number everything hangs on. The
transport facts are known:

- The MIDI endpoints are **bulk, 64 bytes** (`MIDI_EPIN_SIZE 0x40`), `bInterval`
  ignored.
- The IN endpoint is **single-buffered**: `midi_idle()` must be true before the
  next send, and the main loop refills it.

What is *not* known is what a browser's Web MIDI stack, the OS class driver and
the host controller sustain between them. None of that is in this repo and none
of it is guessable, so the spike measures it rather than arguing about it.

### How

`SYSEX_CMD_BENCH_REQ` (0x10) makes the module send `SYSEX_BENCH_MESSAGES` (2000)
SysEx messages as fast as the endpoint takes them. Each is **48 MIDI bytes =
16 USB-MIDI event packets = exactly one 64-byte transfer**, so counting messages
in the browser counts transfers on the endpoint, and bytes/second needs no
assumption about how anything packed anything. `tests/test_sysex.c` holds the
firmware to that alignment - it is the one way this experiment could produce a
confident wrong answer.

The burst is **bounded in the firmware** rather than being a start/stop pair: a
tab that goes away mid-run cannot leave the module streaming at nobody, which
would need a power cycle to stop. Ask again for a longer run.

`bmcv.c` holds back its own control changes while a burst is running
(`midi_bench_active()`), so what is measured is the endpoint's ceiling and not
the endpoint sharing itself with the engine.

```
just build-flash-rel      # flash a module with the bench command
just web                  # then open /midi-bench/
```

The page reports messages/s, payload and wire bytes/s, and converts those into
the two numbers that matter: whole snapshots/s and chunked deltas/s, against the
probe's ~70.

### Reading the result

- **delta ≥ 50/s** — comfortable; build it.
- **delta 20–50/s** — workable. Below the probe but well above what a panel
  needs to look alive. Whole snapshots would not be.
- **delta 5–20/s** — marginal; only a delta scheme gets there at all.
- **delta < 5/s** — a readout, not a display. Stop.

## If it passes: what to build, in order

### 1. The write direction (small, and useful on its own)

**~50 lines.** The remote input mailbox already exists and the module already
merges it in `input_fold`; all that is missing is a SysEx that carries 48 bytes
into `bmcv.input.remote`. At 2 transfers per update this is free at any rate the
measurement could plausibly report.

Worth doing even if the read direction never happens: it means a module with
nothing but a USB cable in it can be driven from the page.

Needs `SYSEX_MAX_LEN` (16) revisited - the parser currently skips anything
longer without buffering it.

### 2. The read direction

Bigger, and the honest cost against the probe's *zero* firmware lines:

- A shadow copy of the instance to diff against (2368 B), plus the outgoing
  frame (2368 B). RAM is fine: 7.5% of 128 KB used today.
- Chunk differ, 7-bit encoder, SysEx framer, send state machine.
- Keyframe and resync: a host that joins mid-stream, or misses a chunk, needs a
  way back to a known state. Periodic full snapshot, or on request.
- 150–250 lines, on the same core running the engine at 4 kHz.

**One thing it would do better than the probe.** Diffing against a copy taken at
a tick boundary makes a reassembled snapshot internally consistent. The probe's
read never is - `probe-bridge.md` documents that a blob can straddle a tick and
warns against relying on cross-field consistency. This transport could offer
what that one cannot.

**And one it would do worse.** It shares the IN endpoint with the engine's own
MIDI output, which `midi_out` reconsiders every 2 ms. A snapshot stream large
enough to be useful will delay control changes. Needs a priority rule, or for
streaming to be a mode you turn on rather than something that runs always.

## Open questions

1. **Firefox.** Half the reason to do this is escaping Chromium, but I have not
   verified the current state of Firefox's Web MIDI SysEx permission. Worth
   checking before the result is used to justify anything - if SysEx is awkward
   there, the main non-technical argument for MIDI weakens a lot.
2. **Does the probe stay?** If MIDI is fast enough, the probe path is still the
   only one that costs the firmware nothing and the only one that works on a
   module whose firmware is wedged. Probably both, with the frontend showing
   whichever is connected - `mode.captureHz` already exists so the scopes follow
   whatever rate a transport manages.
3. **Streaming always, or on request?** Bears on the endpoint conflict above,
   and on whether a module in a DAW is quietly spending its MIDI bandwidth on
   nobody.
4. **Is 32 bytes the right chunk?** 16 B chunks touch 28 of 151; 64 B touch 14
   of 38. 32 looked best on the numbers above but it was not tuned, and the
   index width trades against it.
