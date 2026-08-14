# Spike: the module's own USB port as the transport

Status: **measured, and it passes comfortably.** The bench is on
`spike/midi-transport`; what it decided is below. The bench command itself is
still throwaway.

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

## Measurement 2: what the endpoint is worth — DONE

**5283 transfers/s. 330 kB/s on the wire, 217 kB/s of useful payload.**

Measured on Chromium, engine running at 4 kHz throughout, module's own control
changes stood down for the duration.

| | |
|---|---|
| messages/s (= transfers/s) | 5283 |
| per 1 ms USB frame | 5.3 transfers |
| on the wire | 330 kB/s — 28% of full-speed bulk theoretical |
| useful, after 7-bit encoding is undone | 217 kB/s |
| **whole snapshots/s** | **93.7** |
| chunked deltas/s | 326 |

The transport facts behind it: the MIDI endpoints are bulk, 64 bytes
(`MIDI_EPIN_SIZE 0x40`), and the IN endpoint is single-buffered — `midi_idle()`
must be true before the next send, and the main loop refills it. Five transfers
a frame means the host schedules bulk aggressively and the main loop keeps up
with it *while ticking the engine*, which is the part that could not be assumed.

### What it changes

**93.7 whole snapshots a second beats the probe's ~70.** So the delta scheme
that the 5%-changed measurement was collected to justify is **not needed**, and
with it goes almost everything expensive about the read direction:

- no shadow copy of the instance
- no chunk differ, no chunk indices
- **no keyframe or resync logic** — the hard part, and the one most likely to
  produce a wrong-looking module rather than an error

What is left is: copy the instance, 7-bit encode, frame, stream. The delta
scheme stays on the shelf as headroom if the endpoint ever needs sharing more
carefully, not as a prerequisite.

**One SysEx per snapshot.** 2707 encoded bytes is one SysEx message, and Web
MIDI does not deliver a message until its F7 arrives — so the browser's own MIDI
stack does the reassembly and the JS side has *no framing code at all*. It
receives one message and hands it to `bmcv_sim_import`. The "one decoder"
property is not merely preserved, it is trivial.

**Sharing with musical MIDI is a smaller problem than feared.** `midi_out`
reconsiders every 2 ms and emits only on change, so its worst case is ~500
transfers/s — 9% of the pipe. Streaming with control changes fully active still
lands near 85 snapshots/s. No priority scheme needed; the two can just share.

## What to build

### 1. The write direction (small, and useful on its own)

**~50 lines.** The remote input mailbox already exists and the module already
merges it in `input_fold`; all that is missing is a SysEx carrying 48 bytes into
`bmcv.input.remote`. Two transfers per update against a budget of 5283 a second.

Worth doing even if the read direction never happens: it means a module with
nothing but a USB cable in it can be driven from the page.

Needs `SYSEX_MAX_LEN` (16) revisited — the parser currently skips anything
longer without buffering it, so a 48-byte payload needs a streaming path.

### 2. The read direction

**~80 lines, not the 150–250 this doc first estimated**, because at 93.7 whole
snapshots a second there is nothing to be clever about:

- Copy `bmcv` into a static buffer at a tick boundary.
- 7-bit encode it and stream it out as one SysEx across ~57 transfers, driven
  from the same `midi_idle()` gate the bench used.
- Browser hands the message to `bmcv_sim_import`. No framing code on that side.

RAM: one 2368 B snapshot buffer, and the encode can be done on the fly from it
rather than into a second 2707 B buffer. CPU is roughly 2% of the core at full
rate — but that is arithmetic, and `BMCV_PROFILE` should be the thing that says
so, not this paragraph.

**Streaming must time out**, the same way the remote input mailbox does and for
the same reason: a tab that goes away must not leave the module streaming at
nobody. The host asks periodically; the module stops when it stops hearing.
`REMOTE_TIMEOUT_US` is the precedent, and the symmetry is worth keeping.

**One thing it does better than the probe.** A snapshot copied at a tick
boundary is internally consistent. The probe's read never is — `probe-bridge.md`
documents that a blob can straddle a tick and warns against relying on
cross-field consistency. This transport can offer what that one cannot.

## Open questions

1. **Is it worth it, now that Firefox is not the reason?** The Chromium-only
   restriction was half the case for MIDI and it has been set aside. What is
   left is "no ST-Link, no WinUSB binding, works on a module already in a DAW"
   — real, but a smaller prize than "runs in every browser". The read direction
   costs firmware on the hot path where the probe costs none, and that trade
   should be made deliberately rather than because the number came out well.
2. **Does the probe stay?** Almost certainly: it is the only path that costs the
   firmware nothing and the only one that works on a module whose firmware is
   wedged. Both, with the frontend showing whichever is connected —
   `mode.captureHz` already exists so the scopes follow whatever rate a
   transport manages.
3. **Streaming always, or on request?** Bears on whether a module in a DAW
   quietly spends 90% of its MIDI bandwidth on nobody. Leaning: on request,
   with the timeout above.

## Settled, and no longer worth asking

- ~~Firefox's SysEx permission~~ — out of scope; Chromium only, by decision.
- ~~Chunk size tuning~~ — no chunking. Whole snapshots are fast enough.
- ~~A priority rule for the shared endpoint~~ — `midi_out`'s worst case is 9%
  of the pipe. They can just share.
