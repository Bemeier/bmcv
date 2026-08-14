# Spike: the module's own USB port as the transport

Status: **both directions built, untested on hardware.** The measurement that
justified them is below. `just check` covers everything but the USB itself.
The bench command that produced the number is still throwaway.

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

## What was built

### The write direction

`SYSEX_CMD_REMOTE_INPUT` carries the 48-byte mailbox, seven-bit encoded to 55.
The module already merged that mailbox in `input_fold`, so the whole write
direction is a decode and a copy.

One thing it could not do naively: the decode happens in the USB interrupt,
which can preempt `input_fold` partway through reading the mailbox — and a
mailbox that is half this update and half the last one is the one thing its
design does not tolerate. So the ISR stages it and `bmcv.c` moves it across
between ticks, where nothing can interleave.

### The read direction

`SYSEX_CMD_STREAM_REQ` asks; `SYSEX_CMD_SNAPSHOT` answers with a whole
`BmcvInstance`. The request is repeated rather than an on/off pair, and the
module stops two seconds after the last one — the same heartbeat reasoning
`RemoteInput.seq` uses, for the same reason.

The instance is copied between ticks and streamed from that copy, so **a
snapshot is internally consistent** — which the probe's read never is.

The encoding happens as the stream goes rather than into a second buffer, so
only the 2368-byte copy is held. Firmware RAM 7.4% → 9.4% of 128 KB, flash
+1 KB.

### One codec, not two

`sysex7_encode`/`_decode` live in `sysex.c` and the browser reaches them
*through the wasm* (`bmcv_sim_sysex7_*`). There is no copy of the codec in JS,
for the same reason there is no copy of the struct layout — the two ends have
to agree exactly, and the way to make two things agree here has always been to
give them one implementation.

The frontend has no framing code at all: Web MIDI does not deliver a SysEx until
its F7 arrives, so the browser's own stack reassembles the 57 transfers and
`midilink.js` receives one message, decodes it, and hands it to
`bmcv_sim_import`.

### What is not shared with the probe path

`web/probe/midilink.js` is a sibling of `probe.js`, not a refactor of it. They
duplicate the save/restore of the running simulation and the `mode.drivenBy`
bookkeeping — perhaps twenty lines. That was deliberate: an ST-Link is *polled*
and a module *pushes*, so the two connect and stay alive differently, and
unifying them before knowing whether this transport stays would be guessing at
the shape. If it graduates, that is the first thing to do.

## Still to do

**Try it on hardware.** Flash, open the page, press "Connect over MIDI".

Then the things only a board can show:

1. The achieved snapshot rate with the engine running and control changes
   sharing the endpoint. The arithmetic says ~85/s; nothing has measured it.
2. Whether `midi_stream_poll` costs enough main-loop time to move `engine_fps`.
   `BMCV_PROFILE` should answer that, not a paragraph of estimation.
3. Whether a 2707-byte SysEx every ~11 ms upsets anything else on the MIDI bus —
   a DAW sharing the port, for instance.

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
