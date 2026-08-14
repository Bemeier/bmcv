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

### Then it got much worse, and that was a dropped command

Second run: the meter said 60 Hz and the panel updated about once a second.

Both were true. `sysex_feed` returned *the first* command recognised in a
transfer and ignored the rest - survivable while the only commands were a reboot
and a version request, both rare and both sent alone. It stopped being
survivable the moment the page sent a 61-byte remote input mailbox alongside
small stream requests: a mailbox is 84 packet bytes, so it always spans two
transfers, and whatever was queued behind it shares the second one. A host stack
packs what it has. The stream request in that transfer simply vanished.

So credit ran out, the module fell silent, and the page's stall timer re-armed it
around once a second - which is exactly what was on screen. The meter read 60
because it smoothed the *gap between arrivals*, and within a burst that gap is
near zero. Two independent misreadings agreeing on a wrong story.

There was a second half to the same bug: `sysex_payload()` handed back whatever
was in the parser's buffer *after* the transfer, so a mailbox followed by any
other message in the same transfer decoded from the wrong bytes.

`sysex_feed` now reports every message as it completes, through a handler, with
its payload still intact - which fixes both halves and lets the parser go back to
resetting itself cleanly at F7. `two_messages_in_one_transfer_are_both_reported`
covers exactly the packing that broke it.

The meter now counts arrivals over a window instead of smoothing the gaps. The
gap is only the truth when arrivals are evenly spaced, and a pushed stream's are
not.

### And then it stopped after eight snapshots, which was a protocol violation

Third run, and the measurement that found it: **74 snapshots/s over 0.1
seconds.** About eight arrived, evenly spaced 11.3ms apart, and then nothing at
all for the remaining eight seconds. The transport was never slow. The stream
was stopping.

`midi_publish()` was sending the engine's control changes into the middle of a
snapshot. USB-MIDI allows exactly one thing to interleave a SysEx on a cable:
System Real-Time. A control change between two of a snapshot's fifty-seven
transfers is a protocol violation, and what a host does with it is abandon the
message it was assembling. So the module believed it had sent a snapshot the
page never saw, the page never asked for another, and credit ran out for good.

On the simulator page the stall timer re-armed it about once a second, which is
precisely the choppiness that was reported. On the bench page, which has no
stall timer, it simply stopped - which is what made it obvious.

`midi_stream_active()` had been written for this and then never called. The
guard now exists in both places that can reach the endpoint: control changes in
`bmcv.c`, and the identity reply in `midi_poll_control()`.

**What it costs.** Control changes wait for the gap between snapshots rather
than being lost - `midi_out` only emits on change, and a queue that overflows
marks those entries `CC_NEVER` so the next slot says them again. A snapshot
holds the endpoint for about eleven milliseconds, so that is the output latency
streaming adds. Whether that is acceptable while a module is in a DAW is a real
question, and one of the reasons streaming is a mode you ask for rather than
something always on.

**Why the first bench never saw it:** it suppressed `midi_publish()` for the
duration, deliberately, so the number would be the endpoint's rather than the
endpoint's minus whatever else was talking. The suppression that made the
measurement clean is the same one the real path was missing.

### Then it could not be found at all, which was two more of the same mistake

Fourth run: `no BMCV answered. Outputs tried: BMCV, loopMIDI Port.` The port is
named BMCV - so the naming theory that motivated the handshake was wrong all
along, though the handshake is still the right mechanism and it is what made the
failure legible.

Two causes, both introduced by the fix above.

**Identity was starvable.** Guarding the reply on `!snapshot_sending` was
correct - a second SysEx started inside the first is not something a host can
parse - but a snapshot holds the endpoint for eleven milliseconds and cannot be
interrupted once begun, so a request arriving mid-snapshot waits for a gap that
a topped-up stream never leaves. A module streaming perfectly well became
undiscoverable. `midi_stream_poll` now declines to *start* a snapshot while a
control reply is pending; interrupting one is still forbidden, starting one is
now polite.

**The endpoint could wedge permanently.** A host that stops reading leaves a
transfer outstanding for ever, and a tab closing mid-snapshot does exactly that.
The state stays BUSY, nothing can be sent again - identity included - and the
module is silent until power-cycled. Nothing on its side notices, so it needs a
watchdog rather than an error path: `midi_tx_recover()` flushes the endpoint and
drops the half-sent snapshot after 50ms of a transfer going nowhere.

This is the same failure `probe-bridge.md` records from the other end, where a
refreshed page left the ST-Link mid-reply and the next session hung. Both are a
host disappearing without saying so; both want recovery without being asked.

Discovery also walks the bus twice now, since a deferred reply plus a busy
machine can outlast a single 250ms window.

### Confirmed healthy

The monitor, listening before it spoke: 168 control changes in three seconds,
and `f0 7d 42 4d 02 00 0a 00 f7` - the identity reply, version 0.10.0, returned
immediately.

The run before that one heard *one* message across the whole session, and that
one was loopMIDI echoing our own request. The module had been emitting nothing
at all, control changes included. So the watchdog really was killing the
endpoint, and removing it really did fix it.

Two lessons worth keeping, both about reading rather than writing:

- **The evidence for "the endpoint is dead, not the reply path" was in the log a
  round before it was acted on.** "Total messages heard: 1" said it plainly. A
  module that publishes control changes continuously and sends nothing is not a
  SysEx problem, and three separate fixes went into the reply path after that
  was already knowable.
- **`connection=closed` on all four ports was in the same log.** Discovery was
  sending and listening on ports it had never opened, and both operations open
  implicitly and asynchronously - so the reply came back before anything was
  holding the port. That is what "no BMCV answered" had been from the start.

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
