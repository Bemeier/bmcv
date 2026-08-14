# Watching and driving a module from the browser

The simulator page can show a physical module instead of the simulation, and
send input back to it. Two transports do the same job by different means:

| | over USB | over a debug probe |
|---|---|---|
| what it needs | the module's own USB cable | an ST-Link on the programming header |
| snapshots/s, measured | **90.5** | ~70 |
| firmware cost | ~80 lines, 2.4 KB RAM | **none** |
| browser | Chromium, Firefox (Web MIDI) | Chromium only (WebUSB) |
| works on a wedged module | no | **yes** |

**USB is the default.** It is faster, needs no hardware, no WinUSB binding and
no programming header, and it works on a module already plugged into a DAW.

**The probe is kept for one reason**, and it is a good one: streaming runs from
the module's main loop, which is the thing you would be debugging if the engine
hung. If the core faults or stalls, the snapshots stop and so does everything
else the module says. An ST-Link keeps reading RAM regardless, because a
Cortex-M's debug unit reaches memory over the AHB-AP without the core's
cooperation. A debug tool that goes dark exactly when the module does has a hole
in it.

## The trick, which is the same for both

**The whole module is one struct.** `BmcvInstance` holds the config, the signal
path, the interaction state and the input layer; the firmware keeps exactly one,
at an address the linker chose. Everything the page draws is inside it.

**The wasm decodes it.** `bmcv_sim_import()` takes those bytes, re-points the
nine pointers inside them at itself, and republishes every reading. So the wasm
module - the same firmware code, compiled for the browser - is not a simulation
of the module here, it is *the decoder for a real one*. There is no struct
parser in JavaScript, no duplicated unit conversion, no second copy of the LED
curve, and there cannot be one, because nothing on the page ever looks inside
the blob.

That property is what every other decision here protects.

**The layouts are held equal by construction.** `sim/include/layout_target.h` is
generated from the firmware ELF's debug info and asserts the offset and size of
every leaf field of `BmcvInstance` - 379 of them. It is compiled into the wasm,
so `just check` fails if the two builds ever disagree. Getting there needed one
fix: `arm-none-eabi-gcc` defaults to `-fshort-enums` and clang does not, so
three enum fields are `int8_t` with the enum named in a comment.

Run `just layout-check` after `just build-rel` and review the diff.

## Over USB

The module already enumerates as USB MIDI and already speaks SysEx.

- The host asks with `SYSEX_CMD_STREAM_REQ`; the module answers with one
  `SYSEX_CMD_SNAPSHOT` carrying the whole instance, seven-bit encoded.
- **One request buys one snapshot.** The page asks again only once it has
  decoded and adopted the last, so the module cannot outrun what the page can
  draw. Two requests stay outstanding so the USB does not idle for a round trip;
  credit beyond that is dropped rather than banked, so what goes out is the
  module as it is now and not a queue of stale frames.
- Credit is also the heartbeat. A page that goes away stops asking, credit runs
  out, and the module falls silent on its own.
- **One SysEx per snapshot**, so the browser's own MIDI stack reassembles the 57
  transfers and the page has no framing code at all.
- The snapshot is copied between engine ticks, so it is internally consistent -
  which a probe read is not.

**What it costs.** A snapshot holds the IN endpoint for about eleven
milliseconds and cannot be interrupted once begun, so the engine's own control
changes take the gap between snapshots. That is real MIDI output latency while
streaming, and the reason streaming is a mode you ask for rather than something
always on.

## Over a debug probe

`web/probe/stlink.js` speaks enough of the ST-Link protocol over WebUSB to
identify the probe, enter SWD mode, and read and write memory. A Cortex-M's
memory is readable without halting the core, so the module keeps playing while
it is watched.

Finding the instance is the one thing WebUSB cannot do for itself: `bmcv` moves
whenever anything before it in `.bss` changes size, and a browser has no ELF and
no `nm`. So the firmware says where it is, at an address that does not move -
`Core/Inc/Lib/bmcv_probe.h` declares a 28-byte `const` and the linker pins it to
`0x08000200`, just past the vector table. Two linker `ASSERT`s hold the address
and the size to what the header declares.

## The write direction

Both transports carry the same two mailboxes, and the module cannot tell which
one a byte arrived over.

**`RemoteInput`** is a second panel, wherever it is: button levels to OR in,
free-running encoder positions to sum, an optional crossfader override.
`input_fold` merges it, so the module answers it exactly as it answers a finger
- UX feedback, LED response, autosave, all of it.

Every field is a *level* or a *free-running position*, never a delta waiting to
be consumed. That is what makes it safe to write over a wire with no handshake:
a torn write costs one tick of a wrong value, nothing accumulates so nothing can
be applied twice or lost, and the two sides never have to agree a baseline. The
writer picks its own encoder origin and never learns the module's.

The crossfader is the only control where the two panels can genuinely disagree,
being an absolute position. Last mover wins: a new value in the mailbox takes
the fader and remembers where the physical one was sitting, and the physical one
takes it back by moving `REMOTE_SLIDER_RELEASE_RAW` away from there. The
threshold is what stops ADC noise reading as a hand on the fader.

`RemoteInput.seq` is a heartbeat as well as an update. A writer that stops -
a page refreshed with a button held - would otherwise leave its levels standing,
and a module stuck holding SHIFT has no way back but a power cycle.

**`RemoteCommand`** is the two things a panel cannot say: start again, and forget
what was saved. Destructive, and neither is a level, so it is an edge - acted on
once per change of its sequence number, however often it is rewritten. That is
what "Reset module" and "Clear FRAM" send when a module is connected.

## Diagnosing it

`web/midi-bench/` has two instruments, and they earned their place. Every fault
on this link that was reasoned about was diagnosed wrongly; every one these were
pointed at gave the answer in a single run.

- **Measure the snapshot rate** asks for snapshots exactly as the simulator page
  does but decodes nothing and draws nothing. Fast here and slow there means the
  cost is in decoding and drawing; slow here too means the transport or the
  firmware. It reports the spread as well as the average, because a steady
  stream and a bursty one average the same and look nothing alike.
- **Diagnose the bus** opens every port, reports its state, listens for three
  seconds without speaking, then sends one identity request to each output and
  prints every byte that comes back. It separates a dead endpoint from a lost
  reply, and a port that will not open from a module that will not answer.

## Two things that cost a long evening

**A MIDI port is named by the host driver, not by the device.** The module's USB
product string is "BMCV", but it publishes no jack strings at all, so there is
nothing for a driver to name a port after - Windows is fond of "USB Audio
Device". `web/probe/midiports.js` finds the module by asking it who it is, which
is a positive identification rather than a guess.

**`send()` and `onmidimessage` open a port implicitly, and asynchronously.** A
request issued straight away can go out before anything is listening, and the
reply lands on a port nobody holds - and `send()` on a still-opening port throws,
which reads as "not this one". Both ends are opened and awaited first.
