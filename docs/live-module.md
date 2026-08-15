# Watching and driving a module from the browser

The simulator page can show a physical module instead of the simulation, and
send input back to it. Two transports do the same job by different means:

| | over USB | over a debug probe |
|---|---|---|
| what it needs | the module's own USB cable | an ST-Link on the programming header |
| snapshots/s, measured | **364** | ~70 |
| firmware cost | ~80 lines, 2.4 KB RAM | **none** |
| survives a power cycle | **yes** | yes |
| works on a module that has stopped answering | no | **yes** |

**USB is the default.** It is five times faster, needs no hardware beyond the
cable already in the module, and works on one already plugged into a DAW.

**The probe is kept for one reason.** Streaming runs from the module's main
loop, which is the thing you would be debugging if the engine hung. If the core
faults or stalls, the snapshots stop and so does everything else the module
says. An ST-Link keeps reading RAM regardless, because a Cortex-M's debug unit
reaches memory over the AHB-AP without the core's cooperation. It has already
earned that place once, proving a module was healthy while its USB had gone
quiet.

## The trick, which is the same for both

**The whole module is one struct.** `BmcvInstance` holds the config, the signal
path, the interaction state and the input layer; the firmware keeps exactly one.
Everything the page draws is inside it.

**The wasm decodes it.** `bmcv_sim_import()` takes those bytes, re-points the
pointers inside them at itself, and republishes every reading. So the wasm
module - the same firmware code, compiled for the browser - is not a simulation
of the module here, it is *the decoder for a real one*. There is no struct
parser in JavaScript, no duplicated unit conversion, no second copy of the LED
curve, and there cannot be one, because nothing on the page ever looks inside
the blob.

That property is what every other decision here protects, and it is why three
transports have now been swapped underneath it without any of them touching it.

**The layouts are held equal by construction.** `sim/include/layout_target.h` is
generated from the firmware ELF's debug info and asserts the offset and size of
every leaf field of `BmcvInstance`. It is compiled into the wasm, so `just
check` fails if the two builds ever disagree. Getting there needed one fix:
`arm-none-eabi-gcc` defaults to `-fshort-enums` and clang does not, so three
enum fields are `int8_t` with the enum named in a comment.

Run `just layout-check` after `just build-rel` and review the diff.

## Over USB

The module is a composite device: MIDI on interface 0, exactly as it always was,
and a vendor-specific interface on 1 with its own pair of bulk endpoints.

**Nothing installs a driver.** `USB_Device/App/usbd_webusb.c` carries a BOS
descriptor with a WebUSB platform capability and a Microsoft OS 2.0 one; the
descriptor set behind the second names WinUSB as the driver for interface 1 and
registers a device interface GUID. Windows binds it automatically. Chrome then
claims the interface through `navigator.usb`.

Nothing on the wire is encoded or framed. A bulk transfer keeps its boundaries,
so one transfer is one message, and a snapshot is simply the instance's bytes -
the browser assembles the packets and hands over the whole thing, so neither
side reassembles anything.

- `USBLINK_OP_SNAPSHOT_REQ` asks; the module answers with the instance.
- **One request buys one snapshot.** The page asks again only once it has
  decoded and adopted the last, so the module cannot outrun what the page can
  draw. Two requests stay outstanding so the endpoint does not idle for a round
  trip; credit beyond that is dropped rather than banked, so what goes out is
  the module as it is now and not a queue of stale frames.
- Credit is also the heartbeat. A page that goes away stops asking, credit runs
  out, and the module falls silent on its own.
- The snapshot is copied between engine ticks, so it is internally consistent -
  which a probe read is not.
- Snapshots have their own endpoint, so the engine's MIDI output competes with
  nothing.

`BMCV_REQ_VERSION` is a vendor *control* request answered with the firmware
version. A question about the device rather than part of the stream, and a short
reply sharing an endpoint with 2384-byte snapshots would have to be told apart
from one by length - which is the framing this transport exists to avoid.

**Every call to the device is bounded.** WebUSB transfers have no timeout of
their own: a read for data that never comes simply never settles, with no error
and nothing to retry. During a connect that is an attempt which cannot fail, so
the retries never run and the switch that started it never finishes - which
locks every button on the page, since a switch already running refuses to start
another. `web/frontend-check.mjs` holds this, because it is a property that is
easy to lose one call at a time.

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

One thing the write direction exposed, older than any of this: a rebuilt
instance zeroes the frame encoder positions are compared against, while the
physical counters keep theirs. The difference is not a turn, but the first fold
read it as one and applied it to whichever parameter was selected - which after
a reset is deliberately OFS. `InputFrames.baseline` is why the first fold after
an instance is built reports a position rather than a movement.

## The update page

`web/update/` flashes over DFU, and now reboots the module into DFU over the
same vendor interface. That matters more there than anywhere: the thing that
page does most is make the module leave the USB bus, and it used to ask over the
transport least able to survive that.

## Diagnosing it

`web/diagnostics/` has two instruments, and they earned their place. Every fault
on these links that was reasoned about was diagnosed wrongly; every one these
were pointed at gave the answer in a single run.

- **Inspect the device** reports what the browser can see: interfaces,
  endpoints, whether the vendor interface can be claimed - which is what says
  the driver bound - and what firmware is running. It separates a driver that
  did not bind from a module that is not answering.
- **Measure the snapshot rate** asks for snapshots exactly as the simulator page
  does but decodes nothing and draws nothing. Fast here and slow there means the
  cost is in decoding and drawing; slow here too means the transport or the
  firmware. It reports the spread as well as the average, because a steady
  stream and a bursty one average the same and look nothing alike.

## What the MIDI transport taught, before it was deleted

Snapshots were carried over MIDI SysEx for a while. It worked - 90 a second,
measured - and was removed anyway, because a browser enumerates MIDI *once*: a
module that left the bus, which a power cycle or a reflash does, could not be
reached again until the browser was restarted. No page can reach past that.

Two things from it are worth keeping in mind, because they are properties of
MIDI rather than of that implementation:

- **Only System Real-Time may interleave a SysEx.** Sharing an endpoint with the
  engine's control changes meant every snapshot held it for eleven milliseconds
  while musical output waited - and a control change landing inside a snapshot
  made the host abandon it, which presented as a stream that stopped for no
  visible reason.
- **A host packs what it has into a transfer.** A parser that reported only the
  first message in one silently dropped the second, which is how the module came
  to be starved of the requests it was waiting for.

The module still speaks MIDI, for what MIDI is for: control changes out, a clock
in.
