# MIDI

The module enumerates as a USB MIDI device and talks in both directions. This
is what it says and what it listens for.

## What it sends

Everything on **MIDI channel 1**, in one contiguous block of control changes.

| CC | Source |
|---|---|
| 16–23 | channel outputs 1–8 |
| 24–27 | CV inputs 1–4 |

The value is the signal at the jack, with the converter's full ±10V range
scaled onto 0..127 — so **0V is 64**, a full negative swing is 0 and a full
positive swing is 127.

That mapping is deliberately about the jack rather than about the patch: a
channel clamped to 0..5V uses a quarter of the CC range, because that is the
quarter of the module's output range it actually occupies. Nothing rescales
underneath you when you change the clamp — which also means that **if it is
clipped on the module it is clipped on MIDI**. The clamp is applied in the
output stage before any of this (`channel.c`, `clamp_output`), so a value
sitting at 0 or 127 is the same clipping the jack is doing, and using the clamp
as a shaping tool works the same way in both places.

The channel outputs are read *after* the mute gain, so a muted channel reads as
silent here for the same reason its jack does.

### Rate

Control changes are reconsidered 500 times a second and sent only when the
7-bit value has moved. An idle patch therefore sends nothing at all, and the
worst case is 12 messages per slot.

The rate limit is not a nicety. The engine runs at 4kHz, and a full-scale sine
at 5Hz crosses 128 quantization levels about 256 times a cycle — around 1280
CC/s on one channel and 10k/s across eight, which fits through the endpoint with
no margin and is more than a DAW wants to think about.

7 bits is visibly stepped on a slow ramp into something like a filter cutoff.
That is the cost of a mapping every DAW can learn without configuration; CC
MSB/LSB pairs would fix it at double the traffic and with patchy support.

### Clock

The module sends MIDI Clock at the standard 24 PPQN, plus Start when its clock
resets and Stop when the beat is lost.

**Only when an input jack is the clock source.** That is the same rule the
input direction uses: a patched cable wins, and MIDI is the fallback. So the
module forwards a rack clock out to a DAW, and stays quiet when the DAW is
already the master — regenerating a host's own clock back at it is redundant at
best.

## What it listens for

- **MIDI Clock (0xF8)** and **Start (0xFA)**, used as the clock and reset when
  no input jack is configured for the job. Continue (0xFB) deliberately does not
  read as Start: it resumes rather than restarts, and treating it as a reset
  would drop every oscillator's phase on a DAW punch-in.
And nothing else. Control traffic — the firmware updater's reboot-into-DFU, the
version string it shows beside the image it is about to write, snapshots and the
remote panel — used to come through here as SysEx and now goes over the module's
vendor interface. See [live-module.md](live-module.md) and
`Core/Inc/Lib/usblink.h`. What is left on this endpoint is what MIDI is for.

## Where this lives

`Core/Src/Lib/midi_out.c` decides what to send and holds it in a queue. It is
core code: no USB, no host types, and `tests/test_midi_out.c` drives it with
hand-built engine states. Each host only carries the bytes —
`Core/Src/Lib/midi.c` wraps them as USB-MIDI event packets for the endpoint,
and `web/midi.js` hands them to a Web MIDI port.

The mapping is therefore in exactly one place, and the simulator cannot drift
from the hardware.

## Using the simulator's MIDI out

The **MIDI out** box shows the twelve CCs as they are sent, with a bar per
value: what the far end is receiving, at the 7-bit resolution it receives it,
rather than a re-derivation from the engine. A bar pinned to either end is
marked, so clipping against a channel's clamp is visible as clipping.

That readout is fed by draining the module's queue, which happens whether or
not a port is selected — so it works with no MIDI port on the machine at all,
which is the common case and the quickest way to see what the mapping does.

Two things are worth knowing before it can actually send:

- **Web MIDI is Chromium and Firefox.** Safari does not ship it. The page says
  so rather than failing quietly.
- **A browser can only send to a port that already exists**, and no operating
  system provides one by default. macOS has the IAC driver (Audio MIDI Setup →
  Window → MIDI Studio), Windows needs loopMIDI or similar, Linux has ALSA's
  virmidi. Without one the port list is empty.

## Not implemented

- The VCV Rack module does not send MIDI. Patch its outputs into Rack's own
  CV-MIDI module, which does the same job natively.
- The dialled parameters (FRQ/SHP/MOD/PHS/AMP/OFS) are not published — only the
  values the channels produce. The module is a modulation source here, not a
  control surface.
