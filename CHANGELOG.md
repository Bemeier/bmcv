# Changelog

## v0.15.0 (2026-08-16)

### Feat

- **sr**: measure what a small turn does to the pattern's order, not its values
- **drift**: a third random mode, with no pattern in it
- **sr**: a second stepped mode, driven for modulation
- **sr**: measure each channel's own pattern, a slot at a time
- **sr**: compute the correction from the pattern, behind a build knob

### Fix

- **dac**: serve a due transaction where it lands, not when the loop notices
- **profile**: ask gdb for plain integers, and say what failed
- **profile**: read the struct's layout from the ELF, not from counting by hand
- **dac**: spread the ramp over the tick that is actually being served
- **sr**: give out one pattern measurement per tick, not one per channel
- **sr**: keep SHP and MOD off the level, and leave the pattern centred

### Refactor

- call it stepped, not stepped random
- **sr**: keep the generator's arithmetic off the channel's path
- **sr**: one stepped mode, and it is the one driven for modulation
- **sr**: make the correction a function, not only a table

## v0.14.1 (2026-08-15)

### Fix

- **usblink**: give each credit counter a single writer, as its comment claimed
- **web**: bound every call the updater makes to the device
- **midi**: count the clocks in a transfer, not whether there were any
- **core**: a request from an interrupt is not a plain uint8_t

## v0.14.0 (2026-08-15)

### Feat

- **build**: flash over USB, with the module still in the rack
- **ui**: a parameter button gives you its parameter, on the way out of a page too

### Fix

- **web**: the scope gap belongs between the cells, not around them
- **ui**: a press and a turn in the same tick is a fine adjust, not a reset

## v0.13.0 (2026-08-15)

### Feat

- **stepped**: give SHP and MOD something to steer, not just reshuffle

## v0.12.0 (2026-08-15)

### Feat

- **panel**: the artwork becomes an opaque panel with the cutouts punched out

## v0.11.1 (2026-08-15)

### Fix

- **web**: shift and the wheel, and a bracket that stopped at three corners

## v0.11.0 (2026-08-15)

### BREAKING CHANGE

- SlotStore holds raw EngineConfigs, so the simulator's storage
blob shrinks from 6400 to 5920 bytes. web/storage.js rejects a blob of the wrong
length and the Rack plugin drops the slot, so a saved browser patch and any
stored Rack presets are lost once, and come back as the first-boot defaults.

### Feat

- **web**: a manual, a menu on every page, and one stylesheet behind all four
- **sim**: one scope window for every source, and a ring that forgets
- **web**: the panel documents itself, and the page says what it is
- **web**: a panel that stays readable while you cross it
- **web**: make the panel explain itself
- **web**: set an input's level by pointing at its trace
- **web**: say what an input is for in the colour the module says it in
- **web**: one hover colour, a table that is there on arrival, and a version to ask for
- **web**: a page with a shape before it has numbers
- **web**: pick a source, and a keyboard that scales
- **web**: read the scopes without having to remember anything
- **web**: stack the scopes, and show what the module is configured as
- carry the module over WebUSB, and delete the SysEx layer
- **usb**: a second interface, for a browser to claim
- **usb**: the descriptors that get a browser to the module with no driver
- **core**: let a host reset a module and forget its storage
- **midi**: drive and watch a module over its own USB port
- **input**: let a host drive a module it did not build
- **web**: show a physical module on the simulator page
- **leds**: lift a dim mixed hue to where its dies behave
- **probe**: publish where the module keeps its state
- **sim**: adopt a whole module as bytes

### Fix

- **web**: the updater must load without a simulator build
- **usb**: let a second browser session reach a module the first left mid-frame
- **web**: hover promises the colour the button is about to wear
- **web**: the scale drew outside its box at wide widths
- **web**: a usb connect that can fail, so that it can be retried
- **web**: one live link at a time, and a switch that looks like one
- **web**: a readable keyboard, brackets that mark something, and a first connect that holds
- **input**: a rebuilt instance is a position, not a gesture
- **midi**: say when the browser is holding a port that no longer reaches anything
- **midi**: remove the watchdog that made the module answer nothing
- **midi**: open the ports before using them, and add a raw bus monitor
- **midi**: keep the module findable, and unwedge its endpoint
- **midi**: stop control changes landing inside a snapshot
- **sysex**: report every message in a transfer, not just the first
- **midi**: let the page pace the module instead of the other way round
- **midi**: find the module by asking it, not by reading its label
- **build**: make a version bump reach the firmware

### Refactor

- MON becomes MIX
- **web**: three sources, one session, and resets that reach the module
- retire the throughput spike, and fix the update page's port lookup
- **config**: store every mode as int8_t so the four builds agree

## v0.10.0 (2026-08-13)

### Feat

- **leds**: rebuild the colour pipeline around light rather than duty

## v0.9.0 (2026-08-11)

### Feat

- **ui**: rework the FRQ ring - prime-limit colour, off-grid wash, rate pulse
- **debug**: publish led_fps, the rate the panel is actually redrawn
- **midi**: publish the channel outputs, inputs and clock as MIDI

## v0.8.0 (2026-08-11)

### Feat

- **ui**: save the selected parameter with the patch, defaulting to OFS

## v0.7.2 (2026-08-10)

### Fix

- **update**: refuse an unusable DFU transfer size instead of guessing
- **leds**: recover from a failed LED DMA instead of freezing the panel

### Refactor

- delete two dead linker scripts and the unused FRAM helpers

## v0.7.1 (2026-08-10)

### Fix

- **leds**: guard led_set_bipolar against a zero divisor and INT32_MIN
- **clock**: drop the pulse history when the clock source changes

## v0.7.0 (2026-08-10)

### Feat

- **release**: publish the firmware .elf alongside the .bin

## v0.6.2 (2026-08-10)

### Fix

- **vcv**: correct the plugin install paths and decouple just from the script

## v0.6.1 (2026-08-08)

### Fix

- **release**: stop release notes from picking up stale changelog prose

## v0.6.0 (2026-08-07)

### Refactor

- reorganize repo with firmware at the root

## v0.5.0 (2026-08-07)

### Feat

- **setup**: fetch the ARM toolchain automatically, document all 4 targets

## v0.4.0 (2026-08-07)

### Feat

- **midi**: drive the clock and reset from MIDI when no input is configured

## v0.3.0 (2026-08-07)

### Feat

- **update**: flash a released version from a dropdown

## v0.2.0 (2026-08-07)

### Feat

- **release**: commitizen versioning and GitHub Pages deploy

### Fix

- **release**: include the CI jobs and CMake wiring from the last commit

## The module flashes itself, from a browser tab

The module has a USB-C port and, until now, no way to use it for the one thing a
USB port on a Eurorack module is for. Updating meant an ST-Link on the
programming header.

It turns out the hardware was already designed for this and nobody had noticed:
tracing the schematic, **FN2 is wired to PB8, which is the chip's BOOT0 pin** —
10k pulldown, 100R series, button to 3V3. Holding FN2 while the case powers up
pulls BOOT0 high and hands the module to the DFU bootloader that lives in the
STM32's mask ROM. No bootloader of ours needed, no flash spent on one, and
nothing that can be bricked, because mask ROM cannot be erased.

So `web/update/` is a page that speaks DfuSe over WebUSB, and the firmware
gained just enough to make the button optional.

### Added

- **A SysEx control channel** (`sysex.c`) on the USB MIDI endpoint the firmware
  already enumerated. Two commands: reboot into update mode, and report the
  running version. Parsing is hardware-free and tested — this is the one path by
  which a passing DAW message could reboot the module mid-set, so the tests are
  mostly about everything it must *not* answer.
- **A reset into the ROM bootloader**, rather than a jump into it. The firmware
  lights the panel, leaves the USB bus, writes a marker to a `.noinit` word and
  resets; `fw_update_check_boot()` reads that marker as the first statement in
  `main()`, before `HAL_Init` and before any peripheral exists, and hands over
  from there.

  The direct jump was tried first and does not work, for a reason worth writing
  down: the bootloader ran - `SCB->VTOR` read back as `0x1FFF0000`, which only
  it sets - but it never brought USB up. The giveaway was that USB and SYSCFG
  registers could not be read over SWD at all while it sat there, because their
  clocks were still gated. It will not initialise a chip it inherits mid-flight,
  however carefully that chip is torn down first. Coming in through FN2 works
  because BOOT0 hands it a chip that has just reset, and the marker-and-reset
  path is how the software route gets the same thing.

  The panel stays amber across the reset for free: the WS2811s latch, and they
  run off +5V rather than off the MCU.
- **`web/update/`** — a separate page from the simulator, so a broken simulator
  build cannot take down the thing that flashes the module. Its DfuSe client is
  written out rather than vendored; the protocol we need is two hundred lines,
  against a library we would use a fifth of.
- **`just dfu-check`**, in `just check`. The flashing sequence is the one thing
  here whose failure mode is a half-written module, and it cannot be tried
  casually, so it is asserted against a fake device: command opcodes, data
  blocks numbered from 2, the short final block sent at its real length, and the
  device resetting mid-leave counted as success rather than failure.
- **The page refuses anything that is not a raw image for this target**, before
  it erases rather than partway through writing. An `.elf` is the mistake that
  matters: it sits next to the `.bin`, it is what the ST-Link workflow leaves
  lying around, and it is not an image at all - byte 0 is an ELF header, not the
  vector table, and a debug build is four times the size of the flash. Writing
  one gets a fifth of the way in and then fails on an address past the end of
  the chip, with the module already erased. Checked by its vector table - first
  word a stack pointer in RAM, second a reset handler in flash - which no
  `.elf`, `.hex` or unrelated file passes. `just dfu-check` asserts it against
  the real built artefacts, both of them.

- **`just where`**, which halts the running module over SWD and resolves PC and
  LR to source lines. A frozen panel looks identical whether the main loop is
  stuck in a spin, sitting in a fault handler, or executing code that no longer
  exists, and this is what tells the three apart. It is what diagnosed the jump
  above, and the manifest stall below.

### Known limits

- **The module does not restart itself after an update - power-cycle the case.**
  The zero-length download that closes a DFU session has to carry block 2, not
  0; block 0 means "this is a command", and a command with no payload is
  discarded, so the session never closed at all. With that fixed the bootloader
  does reach its manifest phase - and then stops. It drops its own USB pull-up
  and parks in dfuMANIFEST-WAIT-RESET, waiting for a bus reset that no longer
  has any route to it, in the place ST's reference implementation calls
  `NVIC_SystemReset()`.

  Measured rather than inferred: PC in ROM at `0x1FFF5064`, `VTOR` at
  `0x1FFF0000`, the app's `uwTick` still zero, and nothing on the bus. Nothing
  the host can send reaches a device that has already disconnected, so this is
  where it stays. The image is fully written by then and the page says so.
- **Update mode shows on the panel only when you got there from the page.** The
  LEDs are set to amber before the reset and the WS2811s latch off +5V, so the
  colour survives it. The FN2 route cannot do this and never will — ST's
  bootloader is running by then and knows nothing about the panel. A dark panel
  in that mode is correct.
- **Windows needs Zadig once**, to bind WinUSB to the bootloader so Chrome can
  claim it. This was the deciding tradeoff: a bootloader of our own carrying MS
  OS 2.0 descriptors would remove the step, but only if those descriptors behave
  on a clean Windows install, which is not verifiable until it is in users'
  hands — against a certain cost in flash, a linker offset, and a new way to
  brick a module.
- **Versions are reported, never enforced.** Any build flashes over any other.
  The ROM bootloader has no concept of a version, so a policy here would only be
  one the recovery path ignores.

## The outputs are left alone

### Removed

- **`DAC_OFFSET_CORRECTION`.** 82 DAC units - about 25mV - were subtracted
  inside `quantize_value()` so that the analog output landed on the true note.
  It described the output stage's zero error, so it reached quantized pitch
  outputs and nothing else: every unquantized channel carried the same error
  uncorrected, and the number was a compile-time constant describing one board.

  Removed rather than generalised. A runtime calibration mode was built and
  measured on the bench, and it could not improve on the untouched outputs: what
  is left after a two-point affine fit is the converter's own INL and the spread
  between channels, and neither is reachable by an offset and a gain. The
  outputs are close enough as they come.

  Quantized output is now an exact semitone and nothing else.

## The tick period is a number now, not an outcome

Two rates ran the module and neither was chosen. The engine's deadline restarted
from `now_us` every tick, so each one began late from wherever the last had
landed and never corrected - a 250us period ran at 263 and `engine_fps` read
3800 against a nominal 4000. The DAC service re-armed on any loop pass a DMA
completion allowed, which happened to give 15.8 chunks per tick because that is
where the HAL turnaround and the tick period crossed.

Both were fine while the DAC only repeated values. Interpolating across a tick
made the first one matter: the fraction divides by the nominal period, so the
output reached its target 13us early and sat flat for the rest of every tick.

### Changed

- **The engine deadline advances by a fixed period instead of restarting.**
  `engine_fps` should now read 4000 rather than 3800, and the drift that made
  nominal and actual disagree is gone - which is what the interpolation needs to
  divide by something meaningful.

  Still not a catch-up. A tick that lands a whole period late is dropped and the
  schedule resynchronised, never repaid as a burst of ticks carrying made-up
  timestamps: the engine is dt-driven, so one long dt is the honest account of a
  loop that could not keep up.

- **The DAC service is rate-limited to a stated ratio.** `DAC_SUBSTEPS` is how
  many times the service covers all eight outputs within one tick, and the chunk
  interval follows from it. The 4x that was emerging by coincidence is now the
  number in the source, which also spreads the chunks evenly instead of letting
  them bunch into the gap between ticks - the ADC's sampling cadence evens out
  with them.

  It is the cheap lever for CPU. A chunk costs single-digit microseconds where a
  tick costs over a hundred, so trading tick rate against substeps holds the
  output rate - and therefore the staircase - while moving real load.

### Added

- **`bmcv_profile.resyncs`**, ticks dropped outright. An overrun is the loop
  running late and catching up; a resync is a tick that never happened. Any
  number here that keeps climbing means the period is too short for the work in
  it, which is the failure that used to be silent.

## The output slides between ticks instead of stepping

A scope on a channel output showed stairs 263us wide - one engine tick - and a
5Hz LFO at full depth climbs them in 41mV jumps. That puts the zero-order-hold
images at the tick rate, 3.8kHz, which is both inside what the output filter
passes and inside where the ear is sharpest; amplitude-modulating with an LFO
was audibly carrying it.

The bus was already fast enough to fix this and was wasting it. The DAC service
ships about four frames per engine tick, and three of them re-sent the previous
value byte for byte, because `DAC_BUF` was only refilled inside the tick gate.
The traffic was being paid for and carrying nothing.

### Changed

- **Channel outputs interpolate between the last two ticks instead of holding
  the last one.** Same frames, same rate, same wire - each one now carries the
  level for the instant it is armed rather than a repeat. Steps go from 263us to
  ~66us and from 41mV to 10mV, moving the images from 3.8kHz to 15.2kHz: 12dB
  off the artifact from the step size alone, and more again from any slope the
  output filter has between those two frequencies.

  Interpolated on elapsed time rather than a sub-step counter, because the tick
  period is not exact - it jitters by a loop pass and overruns roughly every two
  seconds - and a fraction of the time that really passed survives both. A tick
  that runs long holds at its target rather than sliding past it.

  It costs one tick of output latency, the levels for tick N being on the pins
  across tick N+1. Extrapolating instead would have avoided that and overshot
  every direction change, which is a worse artifact than the one being removed.
  A constant level interpolates to itself, so nothing about DC accuracy moves.

  Gates and stepped shapes go through it too, on purpose: a step becoming a
  263us ramp is far short of an audible slew, and gentler through a VCA than the
  broadband click a hard edge is. The costs are a fixed ~50us before a gate
  crosses a downstream trigger threshold and a one-tick transient reaching about
  three quarters of its peak.

  Firmware-side, on the way to the DAC. `engine_state` still holds the exact
  per-tick levels, so internal trigger routing, the LED render, the simulator
  and the VCV module all see what they saw before - which is why no golden flow
  moved.

## The module can be flashed optimized

`just build` produces a Debug build: `CMakeLists.txt` defaults `CMAKE_BUILD_TYPE`
to Debug, and `cmake/gcc-arm-none-eabi.cmake` appends `$<$<CONFIG:DEBUG>:-O0>`
after its global `-O2`, so the last flag wins and every translation unit is
unoptimized. That is what `just flash` has been putting on the module.

The cost is in the engine's hot path. At `-O0`, `fclamp`, `iclamp`,
`phase_mod`, `quantize_value`, `pwm_shape`, `clamp_output`,
`div_round_nearest` and `phase_error` are all real calls with their locals
spilled to the stack, and `channel_compute` runs them eight channels deep at
every tick; at `-O2` none of the eight survives as a symbol. The measurement
that says it out loud is `engine_fps` and `dac_fps` reading the same number:
`bmcv_main` re-arms the DAC once per loop iteration, so the two only agree when
the 250us tick gate is already satisfied on arrival - the loop is running one
pass per tick and the tick is late.

### Added

- **`just configure-rel` / `build-rel` / `flash-rel` / `build-flash-rel`**, the
  same four steps as the Debug set against a RelWithDebInfo build in
  `build-rel/`. RelWithDebInfo rather than Release so it is `-O2` rather than
  `-Os` and keeps its symbols; its own directory so `build/`, `just flash` and
  the `compile_commands.json` symlink clangd reads all still mean the Debug
  build. `scripts/flash.sh` already took the ELF path as `$1`, so `flash-rel`
  is that argument and nothing else changed.

- **`bmcv_profile`, what one tick actually costs.** Optimized, `dac_fps` reads
  ~60000 against `engine_fps` ~3800 - decoupled, where they used to be the same
  number, so the loop now spins between ticks instead of arriving late for each
  one. That says the tick fits, but not by how much: the two rates only bound
  the tick's cost to somewhere between 16 and 85us, because the spare time in a
  period is shared with the DAC service and neither readout separates them.

  So measure it directly. `bmcv.c` reads the Cortex-M4 DWT cycle counter around
  `engine_tick` and around the whole gated block, and keeps last/min/max/average
  of each in `bmcv_profile`, in cycles and in microseconds, plus `load` - the
  average against `ENGINE_TICK_US`. Cycle resolution is 6.9ns where the TIM2
  timestamps everything else uses are 1us, which a tens-of-microseconds span
  needs. Watch it live, the same way `bmcv.engine_state.dac_fps` is watched;
  min and max are sticky and re-arm when written to 0.

  A sticky maximum reached once during startup reads exactly the same as one
  reached every 8ms, so `max_at_tick` records which tick set it and `overruns`
  counts the ticks that ran past `ENGINE_TICK_US`. Against `ticks`, the second
  is a rate rather than an anecdote - which is the difference between a periodic
  cost worth restructuring around and a transient worth ignoring.

  The counter was already enabled - `init_cycle_counter()` has been called from
  `main()` all along, with nothing reading it. The probe costs four register
  loads and a few float operations per tick, under 0.2% of the budget, so it
  stays in the build; `BMCV_PROFILE 0` compiles it out.

## The wavetable is generated, and cannot go quiet

The SHAPE_LFO table was drawn by hand in a visual editor, which is the right way
to find shapes and the wrong way to hold three properties that have to be true
everywhere at once. `tools/gen_wavetable.py` builds it now; `just wavetable`,
with `--report` to see it as sparklines.

### Fixed

- **Two neighbouring slices were the same wave in opposite phase, so the
  channel went silent between them.** Slices 13 and 14: each was full scale on
  its own, and their midpoint cancelled to 0.0003 of a possible 2.0. The
  runtime interpolates across the shape axis, so a knob sitting between two
  entries is the normal case rather than an edge one - nothing had ever checked
  what happened there.
- **A phase below zero made the lookup extrapolate rather than interpolate.**
  The sample index came from a cast, which rounds toward zero, so the fraction
  came out negative and the blend ran past the sample instead of toward it -
  leaving the output range. Floor, not truncate.
- **`shape_table` was defined in the header.** Fine while exactly one file
  included it, a multiple-definition link error the moment a test did. The data
  is `wavetables.c` now and the header declares it.

### Changed

- **Phase 0 is the rising edge, in every shape** - zero and rising at 0, peak at
  a quarter, trough at three quarters. The table used to put the *trough* on the
  beat and its rising edge a quarter of a cycle late, which made it the odd one
  out: `SHAPE_PWM` opens its gate at phase 0 and `SHAPE_STEPPED` begins step 0
  at phase 0, so in both of those the waveform's event is already on the beat.
  PHS 0 therefore meant something different depending on which shape mode a
  channel was in.

  The case that makes it concrete is a square channel used as a **clock
  divider**: a divider whose edge is not on the beat is not one, and at
  divide-by-two the old alignment opened the gate half a beat late every time.
  `a_square_channel_divides_the_clock` drives the real engine against a real
  clock and checks every gate opens on a beat, at both ÷2 and ÷4 - ÷4 alone
  would not have caught it, because there a quarter cycle *is* a whole beat and
  the old edge landed on a beat too, just the wrong one.

  Free, as it turns out: the guarantees below are about all slices sharing the
  *same* anchor phases and not about which phases those are, so the table is
  rotated a quarter cycle on its way out of the generator - an exact shift of
  whole samples. It also puts the canonical shapes in their canonical form, the
  sine being `sin(2*pi*p)` rather than `-cos(2*pi*p)`.
- **Every wave is built from an odd monotone rise curve**, anchored at -1 at
  phase 0 and +1 at phase 0.5 and half-wave antisymmetric. Three things follow
  for nothing, and they are the point of the rewrite:
  - every slice reaches the full swing, at the same phases;
  - **so does every blend of two slices**, because a convex combination of
    monotone rise curves is another one - full amplitude between entries is now
    arithmetic rather than something to be tuned for, and it holds across the
    wrap like everywhere else;
  - the DC offset is exactly zero at every setting. Sweeping SHP used to walk
    the channel's average level by as much as 0.7 of full scale.
- **The canonical shapes are exact.** SINE is at shape 0 - what a channel
  resets to - and is `-cos(2*pi*p)` to better than one LSB; TRIANGLE is exact
  too. The axis is built from two families that share those two shapes exactly,
  so the joins between them are identities rather than approximations.
- **The shape axis is a closed loop**, which it has to be: SHP wraps at the
  parameter (`value += delta * 256` on an int16_t) as well as in the lookup.
  square at shape -1, SINE at 0, TRIANGLE at +0.375, a pointy extreme at
  +0.625, and home to the same square at +1. Slices are spaced evenly in RMS
  rather than evenly in the shape exponent - an exponent axis crowds all its
  visible change into one end, which the first draft of the generator did,
  giving fifteen consecutive slices that were the same square.
- **117 slices become 64, and the table costs 28KB less flash** (47.5% -> 42.0%
  of the G474). The drawn table spent roughly thirty slices on staircases and
  another twenty on pulse widths, which are SHAPE_STEPPED's and SHAPE_PWM's
  jobs now, and had runs of up to five identical slices.
- Saw is deliberately absent: MOD's skew already turns the triangle into one,
  and a saw cannot satisfy the anchoring above - its extreme is at the cycle
  boundary rather than at the half cycle.
- `N` and `M` are `WT_LEN` and `WT_SLICES`. Two single-letter macros in a header
  every DSP file includes is a collision waiting to happen.

### Added

- **`docs/wavetable.md`** and the plots in `docs/images/wavetable-*.svg` - the
  four named shapes at a readable size, and the sweep at every second slice with
  the exact ones picked out. Both written by the same run of the generator that
  writes the header, so a picture in the documentation cannot show a shape the
  firmware does not have. SVG so a reviewer sees a shape change as a diff rather
  than as a new binary, and traces are decimated to about the panel width in
  pixels, which is all a plot can show.

  The strip's sample set is the even grid **unioned with the keyframe indices**,
  so the named shapes are in it by construction. They all sit on multiples of
  four today and any grid would catch them; moving one by a single slice would
  otherwise drop it silently.
- **`tests/test_wavetable.c`** - 1921 checks, asserted against the lookup rather
  than the table, because the lookup is what a channel hears. Full swing and
  zero DC at every setting including between slices, both canonical shapes,
  every wave starting at its minimum and peaking at the half, the loop closing
  with the seam no larger a step than any other, each cycle joining itself, and
  the lookup staying in range for any input at all.

## The sync loop stops lurching through a crossfade

The measurements below found one real defect, and it was not a tuning problem.
Baseline and after-table in `docs/pll.md`.

### Fixed

- **A crossfade between scenes at different rates made the oscillator lurch.**
  The target phase is measured from an origin that repeats every `gcd` beats,
  and `gcd` came from `find_denominator` every tick. That is a search for a
  rational approximation, so as the ratio moves its answer does not move with
  it - it jumps between 8, 7, 6, none, 5 and back. **44 times during a
  one-second fader move**, 108 over a three-second sweep to x4. Every jump moved
  the origin, the target teleported by whole beats, and the loop faithfully
  corrected for a step nothing had caused: 3.1 beats of phase error and the
  channel pulled to **1.4x its own rate**, which is heard.

  `gcd` is latched now (`EngineState.channels_gcd[]`) and re-taken only when the
  super-period wraps - the rule the stepped pattern length already follows, for
  the same reason. The origin is latched with it
  (`channels_beat_origin[]`), and that is what makes the change seamless: at the
  wrap the channel is at phase 0 of the period it is leaving, so defining the
  new period to start there moves the target not at all. Re-basing anywhere else
  cannot work - two origins differ by a whole number of *beats*, which is not a
  whole number of *cycles*, so it would step the output waveform.

  Measured over the whole rework: frequency pull 1.398x -> 0.116x, slew 461.9
  -> 32.9, peak error 3.08 -> 0.24 beats, gcd flips 44 -> 1. Long-run alignment
  on a x0.75 channel improved with it, 0.00019 -> 0.00001 beats RMS, because the
  loop no longer spends corrections on a target that was jumping.

- **The phase accumulator wrapped somewhere other than a whole cycle, and it
  clicked.** Found on hardware, moving the slider.

  The waveform is a function of phase modulo one, so a wrap at exactly one cycle
  is invisible - the sample either side of it is the same sample. The
  accumulator wrapped at the super-period instead, `gcd * ratio` cycles, which
  is a whole number of cycles only when the ratio really is the rational
  multiple `find_denominator` claims. That held while gcd was recomputed every
  tick: it only ever returns a gcd for which `gcd * ratio` is within 0.025 of an
  integer, so the step was there but under 2.5% of a cycle and nobody heard it.
  Latching gcd while the ratio kept moving removed the guarantee, and a
  crossfade stepped the output by up to **0.34 of a cycle**.

  The accumulator is one cycle wide now and nothing else. The super-period
  position the loop needs is `channels_cycle[]` - which cycle of the period the
  oscillator is on - plus that phase, so the thing that wraps oddly is a counter
  nobody hears rather than the phase everybody does.
- **...and the obvious way to fix that would have lost the beat alignment.**
  Reducing the loop's error to a single cycle bounds it at half a cycle and
  costs nothing audible in the steady state, since the waveform repeats. It also
  makes the loop lock to whichever of the equivalent phases is nearest, so a
  x2/3 channel still repeats every three beats but can sit a third of a cycle
  off the bar - and which cycle of the pattern lands on the downbeat is the
  whole point of aligning to a beat multiple. The error is measured over the
  full super-period; `the_pattern_lands_on_the_same_beat_every_time` asserts
  what nothing asserted before.

### Changed

- **A new alignment period is only taken once the ratio has held still**
  (`PLL_RATIO_STABLE_US`, 120ms). A ratio crossing a crossfade is a rational
  multiple every few ticks and irrational between them, so acquiring on the
  instant meant taking and discarding eleven periods in a one-second fader move,
  with the correction switching on and off along with them.
- **The correction is clamped** to `PLL_MAX_PULL` (0.15) of the channel's
  nominal rate. The correction is a speed change, so an unbounded one is an
  unbounded lurch - and since the error wraps at half the super-period, a
  channel on a long alignment period could decide it was three and a half beats
  out and act on it at full gain. Past the clamp a large error closes at a
  constant rate rather than an exponential one: slower, and silent. Costs 4% of
  settling time on the largest error there is (3.90s -> 4.06s).
- **The loop's tuning has names and units.** `PLL_TAU_S`, `PLL_MAX_PULL` and
  `PLL_SMOOTH_S` in `channel.h`, in seconds and in fractions of the rate.
  `k_sync` is gone: it was an EMA coefficient per *tick*, so its time constant
  was a property of the host's control rate, and at 3.3ms it was three hundred
  times faster than the loop and had no say in the response at all. The gain it
  was hiding was 1/second, which is `PLL_TAU_S` 1.0 - so the speed of the loop
  is unchanged, it is just written down.
- `channel_reset_phase` drops the latched period and origin with the phase. A
  reset re-establishes where the beat grid starts, and an origin measured from
  before it would leave the channel repeating on the old one.

### Added

- **`max_phase_jump`** in the PLL metrics: the step in output phase that the
  oscillator's own rate does not account for. This is the invariant - the
  accumulator may only ever wrap at a whole cycle - and it is asserted across
  every moving-ratio case rather than only measured. It is what turned "there is
  sometimes a click" into a number, and it was 0.34 when the click was there.
- **`sim/flows/clock_lock.txt`** - the locked waveform over 16 seconds. Nothing
  in the golden set covered it: the one flow with a clock emits only UI state,
  and the two that emit outputs run without one. Three channels against a 120bpm
  clock at x0.5, x2/3 and x1/3 - periods that come out exactly 1.0000s, 0.7500s
  and 1.5000s - with x2/3 as a 2-against-3 polyrhythm that only returns to phase
  0 on the third beat, so any drift walks the trace through the rows instead of
  repeating on them.

## The sync loop, measured

Step one of two: the phase-lock loop gets numbers before it gets changed. See
`docs/pll.md` for the baseline table and what follows from it.

### Added

- **`tests/pll_metrics.{c,h}`** — a clock generator that drives the fixture at a
  tempo (with deterministic jitter, so a failing run reproduces), a trace, and
  the measurements: settling time, peak error, ringing, peak frequency pull,
  frequency slew, steady-state RMS, and how often the alignment period changed.
  Errors are reported in beats rather than in cycles of the channel's own
  waveform, so a x4 channel and a x0.25 one are comparable.
- **`tests/test_pll.c`** — twelve scenarios: cold lock, scene transitions
  snapped and swept, phase step, tempo step, long-run polyrhythmic alignment,
  a ratio with no beat period at all, clock jitter, clock loss and return, eight
  channels at once, and the same phase step at two control rates. Bounds are
  loose on purpose; the printed table is the artifact, and the point is to diff
  it across a change.
- `ChannelEffective.phase_error` — what the loop is actually minimising. The
  tests measure the loop's own quantity rather than re-deriving it outside,
  which would drift from it the moment the algorithm changes.

### Found

- **The loop is a proportional controller with a time constant of one second**,
  and `k_sync` is not that time constant. It is an EMA coefficient applied per
  tick, so its own time constant is ~3.3ms - three hundred times faster than the
  loop - and `correction` therefore just tracks the error. A half-cycle phase
  step settles in 3.90s at a 250us tick and 3.87s at 1000us, so the response is
  effectively rate-independent, which is not what the code reads like.
- **Steady-state behaviour is very good and should be preserved.** A x0.75
  channel - gcd 4, so it only meets the beat every fourth one - holds 0.0002
  beats RMS over four minutes with no drift. Tempo steps, clock loss and 5%
  jitter are all absorbed. Nothing rings anywhere: 0 or 1 significant crossings
  in every case.
- **A moving ratio is the one real defect.** `find_denominator` changes its
  answer **44 times during a one-second crossfade** between scenes at x1 and x2,
  and 108 times over a three-second sweep to x4. Each change alters
  `beat_counter % gcd` by an arbitrary number of beats, so the target teleports
  and the loop chases a step nothing caused: phase excursions of 3.1 beats, and
  the correction pulling the oscillator to 1.4x its nominal rate. That is an
  audible speed lurch, and it is the artifact the rest of the tuning exists to
  avoid. The error wraps at half the super-period, so a large gcd admits a large
  error - at gcd 7 the loop can believe it is 3.5 beats out and act on it.

## The clock cannot poison the module, and CI runs

### Fixed

- **A clock pulse inside the post-reset guard divided by zero, and the module
  never came back.** `Clock_Trigger` only measures an interval outside the 2ms
  window after a reset, but the divide was not gated on having measured one:
  two pulses inside that window gave `1e6 / 0`. `freq_est` is a leaky
  integrator, so the inf never washed out - every later estimate was inf, every
  channel's phase became NaN, and `wavetable_lookup` was then indexed with
  whatever a NaN casts to. Under ASan that is a read at index -2147483648; on
  the module it is a read from an arbitrary flash address, and on the wasm and
  Rack hosts it is on the audio path. Nothing short of a power cycle recovered.

  Four changes, each independently worth having:
  - The interval is only taken against a pulse that actually happened, and
    `Clock_Reset` drops the pulse history rather than leaving the previous
    run's timing to be measured across the reset. `last_pulse_delta_us` itself
    survives, because it is also `Clock_Poll`'s timeout reference and a reset
    must not make a stopped clock look live.
  - `CLOCK_MIN_PULSE_US` - a pulse closer than 1ms to the last one is not a
    pulse. At four pulses per beat that still admits 15000 BPM, so it only ever
    rejects bounce. Applied before the beat counter, so a bouncing edge does
    not advance the beat either.
  - `CLOCK_BPM_MIN`/`CLOCK_BPM_MAX` - an interval that works out to a tempo
    outside 1..1000 BPM is not a measurement. Rejected rather than clamped: a
    clamp still drags the estimate to the bound and takes several good pulses
    to come back, which is audible as every LFO changing speed at once. Two
    edges 50us apart used to read as a 5kHz beat.
  - `wavetable_lookup` masks `i0`. `i1` was already masked and `i0` was left to
    the caller's contract; one AND makes a bad phase a wrong sample rather than
    a wild read.
- **A non-finite phase is recoverable.** Every comparison against a NaN is
  false, so it survived the wrap, the `fmodf` and the next tick's accumulate -
  once a channel's phase was NaN it stayed NaN. `channel_compute` resets the
  accumulator instead. The clock can no longer produce one, but a host chooses
  its own dt and its own tempo, and one bad frame should cost one cycle rather
  than the channel.
- **An encoder dithering in its detent walked the parameter.** The decoder
  counted a step whenever the quadrature state landed on 00, which scores the
  return leg of a dither and ignores the outward one - so a noisy contact at
  rest accumulated +1 per bounce with nobody touching the panel. Steps are
  accumulated and a detent is emitted per whole quadrature cycle, so the two
  halves cancel exactly and a real turn yields the same one count per detent it
  always did. The first sample after power-on now establishes position rather
  than decoding as movement.

### Added

- **`.github/workflows/ci.yml`.** `just check` is 21 unit tests, four golden
  flows, a headless wasm load and a headless import of the whole frontend, and
  it runs in a couple of seconds with no hardware and no browser - and nothing
  ran it automatically. Four jobs, one per build in `docs/architecture.md`, so
  a failure names the toolchain it failed under:
  - tests + flows, then the same tests again under **ASan/UBSan** (`just
    test-san`). The suite has been clean under it from the day it was added,
    and it is the cheapest thing that would have caught the wavetable read
    above.
  - wasm and the frontend check, under emsdk.
  - **the ARM firmware**, which `just check` does not build - a break confined
    to `BMCV_DRIVER_SOURCES` was otherwise invisible until you flashed. The HAL
    is not vendored, so CI fetches STM32CubeG4 v1.6.1 from ST's public mirror
    with only the two submodules the firmware compiles against, and caches it.
    The size goes into the job summary rather than being gated: the sr-table
    alone moved flash 16 points in one commit, and a number in the log is what
    makes that visible while there is still room to react.
  - `clang-format`, pinned to 18. `.clang-format` existed and nothing enforced
    it.
- **Preset migration** (`config_migrate.{c,h}`). The record format has changed
  three times in as many months and every bump threw away all eight slots -
  correct while the only module is on the bench, wrong the moment somebody else
  has seven scenes dialled in. A version this build knows is now converted:
  - **v3 -> v4** renumbers `shape_mode`. Three stepped modes became one, which
    moved PWM under them; `config_validate` would have clamped SEMI and HARD
    onto PWM, which is the right numeric range and the wrong shape. A channel
    that was SEMI or HARD comes back as `SHAPE_STEPPED` - the shape survives,
    its hold value does not, because hold is not a per-channel setting yet.
  - **v2 -> v3** appends `sr_length_idx` and `clamp_mode` at their defaults and
    **halves every AMP**, because AMP is the peak swing now and was half of it
    then. An unconverted v2 patch would come back twice as loud as it was
    dialled in.
  - The old layouts are spelled out with the types their headers used, not by
    byte offset. A record is only ever read back by the target that wrote it,
    and the layouts genuinely differ between targets - see the note below.
  - `preset_load` checks the CRC over `hdr.length` rather than the current
    struct's size, since an older payload is shorter, and bounds that length
    against what was actually read out of FRAM.
  - A `_Static_assert` on `CONFIG_STATE_VERSION` sits in the way, so the next
    bump is a compile error until somebody says what happens to the records
    already written. `CONFIG_STATE_VERSION` moved to `config.h` for it -
    migration is core code and must not include a driver header, the same
    reason `FRAM_CONFIG_SLOTS` is already there.
- **`mcp_decode.{c,h}`** and `tests/test_mcp_decode.c`. What the two panel
  expanders' port bytes *mean* - the quadrature state machine, the bit-plane
  unpacking, the pin map - had no SPI in it but sat inside `mcp.c` behind the
  HAL, so the one piece of logic every gesture test in the suite depends on was
  the one piece nothing could exercise. It is plain C on the core source list
  now, driven from recorded port bytes: detent counting, dither rejection,
  per-encoder independence, and that the pin map is a permutation rather than
  something with a duplicate in it. `mcp.c` keeps the transfers and the DMA
  state machine.
- `just test-san`, `just build-ci`, `just fmt`, `just fmt-check`.

### Changed

- **The engine runs on a fixed period** (`ENGINE_TICK_US`, 250us). `main()`
  calls `bmcv_main` in a bare `while (1)`, so the engine used to run once per
  iteration and the interval between DAC updates was however long the last pass
  happened to take - an LED flush and a USB frame land in some passes and not
  others. The oscillators are dt-driven and stay correct through that, but the
  samples leaving the module are not evenly spaced and an LFO's edges carry the
  jitter. A floor rather than a catch-up: if the loop cannot keep up, a longer
  dt is the honest thing to hand the engine and `engine_fps` says so. It also
  makes 4kHz true rather than assumed - it is what `web/const.js` and the Rack
  plugin already tick at.
- The USB MIDI output is `midi_publish_inputs()` behind `BMCV_MIDI_CC_INPUTS`,
  and says what it is: the four CV inputs as CC 0x10..0x13 on channel 1. It is
  the only use of the USB MIDI stack, nothing asks for it and nothing
  documented it. Kept and switchable rather than deleted.

### Notes

- **`EngineConfig` is not the same size on the module as in a test.**
  `arm-none-eabi-gcc` defaults to `-fshort-enums` and a host compiler does not,
  so the enum-typed members of `ChannelConfig` are one byte on ARM and four on
  a host: 91 bytes against 97, and 738 against 798 for the whole config. Each
  target is self-consistent - FRAM is only ever read by the module, and a
  host's slot store only by that host - so nothing is broken today. But the
  struct is not a wire format, and anything that would make it one (importing a
  FRAM dump into the simulator, sharing a patch file between hosts) has to fix
  this first. The cure is to give the persisted fields explicit widths, the way
  `shape_mode` and `clamp_mode` already have them, which is a format change and
  wants its own version bump.
- **`find_denominator` is not worth caching.** It runs per channel per tick and
  looks like an obvious cache; measured, it is ~50 cycles, so about 4us of a
  250us budget across all eight channels, against roughly 750 instructions for
  the rest of `channel_compute`. A cache plus its invalidation is more risk than
  1.6% of the control loop is worth while flash is at 47% and RAM at 7%.

## A review pass over the four targets

### Fixed

- **The shipped Rack plugin was linking `tests/fakes/fake_drivers.c`.** Nothing
  in the core has called a driver since the DAC write moved into `bmcv.c`, so
  the stub was dead in all three host builds. Removed from the plugin, the
  simulator and the test library; the file is gone.

### Changed

- **Both hosts open-coded the jack -> converter mapping.** `sim_input_cv()`,
  `sim_input_take_trigs()` and `sim_input_slider()` join the rest of the
  host-side glue in `sim_rt.h`, so filling an `InputSample` is one
  implementation with tests rather than a rule each host has to remember.
  Getting it backwards routes the clock into a modulation input and is
  invisible until something refuses to sync.
- **The Rack widget walked the `HwSetup` index tables to find out what sits
  behind each button.** `panel_layout.h` now carries `panel_button_kind[]`,
  `panel_button_led[]` and `panel_encoder[]`, generated beside the geometry and
  the legends that were already there. Four lookup helpers in `BMCV.cpp` are
  gone with it.
- **`web/leds.js` and `sim/include/led_color.h` are checked against each
  other.** They have to agree or the browser and the Rack module show different
  modules, and nothing linked them but a comment. `web/smoke.mjs` now reads the
  constants out of the C header and compares.

### Local environment

Machine-specific settings no longer live in tracked files:

- **`local.just`**, imported if present and not checked in - the arrangement
  `CMakeLists.txt` already had with `local.cmake`. The Rack SDK path, the
  Windows Rack directory, the SDK version and the browser to drive are all
  overridable there.
- **`vcv/compile_commands.json`**, generated by `just vcv-compdb`. The Rack
  SDK's include paths are per-machine, and `vcv/.clangd` used to invite you to
  paste yours into a tracked file.

### Documentation

- The README opens with what the module *is* and how it is used - the panel,
  the parameter row, scenes and the crossfader, what a shift mode is - instead
  of a TODO list, which has moved to the end.
- **`docs/architecture.md`**: the seam between the core and its four hosts, why
  `sim/` exists, what each build is for, and what is generated rather than
  written.
- **`docs/images/`**, from `just docs-shots`: screenshots of the real frontend
  running the real firmware, driven by a headless browser rather than posed.

## A VCV Rack module, running the firmware

### Added

- **`vcv/` — BMCV as a Rack 2 plugin.** The core is compiled straight out of
  `Core/Src/Lib` from the same list the ARM build, `tests/` and `sim/` use, so
  there is no copy to drift. The module allocates one `BmcvInstance`, samples
  Rack's ports into an `InputSample` every audio frame, ticks the engine on a
  divider at 4kHz and reads `channels_gated_level[]` and `leds[]` back out.
  There is no DSP in the plugin and there must never be any. `just vcv-sdk`,
  `just vcv`, `just vcv-install`.
- **A Windows build, cross-compiled with MinGW-w64** (`just vcv-win-sdk`,
  `just vcv-win-install`), because Rack running on Windows against a checkout
  in WSL cannot load a `.so`. It links libgcc statically as well as libstdc++ -
  Rack's own `plugin.mk` does only the latter, which leaves the DLL importing
  a MinGW runtime file that exists on the build machine and nowhere else.
- **The panel is the real artwork, and the parts are Rack's own.** `just panel`
  emits `vcv/res/BMCV.png` and `vcv/res/BMCV.svg`. Rack derives module width
  from the panel SVG and only lands on the rail grid at a whole number of HP,
  so that one is 81.28mm rather than the 81.0mm the artwork was exported for -
  the 0.28mm gets its own width and offset instead of moving everything else,
  and the image is placed by its board origin so every cutout still lands on
  the control that sits in it. Rack's SVG renderer cannot embed a raster, so
  the SVG is a plain rectangle of the artwork's background colour and the
  plugin draws the image over it - which also means a missing artwork file
  degrades to a blank panel rather than a black one.

  Jacks are `PJ301M`, the lit switches are `VCVLightBezel` with an RGB light
  filling the cap, the three centre tactiles are `TL1105` and the corners have
  Rack's screws: every one of those is both the native part and a fair match
  for what is on the board, and drawing our own versions of them would only
  look like a module that could not be bothered. What is left is what Rack has
  no part for - an endless encoder and a horizontal fader - and those follow
  web/panel.js down to the palette, so the two frontends are one module rather
  than two designs of it.

  `panel_layout.h` gained `PANEL_VCV_*`, `PANEL_ART_*`, the mounting slots and
  the legend tables the widgets and tooltips read.
- **`sim/src/slot_store.c`** — the eight preset slots in memory, factored out
  of `bmcv_sim.c` and now shared with the Rack plugin. Saving a patch writes the
  live config into the autosave slot first, so a reopened patch comes back where
  it was left rather than up to two seconds behind.
- **`sim_tickdiv_reconfig()`** — change the host's sample rate without moving
  the engine's clock. Elapsed time is an unsigned difference, so a timestamp
  that steps back one second is a 71 minute step forward, and every oscillator
  phase and the clock's tempo estimate go with it.

### Fixed

- **HUE_YELLOW was chartreuse.** `led_set_hsv()` divides the wheel into six
  regions of 43, so a hue is pure at a multiple of 43 and a blend in between.
  65 sat two thirds of the way from yellow to green and came out
  `(0.48, 1.00, 0.00)` - and it was only 15 from `HUE_GREEN`, so SHP and AMP
  were nearly the same colour. 43 is yellow. This is the hardware's colour too,
  not only the simulators'.
- **A lit LED washed out to white in Rack.** Rack composites its light layer
  *additively*, which is right over the dark panels its own modules wear and
  wrong over this one: adding a 30%-bright red to a light grey panel gives
  white-pink. The colour now goes down in the base layer, alpha-blended at the
  level `led_color_of()` computed and at web/leds.js's own opacities, so the
  two frontends agree; the light layer carries only the bloom around the part.
  An unlinked module - the module browser's preview - draws dark rather than at
  Rack's default full brightness, because twenty-one lights at once is not what
  a BMCV at rest looks like.
- **A fresh Rack module came up on heap noise.** `SlotStore` is a plain member
  of a `Module`, and a `Module` is allocated with `new`, so `occupied[]` held
  whatever was in that memory: every slot reported itself full,
  `bmcv_instance_init()` "loaded" the autosave slot, and the module booted with
  a -23131 offset, both scenes set to the same one and encoders that moved
  nothing. `slot_store_init()` clears the store rather than trusting its caller
  to have zeroed it - the simulator calloc'd its way past this and never saw it.
- **`config_defaults()` now zeroes the config it fills.** It only ever set the
  handful of fields that are not zero, which was invisible because every caller
  in the firmware passed a zeroed struct - `bmcv_instance_init()` memsets the
  instance. The first host to build one on the stack got a module with a
  -30639 offset and a 0.004Hz oscillator.

### Changed

- **A shift mode takes 350ms to latch, not 150ms.** `UI_T_HOLD`'s only consumer
  is shift-mode entry, and 150ms was short enough to latch a mode when a
  parameter tap was meant. It also widens the tap window by the same amount,
  which *shrinks* the gap between a tap and the `UI_T_LONG` actions rather than
  opening a new one. The tests and flows that hold a button now say
  `UI_T_HOLD + …` instead of a number that happened to clear it.
- The three centre tactiles are drawn at the size of the part. SW14-16 are 7mm
  against 6mm for the lit switches, and a `TL1105` at its native 5.2mm made the
  bigger switch on the board the smaller one on the panel.
- The scroll wheel turns an encoder over its centre cap as well as its ring.
  The cap is a good half of the knob and a wheel that did nothing there read as
  a dead spot - web/panel.js binds both for the same reason.
- `hw_setup.h` uses `static_assert` and `helpers.h` spells out one `void*`
  cast, so the firmware's headers can be read from C++. Both are C23 already;
  neither changes what the firmware compiles to.

## Clearing looks like clearing

### Changed

- **A held press dips out once instead of blinking.** It crosses a threshold,
  drops for 90ms and comes back - "that registered" - rather than pulsing for as
  long as it is held, which reads as something still in progress. At the first
  stage it returns to exactly what the page was showing, so the dip is the whole
  message; the second, wider stage returns brighter and dips again on the way
  there. `blink_fast` had no other user and is gone with it, along with
  `FAST_BLINK_PERIOD`.
- **The held colour is no longer a second purple.** It was the confirmation
  purple over a page tinted a different purple - two shades of the same hue
  meaning different things. It is now the colour of the act itself, which on the
  clear page is what the page is already tinted.
- **Clearing is pink.** Purple means "off" or "default" in half the settings
  lists and "selected" in the other half, which is far too neutral for the one
  page whose whole job is destructive. `HUE_PINK` sits between magenta and red,
  clear of both.
- **A committed copy, save or assign flashes purple**, not green - the same
  purple the source was held in, so the flash says "this is where the thing you
  picked went" in the colour you picked it with.

## Three shape modes, and settings that stop at the ends

### Changed

- **Ctrl-page settings clamp instead of wrapping.** Shape mode, quantize mode,
  cross-modulation mode, output clamp and pattern length are short lists of
  unrelated states, so rolling off one end into the other was the largest
  change available on the page and never a deliberate one. Every list has its
  default at index 0, which makes spinning fully left a reset that can be done
  blind. `delta_modulo_step()` lost its last caller with it.
- **The stepped-random variants are one mode again.** SMOOTH, SEMI and HARD
  were the same algorithm at three hold values - a long list made out of one
  idea. `SHAPE_STEPPED` is what is left, beside the wavetable and PWM. The hold
  values stay in `stepped_random.h` because they are what that parameter's
  range means and the tests exercise it; if hold returns it should return as a
  per-channel setting, the way pattern length did.
- The parameter row has more contrast: unselected buttons drop to `VAL_DIM` 2
  and the selected one rises to `VAL_HIG`, which is roughly five times brighter
  to the eye.

### Fixed

- **Switching to the FRQ page flashed every encoder green.** The frequency hue
  was multiplied by the fast blink to mark it as a coded colour rather than a
  level. That was survivable when touching one channel lit one LED; once
  picking a parameter lit all eight, it became a row flashing in unison - and
  green, because the default frequency lands on a green ratio. It is steady now;
  the hue itself already distinguishes a ratio from a level.

### FRAM

Removing shape modes renumbers the ones after them, so `CONFIG_STATE_VERSION`
goes 3 -> 4 and older records are rejected rather than silently remapped.

## MOD, per shape mode

MOD warped the phase in the wavetable mode and did nothing in the other two.
It now has a job in each, chosen for what that shape actually wants rather than
for what was easy to reuse. SHP picks the shape; MOD is the second axis.

### Added

- **Wavetable: skew.** `phase_mod()` was two straight segments with a slope
  discontinuity where they met - a corner halfway through every cycle - and a
  curvature term riding on the same parameter, so one knob did two things. It is
  now a single rational warp, `p / (p + (1-p)·r)` with `r = (1+mod)/(1-mod)`:
  monotone, both cycle endpoints fixed, smooth everywhere, one divide and no
  `expf`. Negative leans the shape early, positive late - the sign convention
  the other modes follow.
- **Stepped random: density.** MOD scales the tie probability that was the
  fixed `SR_HOLD_PROBABILITY` 0.30. Fully left every step takes a new value;
  fully right the cycle is a handful of long notes. The provisional phase skew
  is gone.
- **PWM: an AD envelope.** MOD 0 is a hard gate; negative snaps up and decays
  through the off-time, positive swells across the on-time and drops at the end.
  Each ramp is confined to its own segment, so the width still means what it
  says at either extreme. The ramps are curved - concave attack, convex decay -
  which is what makes the negative side read as an envelope rather than as a
  triangle. A narrow pulse with MOD hard left is a beat-locked percussive
  envelope, which the module could not previously make.

### Changed

- **The stepped-random normalisation table has a probability axis.** It was
  generated at the one fixed probability, so the correction drifted as soon as
  MOD moved off it - the symptom being a pattern going quiet at some settings
  and not others. Now `[length][probability][morph]`, 8 probability bins over
  `[0, SR_HOLD_MAX]`, interpolated on both bin axes. Costs ~86KB of flash (47%
  used, from 31%). `SR_HOLD_MAX` and `SR_PROB_BINS` are emitted into the
  generated header, so the runtime cannot disagree with the table's own axis,
  and the short-pattern fade moved out of the generator into the runtime.
- `just sr-table` regenerates the table. It had been regenerated by hand, which
  is how it came to still carry `sr_length_param[]` after nothing read it.
- **`wave_fn()` is gone.** Nothing ever called it - `channel_compute` has always
  used the table - so it was a second shape engine that never ran, and it was
  where the phase warp was first written. Its file kept `wavetable_lookup()`
  and is now named after it: `wave_fn.{c,h}` -> `wavetable.{c,h}`.
  `smoothstep_edge()` in helpers.h lost its only caller with it.

### Added tests

- `test_helpers.c`: the phase warp's endpoints, monotonicity, identity at the
  centre, sign convention, and the absence of the corner the old one had.
- Density is asserted directly - counting how many of a pattern's steps take a
  new value - and the loop-closure and range properties are re-checked across
  the whole MOD sweep, since density changes which slots are sources.

## Held presses, the parameter row, and a debuggable module

### Changed

- **A held press now shows what letting go would do.** Nothing acted until the
  release and nothing was shown until after it, so the whole gesture was
  invisible while it was happening. An element being held wears the colour of
  the act, blinking, and a press with a wider second stage - CLR on a channel:
  this scene, or all of them - says so by getting brighter at the threshold.
  Which stages exist comes from `ui_sel_press_stages()` in the selection model,
  so the renderer is not re-deriving the rules the dispatcher follows.
- **`UI_T_HOLD` 100ms -> 150ms**, so latching a shift mode takes a deliberate
  press. The tests that used to hold for a hardcoded 150ms now hold for
  `UI_T_HOLD + 50ms`, so retuning the threshold cannot silently change what
  they cover.
- **The parameter display covers all eight encoders at once** and lasts 2s
  instead of 1s. Seeing where a parameter sits is a comparison between
  channels, which needs the row lit together - and that made the per-channel
  timers pointless, so `channels_edit_hold[8]` is one `param_display_hold`.
  Tapping a parameter button arms it too, including re-tapping the current one;
  the tap that *leaves* a shift mode does not, so exiting lands back on the
  output monitor.
- `channels_edit_hue[]` is gone: the frequency hue is derived from the stored
  parameter (`ui_channel_freq_hue()`) rather than cached when an encoder moves.
  The cache was only ever filled for the channel being turned, which is why
  lighting all eight at once used to paint the untouched ones red.
- **Unselected parameter buttons glow very dim** in their own colour with no
  mode active, so the row says which colour belongs to which parameter. They
  stay dark inside a mode, where only that mode's button is lit.

### Fixed

- **The MCU-viewer setup could not read the module any more.** It samples fixed
  addresses, and the refactors had taken away every global it named. Two
  changes bring it back without giving up the layering: the 2-deep `HwState`
  ring becomes a named `curr`/`prev` pair - only one tick of history was ever
  wanted, and the ring's address alternated every tick, which a sampler cannot
  follow - and the firmware's single `BmcvInstance` gets external linkage and a
  declaration in `bmcv.h`, so `bmcv.engine_state.dac_fps` and everything beside
  it has a stable address. Other hosts still allocate their own instance.
  `tools/bmcv.mcuvproj` is repointed accordingly, including the per-channel
  arrays that became `channels_effective` members.

### Web simulator

- Enabling the clock generator zeroes IN0's fader. Whatever it was left at
  became the gate's low level, so a jack sitting below 0V produced a clock that
  never crossed the threshold.
- The scroll wheel turns an encoder over its centre cap as well as its ring;
  the middle of the knob was a dead spot.
- The crossfader handle wears the same grey as the encoder bodies instead of
  being the brightest thing on the panel.

## MOD freed, output clamp, PWM

`CH_PARAM_MOD` meant two unrelated things: a continuous phase warp in
`SHAPE_LFO`, and a *discrete pattern-length index* in the three stepped modes.
That one overload is what forced `val_neighbour` stepping into the parameter
editor, a mid-cycle latch in the engine, and per-scene crossfading of a value
that cannot meaningfully be crossfaded - there is nothing between 8 steps and
12, so any blend of two scenes landed on a length neither of them asked for.

### Changed

- **Pattern length is a per-channel setting** (`ChannelConfig.sr_length_idx`),
  edited by the channel encoders on the **STA** page. Dark and inert on channels
  that are not in a stepped mode, since there it would do nothing. The engine's
  cycle-boundary latch stays, now latching the setting rather than a value
  derived from MOD.
- **`CH_PARAM_MOD` is continuous again in every mode**, like AMP and OFS, and is
  passed to `stepped_random()` as a second shaping parameter. Its mapping there
  is provisional - a monotonic skew of where the step boundaries fall, which is
  a swing and keeps the loop closing seamlessly - pending a design for what it
  should really do. It does nothing in PWM yet.
- **AMP is the peak swing, not half of it.** A channel at full amplitude now
  covers the whole ±10V the converter can reach on its own; it used to top out
  at ±5V unless an offset or cross-modulation pushed it further, which left the
  wider output clamps unreachable from the oscillator alone. Every level in a
  stored patch doubles - which is moot, since the record version changed anyway.
- **Per-channel output clamp** on the **SAV** page: ±10V, ±5V, 0..10V, 0..5V.
  Applied after cross-modulation, so it bounds what actually leaves the module,
  and it clamps rather than rescales - a channel set to 0..5V does not quietly
  halve everything already dialled in. Not per scene and not touched by a
  channel clear: it describes the rig, not the patch.
- **New shape mode, `SHAPE_PWM`**: a square whose duty follows SHP, kept off
  both end stops so the pulse never disappears. Appended to the enum, so saved
  shape modes keep meaning what they did.
- **A long-press channel clear now clears the whole channel** - MON routing and
  cross-modulation mode, QNT mode and trigger source, shape mode and pattern
  length, on top of every scene's parameters. A channel that reads as cleared
  while an input is still modulating it was a half-state. The output clamp is
  the deliberate exception.
- MUT shows green for passing as well as purple for gated: on the page whose
  subject is mute, "not muted" is a state too, and a dark ring read as neither.
- `INPUT_AMP_MULT` moves from magenta to cyan - at this brightness magenta is a
  hair from the purple that means "off" on the same page.
- `channel_reset` no longer loops the scenes itself; `channel_reset_param`
  already does, since it started honouring `scene < 0`.
- `sr_length_index_from_mod()` and the generated `sr_length_param[]` table are
  gone with the parameter overload that needed them.

### Web simulator

- The input scope's height is taken from the output scope rather than from an
  aspect ratio of its own width, so the two boxes end level whatever widths the
  grid gives them, and its cells carry the same ±10V markers.
- The clock cell's pulse button and bpm box share a row.

### FRAM

`ChannelConfig` gained two bytes, so the record length changed and
`CONFIG_STATE_VERSION` goes 2 -> 3. Presets written by an older build are
rejected on load, which is the existing first-boot path.

## LED language: one fact per LED

The panel had two vocabularies fighting over the same 21 LEDs. Purple meant
"selectable" *and* "off" *and* "muted"; a candidate pulse at 50% duty sat on top
of whatever state colour was underneath, so half the time an element showed the
wrong thing; and a channel's ring showed its output level in every mode, under
whatever that mode was editing. The rules are now stated once, in the README and
in `color_presets.h`, and the renderer follows them.

### Changed

- **A shift mode's channel LEDs show that mode's own setting, and nothing else.**
  New `ChannelBaseLayer` in the mode table, alongside the `SceneBaseLayer` that
  already existed. Output level appears in one arm, selected only by no-mode; a
  mode with no per-channel setting leaves the ring dark rather than falling back
  to the level. `render_channel_edit` shrank to the one case that is genuinely
  transient - the selected parameter, with no mode active.
- **White is the whole vocabulary of assignment.** A pickable element keeps its
  own colour and flashes white for 200ms every 1.6s (`blink_mark`); once a
  source is held, valid destinations are steady white with the flash inverted
  into a dropout, the source is steady purple, and **everything else goes
  dark**. Previously a non-target kept showing its state and read as pressable.
  In MON that means the other seven channels no longer sit there previewing
  their outputs while an input is being routed.
- **Setting-state hues are shared across pages** (`HUE_STATE_*`): index 0 of
  every settings enum - disabled, default, LFO - is purple, cyan is continuous,
  yellow is triggered or clocked, and so on. Each mode used to pick its own
  four colours, so "off" was red on one page and green on another.
- CPY and CLR each have one tint - blue for copy, purple for clear - across
  their channels and scene buttons, since on those pages every element is the
  same act.
- Confirmations: clearing is purple rather than red, loading is cyan. Red is
  errors only.
- **MUT**: rotating is absolute - right unmutes, left mutes - so a row can be
  muted by feel. The press is still a toggle.
- **No mode**: a press on the encoder clears the selected parameter in the
  active scene, and a hold clears it in every scene; both flash purple, longer
  for the wider one. It was a long press only, silently, and only ever touched
  the active scene. `channel_reset_param` now honours `scene < 0` as its header
  had claimed all along.

### Fixed

- **Leaving a shift mode flashed every encoder red.** Mode entry armed the
  transient value display on all eight channels; on the way out to no-mode that
  display is the frequency parameter, whose hue is zero - red - until the
  encoder has been turned at least once. The reveal is gone, because a mode's
  setting is now shown for as long as the mode is active and there is nothing
  left to reveal.
- `web/frontend-check.mjs` checked element ids against a hand-written list of
  what `index.html` contains. It reads the page now.

### Web simulator

- BPM is two readouts: what the clock input measures, and what the oscillators
  are actually running against. They differ whenever the clock stops - the
  engine free-runs at the last tempo - and only the second was shown.
- The channel table shows the phase *offset* that was dialled in rather than the
  live phase, which is already on the scope and moving, plus each channel's
  waveshape mode. Both names and values come from the firmware
  (`BMCV_EFF_PHASE_OFS`, `bmcv_sim_shape_mode_name`).
- Bigger grab ring on the encoders; header tagline and the redundant sentence
  about buttons removed.

## Frontend split

`web/main.js` was 926 lines doing wasm binding, base64 `localStorage`, SVG panel
construction, LED gradients, encoder and slider gestures, two canvas scopes, CV
and clock generators, and the parameter table - the least modular thing in a
tree whose whole theme is modularity. It is now nine modules, none over 300
lines, with `main.js` down to wiring and the frame loop:

| module | what it owns |
|---|---|
| `sim.js` | the wasm module behind readable names; nothing else touches an `_bmcv_sim_*` symbol or a heap pointer |
| `spec.js` | `panel.json` and everything derived from it |
| `panel.js` | the SVG panel and its pointer handling |
| `leds.js` | the 21 WS2812s and their gradients |
| `scope.js` | both scopes |
| `inputs.js` | input faders, gate buttons, clock generator |
| `readouts.js` | the readouts and the channel table |
| `storage.js` | the `localStorage` mirror |
| `const.js` | the numbers more than one of them needs |

### Added

- **`web/frontend-check.mjs`** (`just web-check`): stands up enough of a DOM to
  import all nine modules for real, against the real `bmcv.wasm` and the real
  `panel.json`, then drives frames and asserts the module responded. It catches
  what a static check cannot - a missing export, a module-scope reference to
  something not yet defined, an element id that is not in `index.html`, an
  import cycle leaving a binding uninitialised - without opening a browser.
  `web/smoke.mjs` covers the wasm boundary; this covers everything on top of it.
- `just check` runs everything checkable without hardware or a browser: unit
  tests, golden flows, wasm smoke, frontend check.

### Fixed

- **The drawn crossfader position was published before the saved patch loaded.**
  Restoring reboots the module, which puts `slider_raw` back at one end, so the
  panel and the engine disagreed about where the handle was until the first
  drag. Publishing is now an explicit step in `main.js`, after `restore()`,
  rather than a side effect of building the panel.
- The input scope called `getBoundingClientRect()` every frame to decide
  whether to resize, which forces a layout for a size that only changes when
  the window does. Both canvases now size off a `ResizeObserver`.

### Changed

- The two scope draw functions were near-identical copies; cell drawing (grid,
  decimated trace, border) is shared, and the two differ only in their
  dimensions and whether cells are labelled.
- `IN_ORDER` is derived once in `spec.js`. The fader overlay and the input
  scope both lay out cells in panel order and have to agree - they each
  computed it before.

## Layering cleanup

A review pass over the UI/engine split and the simulator. Behaviour-preserving
except where noted: all four golden flows in `sim/flows/` still match byte for
byte. See `docs/review-ui-engine-split.md` for what was found and what is left.

### Fixed

- **Four test files never failed the build.** Their `main` called
  `TESTKIT_SUMMARY()` without `return`, so it fell off the end and exited 0 -
  ctest reported a green run no matter how many checks failed
  (`test_instance`, `test_panel_spec`, `test_sim_rt`, `test_input_state`).
  Fixed, and `testkit_summary` is now `[[nodiscard]]` so dropping the value is
  a compiler warning rather than a silent pass.
- **A channel counted as triggering another channel from 0.31V.**
  `CHANNEL_TRIG_THRESH` was the input-side value 1024 copied across without
  rescaling: identical numbers, but ADC counts are a quarter of DAC counts at
  the same voltage. Both domains now derive from one documented pair
  (`TRIG_THRESH`/`TRIG_THRESH_DAC`, ~1.25V rising with ~0.98V hysteresis), so a
  trigger means the same voltage wherever it is measured.
- **A save that failed still flashed green.** `SCN_PRESET` ignored
  `ux_preset_store`'s return and emitted the confirmation unconditionally,
  while the adjacent load path checked and raised an error. Store is now
  symmetric (`ERR_PRESET_STORE`).
- **A failed autosave was never retried.** `last_crc` was updated whether or
  not the write succeeded, so the next interval saw no change and did nothing
  until the user edited something else.
- **`preset_store`/`preset_load` did not guard a negative slot**, which would
  wrap the `uint16_t` address and read or write an arbitrary part of FRAM. Not
  reachable from any current caller; the simulator's equivalents already
  checked, and the two backends should not disagree about who validates.
- **Scene buttons 4-6 blinked as candidates in MON and QNT**, where the scene
  row addresses the four input jacks and those three buttons do nothing. The
  renderer drew them dark and then let the context layer paint over it; the
  handler had the guard and the renderer did not.

### Changed

- **Mute is applied by the engine, not by whoever reads the output.**
  `channel_output_level()` advanced the ramp as a side effect and carried a
  prose contract that any host reading outputs must call it - so a host that
  read `channels_output_level[]` directly got ungated audio and a mute ramp
  frozen in place. `engine_tick` now calls `channel_apply_mute` once per tick
  and publishes `EngineState.channels_gated_level[]`; the firmware and the
  simulator both just read it.
- **The DSP takes what it touches.** `channel_compute`, `channel_detect_trigger`,
  `channel_apply_mute`, `channel_init`, `channel_reset*` and `channel_take_trig`
  took `UxState*` - the whole module, interaction layer and preset vtable
  included - and used only a channel index from it. They now take
  `(uint8_t ch, EngineState*, const EngineConfig*, const HwState*)` as needed,
  which is what makes the layering structural rather than a convention, and is
  the shape an audio thread in a plugin host wants.
- **`input_fold.c` (was `input_state.c`) is a transducer again.** It drove the
  clock, reset every channel's phase, cleared error flags and ran the config
  autosave. It now latches `HwState.clock_pulse`/`clock_reset` and stops;
  `engine_tick` acts on them with the same timestamp, and the autosave moved to
  `ux_state.c`, which owns the preset io. `InputFrames` is the frame ring and
  nothing else.
- **The mode table drives the scene LEDs and the keyboard overlay.**
  `render_scene` branched on the descriptor and then re-checked `shift_state`
  three more times for SYS, SAV and STA/STB; `quantizer.c` and
  `render_quantizer` each named `SHIFT_STATE_QNT` themselves. New descriptor
  fields `scene_btn_base`, `xfade_end` and `keyboard_overlay` cover all of it,
  and `SCN_SET_A`/`SCN_SET_B` collapse into one `SCN_SET_XFADE`. Adding a mode
  is a row in `ui_mode.c`.
- **`ui_select.c` no longer names a shift mode.** Arming QNT's trigger mode and
  clearing MON's routing were written inline in the generic "pick a source"
  path. They are `assign_trig_arm_channel` / `assign_input_clear`, dispatched on
  `UiAction` - the vocabulary that file does own.
- **`channel.c` and `scene.c` split by layer.** `channel.c` was 510 lines of
  DSP, UX dispatch, config mutation, presentation tables and a driver call;
  the interaction half is `ui_channel.c` and the scene buttons are `ui_scene.c`.
  The one remaining driver call moved to `bmcv.c`, so nothing in the core
  touches a peripheral.
- **`state.h` split into `config.h`, `hw_state.h` and `engine_state.h`** -
  persisted record, hardware frame, running state - with `ShiftStates` moved to
  `ui_mode.h`, the file that enumerates what each mode does. No umbrella header:
  every file now includes only the layer it uses.
- Names follow the file they live in: `channel_*` for the signal path,
  `ui_channel_*`/`ui_scene_*`/`ui_ctrl_*` for the handlers, `ux_update` for the
  pass, `input_fold` for the sample-to-frame step.
- **First-boot defaults are `config_defaults()`**, next to the validation that
  has to accept them, rather than inline in the composition root.
- `bmcv_sim_mode_name()`/`bmcv_sim_mode_count()` expose the firmware's own mode
  names, and `dump_hw_setup` emits them as `ctrl_names`. The list
  STA/SYS/QNT/... existed in four places - the mode table, the CLI,
  `gen_panel_spec.py` and `web/main.js` - and four copies can disagree. The
  other three are now derived.
- `engine_fps` is measured inside `engine_tick`, so every host reports it the
  same way; both it and `dac_fps` are readable through the sim API.
- Ctrl-button ids mean three things at once (a `ShiftStates`, a
  `ChannelParameters`, an index into `ctrl_button_color[]`) and nothing said so.
  Two `_Static_assert`s in `ui_mode.c` now do.
- `TRIG_SRC_INPUT()`/`TRIG_SRC_CHANNEL()` name the composite trigger-source
  index space that was open-coded as `N_INPUTS + c`. Channel-indexed arrays say
  `N_CHANNELS` rather than `N_ENCODERS`, with a `_Static_assert` where the two
  being equal is relied on.
- `SLIDER_MIN_VALUE`/`SLIDER_MAX_VALUE` moved to `hw_setup.h` (analog
  calibration), the blink periods to `ui_state.h` (presentation),
  `FRAM_CONFIG_SLOTS` to `config.h` so a core file no longer includes a driver
  header for it.
- Both native builds compile with `-Wmissing-prototypes`; `led_set_adcr` and
  `led_set_dac` are wrappers over one `led_set_bipolar`.

### Removed

- **`envelope.c` / `envelope.h`**, an old experiment nothing referenced. It was
  in `BMCV_CORE_SOURCES`, so it compiled into all three builds; the linker was
  already dead-stripping it, which is why the firmware is the same size without
  it.
- `compute_channel_scene()` (empty, no callers, no prototype), `CTRL_DEFAULT`,
  `UxState.last_ux_update` (write-only), `UxState.dt` (a copy of `ui->in.dt`),
  and the commented-out level tables in `channel.c`.
- `ui_render_arm_all_edits` - arming a UI timer is `ui_state.h`'s business, not
  the renderer's (`ui_show_channel_edit` / `ui_show_all_channel_edits`).

## Virtual BMCV

Work toward running the real firmware logic off-target - headless, then in a
browser, later as a VCV Rack module. See `docs/plans/virtual-bmcv.md`.

### Added

- **Generated panel spec** (`panel/bmcv_panel.json`, `panel/panel_layout.h`,
  `panel/bmcv_panel.svg`, `web/panel.json`) built by `just panel` from three
  sources that cannot disagree: `BMCV.kicad_pcb` (board outline, placements and
  body sizes), `netlist.ipc` (WS2812 chain order, jack-to-converter paths), and
  the firmware's own `HwSetup`/`UxSetup` tables via `tools/dump_hw_setup.c`.
  Nothing in the layout is hand-typed - every button, LED, encoder and jack
  position is derived, because each WS2811 is mounted directly behind the
  control it lights, which resolves LED index to physical part by co-location.
- `tools/gen_panel_spec.py`, `tools/dump_hw_setup.c`, `panel/overrides.json`
  (the three facts the CAD data does not carry, listed as `assumptions` in the
  generated spec).
- `tests/test_panel_spec.c`: asserts every `UxSetup` pairing is physically
  co-located, that exactly three buttons have no LED, that no two controls
  share a position, and that everything lands inside the board outline.
  Swapping two entries in any `hw_setup.c` index table fails it.
- **`input_state.c`**: the step between raw hardware and `HwState`, hoisted out
  of `bmcv.c`. Takes a plain `InputSample` (slider, CV levels, latched gate
  edges, button levels, free-running encoder positions) and does the frame ring
  buffer, clock trigger/reset dispatch, slider CV summing, CV scaling, encoder
  deltas and the periodic config autosave. `input_trig_step()` exposes the
  ADC driver's gate hysteresis so a host can reproduce the same edges.
- `tests/test_input_state.c`: 77 checks over paths that previously had none -
  clock dispatch, reset-before-clock ordering, `INPUT_SLIDER` CV summing,
  autosave, encoder int16 wraparound, gate hysteresis.
- **`ChannelEffective`** in `EngineState`: per channel, what it is actually
  doing after the scene crossfade and the parameter maths - frequency in Hz and
  as a ratio of the beat, phase, shape, mod, peak amplitude and DC offset. This
  replaces the ad hoc `cgcd`/`cphsc`/`csphs`/`cfrm`/`cshp`/`cmod` fields, which
  were written every tick and read by nothing. Exposed by
  `bmcv_sim_effective()` and shown in the web frontend's channel table, which
  now reports what the module is doing rather than what was dialled into the
  active scene.
- **`instance.c`**: `BmcvInstance` is one module in one struct - config, signal
  path, interaction state, input layer and the wiring between them.
  `bmcv_instance_init()` is the power-on sequence; `bmcv_instance_tick()` is
  input fold + engine tick. The firmware holds one static instance; a
  simulator or VCV Rack patch holds one per module.
- `tests/test_instance.c`: two instances must not share a clock, error flags,
  input/UI state, preset storage or the LED framebuffer.
- **`sim/`**: the firmware's core behind a flat C API (`sim/include/bmcv_sim.h`)
  that speaks volts, 0..1 slider positions and encoder detents rather than
  converter counts, so a frontend needs none of the firmware's types.
  `bmcv_sim_run()` drives the real `engine_tick`; outputs, the 21 LEDs and an
  8-channel scope ring are readable as flat arrays.
- **`sim/src/sim_rt.c`**: host-side glue shared by every frontend - unit
  conversion, tick decimation and gate latching. Time accumulates in Q32
  microseconds: at 44.1kHz a rounded integer step would run the engine 0.17%
  slow, putting every LFO permanently flat.
- **The simulator's preset store persists across browser sessions.** The
  module autosaves its config to its last slot every couple of seconds; the
  frontend mirrors that blob into `localStorage` on the same cadence and
  restores it on load, so a patch survives a reload. Reset clears both. A blob
  whose length does not match `bmcv_sim_storage_size()` is rejected, and
  anything past that is still validated by `config_validate()` on boot.
- **Input panel rebuilt** as one canvas on a 2x2 grid laid out like the input
  jacks, matching the output scopes, with the controls overlaid on the cells
  rather than wrapped in boxes and labels of their own. Each cell carries a
  vertical fader, the trace of what the engine actually latched, and a pulse
  button; the clock generator sits on input 0's cell as a bpm number field and
  disables that fader while it runs. The fader fills from the centre outward,
  so the bar means "offset from 0V" - a native range input fills from its
  minimum and put a solid bar at -10V. `bmcv_sim_input_scope()` exposes the
  per-input history.
- The panel controls are restyled for the light artwork: dark outlines and
  spokes, a nearly transparent knob fill, lighter switch caps and a stronger,
  wider LED glow, since a wash that read well on a dark mock-up disappeared on
  a silver panel. The slider draws only its handle - the artwork has the slot -
  and the wiper travel (49mm) is now measured off that slot rather than guessed
  from the potentiometer body.
- The encoder cap is sized to the 7.03mm cutout in the panel artwork, leaving a
  hairline of it showing, with a correspondingly larger knob body. Legends are
  bold; the crossfader handle is shorter, its hit area uses a horizontal resize
  cursor, and the switch caps are lighter.
- Button legends are drawn rather than left to the artwork, so they stay correct
  if a button is ever reassigned: the parameter above (FRQ, SHP, ...) and the
  shift mode below (STA, SYS, ...), with CLR/CPY/MUT taking the lower slot
  alone since latching a mode is all they do.
- **The panel is drawn like the parts.** Buttons are round: 6mm for the
  illuminated switches, 7mm for the three unlit tactiles. Only the ctrl
  functions carry a legend and it sits above the button - the scene numbers and
  semitone names are gone, since a 6mm cap has no room and the layout already
  says which is which. Encoders have a smaller body (the cap is unchanged) and
  four spokes rather than one, because a single mark reads as an absolute
  pointer and these are endless relative encoders. Their LED is a ring around
  the body rather than a tint on the face: the WS2812 is behind the encoder, so
  its light spills around opaque plastic rather than through it.
- Output-jack and channel labels dropped, along with the slider's SCENE label.
  Separate **Reset module** (reboot, keep presets) and **Reset FRAM** (wipe
  presets and the browser copy) buttons.
- The input scope's cells are the same size as the output scope's, at the same
  pixel density and the same time per pixel, so a trace means the same thing in
  both. It reads at half the canvas width because it is 2 columns to the output
  scope's 4.
- **The exported panel artwork (`web/bmcv_panel.png`) is the panel background**,
  drawn 1:1 over the 81 x 128.5mm panel rectangle with the live controls on
  top. It lands exactly on the generated geometry, which is a useful check on
  the whole KiCad-derived pipeline. The drawn ctrl legends are gone with it -
  the artwork carries them, and it names the *parameter* (FRQ, SHP, MOD, ...)
  rather than the shift mode. The spec still exposes both as roles, for the
  hover hint and for anyone rendering without the artwork.
- LED edges are soft: each one paints through its own radial gradient rather
  than a flat fill - solid through the middle with the falloff only at the rim,
  rather than a bright point fading linearly all the way out. Done with
  gradients rather than an SVG blur filter because a paint server costs nothing
  to rasterise. Encoder bodies are slightly translucent so the colour bleeds
  through the knob as well as around it.
- The channel parameter table uses `table-layout: fixed` with tabular figures;
  it used to resize its columns on every value change and jitter constantly.
- **`bmcv_sim_cli`**: drives the module from a scripted input timeline
  (`sim/flows/*.txt`) and prints outputs / LEDs / UI state as CSV. Scripts name
  controls by role (`ctrl QNT 1`, `scene 2 1`, `chbtn 0 1`) rather than by
  button index.
- **`sim/flows/`** with `just flows` / `just flows-bless`: golden-file coverage
  of whole interactions - shift-mode entry and exit, mute, clock lock, param
  selection - which unit tests cannot express.
- `tests/test_sim_rt.c`: 125 checks over the shared runtime glue, including
  no-drift-over-ten-minutes at 44.1k and 48k, and a gate shorter than one tick
  still being seen.
- **WebAssembly build** (`just wasm`) of the same `sim/` project via emscripten,
  producing `web/bmcv.js` + `web/bmcv.wasm` (~105KB). The export list lives in
  `sim/CMakeLists.txt` and is explicit: `bmcv_sim` is a static library, so
  anything unnamed is dead-stripped before it can be exported.
- **`web/`**: a browser frontend that builds its whole panel from
  `web/panel.json` - nothing in the JS knows where any control is. Encoders
  turn by dragging the ring or scrolling and push via the centre cap (Shift
  while turning gives press-and-turn); the 21 LEDs are driven from the real
  framebuffer with a gamma lift so they read like WS2812s rather than flat
  fills. An 8-lane scope, scene contribution bars, a live parameter table and
  CV/clock generators fill in what a panel photo cannot show.
- **`web/smoke.mjs`** (`just wasm-check`): headless node check that the module
  loads, that all 26 exports are present, that heap views line up, and that
  driving it through the flat API actually moves the engine - so a broken wasm
  build is caught without opening a browser.

### Changed

- **`bmcv_state_update` is now only peripheral reads** (~25 lines): it fills an
  `InputSample` and hands it to the core. Every `bmcv_*` entry point keeps its
  signature, so `main.c` and the HAL callbacks are untouched.
- **Presets go through a `PresetIo` vtable on `UxState`** rather than direct
  `preset_store`/`preset_load` calls from `scene.c`. The backing store is FRAM
  on the module and will be patch JSON / browser storage off it. A NULL
  `PresetIo` is valid: store is a no-op, load reports "nothing stored".
- `ui_render.c` no longer calls `get_adc` for the MON scene LED - it reads the
  same value from `hw_state->input_state[]`, which makes that path assertable
  and removes the last driver dependency from the render layer.
- `TRIG_THRESH`/`TRIG_THRESH_LOW` moved from `dac_adc.h` to `hw_setup.h`,
  alongside the CV range constants and for the same stated reason: a simulator
  needs them without pulling in a driver header.
- `tests/fakes/fake_drivers.c` is down from five stubs to one (`dacadc_write`).
- **`channel_output_level()` split out of `write_channel_dac()`.** Mute is an
  output-stage gain, not a zeroed `channels_output_level`, so what actually
  leaves the module was only computed inside the function that talks to the
  DAC. Any host reading `channels_output_level[]` directly would show a muted
  channel as live - the simulator did, until this split.
- **The clock is per instance.** `g_clk` is gone; `ClockState` is a field of
  `EngineState` and every `Clock_*` takes a `ClockState*`.
- **Error flags are per instance.** The `error.c` file static is gone;
  `error_flags` is a field of `EngineState` and every `error_*` takes an
  `EngineState*`.
- **`bmcv.c` holds a single `BmcvInstance`** instead of six file statics, and
  `bmcv_init` is now pin setup plus one `bmcv_instance_init` call. Every
  `bmcv_*` entry point keeps its signature.

### Fixed

- **Panel geometry came from the wrong file.** It was read from
  `production/positions.csv`, which is a *pick-and-place centroid* export: for a
  footprint whose pads are not symmetric about its origin the centroid sits to
  one side, and the offset flips sign with the footprint's rotation. The jacks
  are on a uniform 13mm grid but alternate 90/270 degrees, so they came out at
  14.03/11.97mm alternating - visibly wrong. Geometry now comes from the
  footprint placements in `BMCV.kicad_pcb`, which also yields the board outline
  from Edge.Cuts, so the board size and the panel offset are derived instead of
  typed into the overrides.
- **The simulator booted showing scene 6 until the slider was touched.** The
  frontend drew the crossfader at one end but never told the engine, which had
  come up with `slider_raw` at the other. The drawn position is now published at
  startup, and there is one function that moves both.
- **The browser tab could freeze solid on load.** `bmcv_sim_run` took an
  unsigned `n_ticks`, and the frontend computed it from `rAF` timestamp minus
  `performance.now()` without a lower bound. rAF hands a callback the *current
  frame's* start time, which can be earlier than a `performance.now()` sampled
  just before scheduling it, so the first frame sometimes produced a negative
  count - which arrived in C as ~4.29 billion ticks and blocked the main thread
  for good. No JS error, no crash event, just a dead page, and intermittent
  because it depended on frame timing.
  `bmcv_sim_run` now takes signed counts, rejects zero or negative and caps at
  `BMCV_SIM_MAX_TICKS`; the frontend clamps elapsed time at zero. Covered by
  `web/smoke.mjs`. Eight consecutive page loads: 8/8 alive, previously about one
  in three froze.
- **A fresh module no longer boots into an error display.** Finding no stored
  config raised error bit 6, which `render_error` draws by blanking every LED
  and blinking scene 6 until the next interaction - a startup screen announcing
  that nothing is wrong. An unused module having no saved config is its normal
  state, and `config_validate()` has already made the defaults safe to index. A
  slot the user *explicitly* asks for and cannot read still reports (`scene.c`,
  error bit 5).
- **The 2s config autosave is silent.** It flashed the whole scene row green on
  every write, so idling with a knob just moved produced a confirmation nobody
  asked for. Confirmations are for committed actions; background housekeeping
  on a timer is not one. An explicit save still flashes.
- **The slider was drawn vertical with a guessed 45mm travel.** RV13's
  footprint is 73 x 16.8mm with its pads at x = +/-34, so it is *horizontal*
  with roughly 60mm of travel. The generator now reads the footprint extent
  out of `BMCV.kicad_pcb`, so the axis and body size are derived rather than
  assumed and only the wiper stroke remains an override.
- **The web panel's LEDs were invisible.** They were drawn as circles in a
  layer beneath the controls, and the opaque button and knob bodies covered
  them completely. Physically the switch cap *is* the lamp, so each control now
  carries a tint plus a halo, both transparent when the LED is off.
- **LED brightness was calibrated against the wrong ceiling.** `led_fb.c` caps
  at `VAL_MED` (32) and draws base layers at `VAL_LOW` (8); normalising against
  255 left everything the renderer actually draws invisible. Brightness now
  rides on opacity against a `VAL_MED` full scale with a perceptual curve, and
  hue on a fill normalised to full range so a dim red still reads as red.
- An SVG blur filter over a panel-sized group, re-rasterised every animation
  frame, crashed the renderer outright; the glow is a translucent oversized
  shape instead. The scope polyline is decimated to the canvas width and the
  readout table updates at 10Hz rather than 60Hz - it was the largest source of
  DOM churn on the page.
- **The scope was a fixed 1000x440 bitmap stretched to fit**, so it was visibly
  soft on a wide or HiDPI display. Its backing store now tracks the element's
  real size times `devicePixelRatio`, via a `ResizeObserver`.
- **Slider direction.** Scene A anchors at `SLIDER_MAX_VALUE` (`scene.c`) and
  sits on the left of the panel, so the leftmost position has to map to full
  scale, not zero.
- **The scope stops growing past ~1100px** and the side panels move alongside
  it on wide screens rather than the traces stretching forever. Two jacks
  abreast collapses the range inputs in the narrower column, so they go one per
  row there.

### Notes

- **There is no channel-to-jack crossing.** An earlier revision of the
  generator reported one; that was a tracing error, not a hardware or firmware
  fault. The two AD5754Rs are daisy-chained (MCU -> U34 -> U35) behind a single
  shared `DAC_1_SYNC`, so in each 6-byte transfer the *first* command shifts
  all the way through U34 into U35 and the second stays in U34 - even buffer
  indices are U35, odd are U34, the opposite of what `dac_init()`'s ordering
  suggests at a glance. With that corrected, `channel_dac_idx` puts channels
  0-3 on J1-J4 (left block) and 4-7 on J5-J8 (right block), each in the same
  reading order as their encoders. `hw_setup.c` is correct as shipped.
- **The ADC ADDR phase is resolved.** `input_adc_idx = {2,3,0,1}` only reads as
  a sensible panel if the A2/B2 converter slots land in `adc_i[0..1]`, which
  puts inputs 0-3 on J9-J12 in reading order. The other phase would number the
  jacks 2,3,0,1 across the panel, which nothing else on the module does.

## UI layer restructure

Separated the interaction layer from the engine and gave the recurring UI
concepts one implementation each.

### Added

- **MUT shift mode.** Channel button toggles that channel's mute on release.
  Gates the DAC write only, so a muted channel still feeds cross-modulation and
  still works as a trigger source. Ramps over 5ms so it does not click. Renders
  dim purple in every mode. Not persisted - clears on power cycle.
- **Confirmation feedback** (`ui_feedback.c`) for copy, clear, save, load,
  assign and scene A/B - same flash, same timing, colour by action class. Most
  of these previously gave no feedback at all.
- **Semantic palette** in `color_presets.h` plus `HUE_PURPLE`. The renderer
  names meanings, not colours. Brightness capped at `VAL_MED`.
- **Candidate highlighting** driven by the same predicate the dispatcher uses,
  so what blinks and what responds cannot disagree.
- Tests: `test_ui_input`, `test_ui_gestures`, `test_ui_select`,
  `test_ui_feedback`, `test_ui_mode`. The button path had no coverage before.

### Changed

- **Button gestures derived once** (`ui_input.c`) as edge events, with the
  press durations defined in one place. Events accumulate across ticks where
  the UX layer does not run, so a hold crossing its threshold is never missed.
- **`EngineState` split**: signal path only; interaction and view state moved
  to `UiState`. `compute_scenes_contribution` is now pure.
- **Selection model** (`ui_select.c`) replaces the assign type/target matrix
  and the mode-specific exceptions that lived in its callers.
- **Layered renderer** (`ui_render.c`) replaces the four `write_*_led`
  switches: base, context, transient value, confirmation. The base layer always
  writes, so stale pixels are impossible.
- **Channel LEDs** show the output level as their base in every mode; the value
  being edited surfaces on touch and decays. Entering a mode reveals every
  channel's state for that mode briefly.
- **Mode behaviour is a table** (`ui_mode.c`) read by both dispatch and render.
- **Shift-mode exit**: a mode's own button always exits; other ctrl buttons
  exit unless the mode opts out (only QNT does, because its buttons are a
  semitone keyboard). The exit tap no longer also changes the selected param.
- All UI timers age in one place against one dt.

### Fixed

- `scene.c` compared a hold against 10 microseconds where 10 milliseconds was
  meant.
- SAV drew its LED from the press timer but acted on the release timer, so the
  two disagreed at the boundary. Store now fires on the hold crossing.
- The QNT scene-button arm wrote no LED unless a trigger source was pending,
  leaving the framebuffer holding the previous frame.
- The MON channel arm fell through into the default, letting the value overlay
  clobber the mode colour.
- An error display returned early from the input update, suppressing the whole
  UX pass while it showed.
- `momentary_scene` was zero-initialised on hardware, reading as "scene 0 held"
  until the first UX pass corrected it.

### Removed

- `HwState.button_pressed_t` / `button_released_t`, `AssignType`, and the three
  ad hoc indicator timers they fed.

