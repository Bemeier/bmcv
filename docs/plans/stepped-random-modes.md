# Splitting stepped random into modes with distinct characters

Status: **phases 0 and 1 done and confirmed on hardware; phase 2 next.**
All on `spike/stepped-modes`.

## Where this comes from

Three rounds of widening SHP and MOD landed (range, layering, level). The
verdict: it works, the areas are good, but

1. **the typical results are hard to find again** - one mode covering everything
   from a bassline to a slow modulation means every useful character is a small
   island somewhere on a two-knob surface;
2. **a small turn of SHP does too little** - the coarse clicks are already quick
   enough to cross the knob, but each one barely changes the sound, which is why
   fine adjust has never been wanted;
3. **different jobs want different characters** - a baseline melody and a slow
   modulation are not two points on one axis.

The reference the feedback came with is Random8's **Style** parameter: eight
named distributions (fBm, Perlin, high-low, Weibull, exponential, gamma, ROSC,
uniform), each picked for a job. Two or three modes, not eight styles - a mode
costs the user something to learn and to reach.

## Decisions taken

- **Three modes**: melodic, modulation, drift.
- **Detent sizes stay** at 256 coarse / 32 fine. A click matters more because
  the algorithm moves more per unit of travel, not because the click is bigger.
- **The flash ceiling stops driving the design.** Prototype the modes in the
  simulator, where the correction can simply be computed, then bring the
  architecture back to the module.
- **Level centring stays exactly as it ships**: one constant shared by every
  length, so switching pattern length on the wrap is still seamless by
  construction. Centring each length exactly - which a runtime correction would
  make possible - is dropped. It would have bought ~0.2 of residual DC and cost
  the one behaviour that is currently free and exact.
- **Saved presets may break.** `ChannelConfig` and the mode enum are open, which
  also means per-channel settings a mode needs (hold, length defaults) are on
  the table.
- **This runs as a spike on a branch.** See "How the spike runs" below.

## What rounds one to three leave us

Keep all of it. The engine is the pattern, and it is good: slot orbits, ties as
a crossfade, gate rotation, swing, motif fold, easing, and the affine correction
that holds level steady (peak-to-peak near 1.5, centred, `SR_NORM_EXP` 0.7 so
the natural loud/quiet ordering survives). Also keep the rule that gives a
phase-driven module what stochastic draws give Random8: **neighbouring knob
positions stay related** (0.35 per 1% of travel from eight steps up).

What the engine does **not** have, and what the Random8 list is really about, is
an **asymmetric value distribution**. `sr_shape_blend` is odd: it compresses or
expands the middle symmetrically. Low-bias-with-occasional-highs (Weibull,
exponential, gamma) is exactly what it cannot make, and that is the family the
feedback calls "modulation".

## Architecture: tabulate the constant, compute the gain

The normalisation table is `[11 lengths][16 MOD][128 orbit]` floats x2 =
**180 KB**, against ~200 KB of free flash (firmware is 322 KB of 512 KB). Two
value paths just fit; three do not. The length axis cannot be collapsed to buy
room - measured, the gain still differs by 0.46 on average between 16 and 64
steps at the same point, and by up to 8.9 where a short pattern needs lifting.

The correction is `out = c + (v - anchor) * g`, and its two halves have opposite
shapes:

- **`c`, the centring constant, is length-independent but expensive**: it is an
  average across all eleven lengths of what would centre each, which is why it
  is shared and why the cycle boundary is seamless. A channel knows only its own
  length, so it cannot compute this.
- **`g`, the gain, is length-dependent but cheap**: the pattern's own span,
  which is `length` evaluations of `sr_step_value`.

So split them. **Tabulate `c` - `[16 MOD][128 orbit]` = 8 KB per mode, 24 KB for
three - and compute `g` at runtime.** Per sample the scan is unaffordable;
**amortised over the cycle it is one extra slot per tick per stepped channel**,
roughly +40-50% on the stepped path, and only while a knob (or a CV on one) is
moving - an unchanged pattern needs no rescan.

What that buys, beyond 180 KB becoming 24 KB:

- **per-mode value paths become free** - lever depths, rates, taps, anything;
- **the length seam becomes exact.** Today's 0.044 (and 0.122 off-centre MOD)
  is entirely interpolation: the table's gain and its anchor come from a bin
  while slot 0's value is computed at the real orbit, so the two no longer
  cancel. Compute the anchor and the gain at the real point and
  `out(0) = c` exactly, at every length, for free;
- **`SR_GAIN_SUBS` goes.** Fitting each bin's gain to the worst point in its
  neighbourhood exists only to survive interpolation, and it biases every
  setting upward. Level consistency improves as a side effect;
- **the 88 KB of MOD resolution goes with it**, and the question of whether 8
  bins would have done.

What it costs, and how each is handled:

- **CPU.** Measure before committing: `engine_fps` and `dac_fps` are reported by
  the module, and the worst case is eight channels in a stepped mode with SHP
  under CV. If one slot per tick is too slow to converge, scan a fixed subset -
  the runtime already clamps, so an underestimated peak is a clipped sample for
  a few ms rather than a failure.
- **Per-channel state**, ~6 floats: the committed gain, the accumulating
  min/max, and the scan index. The DC is not needed - it only ever fed `c`,
  which stays in the table. `EngineState` already owns per-channel latches like
  `channels_length_idx`; this sits with them. `stepped_random()` stays pure: the
  correction becomes an argument, not a hidden cache.
- **The correction changes while the pattern plays.** It must be constant across
  a cycle or the loop stops closing, so commit a completed scan on the wrap and
  slew between old and new over a few ms rather than stepping.

**The seam rule still holds, and the spike is where it is at risk.** The
prototype must not be a simulator-only behaviour: it is one function,
`sr_norm_compute(mode, length, mod, orbit)`, called directly by the sim and the
native tests, and *memoised* by the firmware for as long as it still has a
table. Same output, different cost. What must never happen is a mode that sounds
right in the browser because the browser can afford something the module cannot;
that is the trap, and the point of the prototype is to decide what the module
will do.

A mode is also free to need **no correction at all**: a construction whose span
is analytically bounded (see drift, below) skips this entirely. Prefer those
where the character allows.

## Making a small turn count

Measured: a coarse detent is 256 counts of a 65536-count parameter, so **256
detents cross SHP**; the character metric says a 2% turn (5 detents) moves ~0.8
sigma, so one click moves ~0.16 sigma - below noticing. The budget is not the
problem: sample movement per 1% is already 0.29-0.30 against the 0.35 limit.
**The budget is being spent on the wrong thing.** It goes on the orbit
re-drawing the values (jitter) rather than on the levers steering the character
(steer), and jitter is what a small turn must not do.

Three levers, to be set together against `from_c.py`:

- **Slow the orbit.** `SR_MAX_ORBIT_RATE` is 4, so a slot's value can cross its
  whole triangle four times in one sweep. Round one already measured the
  converse: widening it to 10 raised jitter 59% and lowered steer. Narrowing it
  frees budget and costs little.
- **Raise the lever rates.** A rate-1 lever spends the whole knob on one
  traversal, so a click is 1/256 of it. At rates 8-12 a click is a real fraction
  of an excursion. The trade is honest and worth stating: **a character then
  recurs several times across the knob.** That is acceptable because the rates
  are coprime and the orbit is monotone, so the *combination* is still unique
  everywhere - which is what round two's layering was for. What is bought is
  that the knob has fewer unique destinations but every click reaches one.
- **Post-maps are the cheapest steer there is.** A monotone reshaping of the
  values changes the whole feel while leaving every step's ordering intact - it
  cannot read as re-randomising, because nothing is re-drawn. The modulation
  mode's bias axis is the best example, and it is a good reason to put a post-map
  on SHP in every mode.

Target: **~0.5 sigma of character per click**, with sample movement per 1% of
travel unchanged at 0.35, `jitter` down and `steer` up.

## The modes

Three. Presets are open, so the enum can be ordered for sense rather than for
compatibility: keep the stepped family contiguous.

### 1. Melodic

Notes: a line you would quantise. Centred, mid-biased values, no spikes.

- **SHP** - interval width (calm and close together <-> wide leaps) crossed with
  walk <-> independent leaps, both at rates that make a click audible.
- **MOD** - rhythm: density (note lengths), swing, the quarter-cycle motif
  repeat. As today.
- **post-map** - mid-bias at one end, uniform at the other: Random8's ROSC (7)
  and uniform (8).
- **level** - centred.

### 2. Modulation

Something to modulate with: sits low, rises occasionally, or gates.

- **SHP** - the **bias** axis, the one genuinely new thing here: deep low with
  rare peaks (Weibull/exponential, 4-5) -> slight low bias (gamma, 6) -> uniform
  -> bimodal high/low (3, gate-like). One traversal so the ends are the extremes,
  with a faster second lever layered on it.
- **MOD** - **motion**: `hold` from fully slewed to hard-stepped, crossed with
  density. `hold` is hard-wired to `SR_HOLD_SMOOTH` in `channel.c` today and is
  the axis that turns a pattern into an envelope. Free of the correction either
  way - the span is measured on the step values and the eased curve passes
  exactly through them.
- **level** - the floor, not the mean. "Sits low with occasional peaks" *is* a DC
  offset, so the centring is re-aimed per mode and the level test asserts the
  floor is steady rather than the mean.

### 3. Drift

The invisible hand: slow, smooth, no audible steps. Random8's fBm and Perlin.

- Not the slot engine: a sum of 2-4 octaves of triangles in the **phase** domain
  at integer rates, so it closes on itself exactly and **needs no correction** -
  its span is analytically bounded.
- **SHP** - roughness: how much of the higher octaves, and which drift.
- **MOD** - excursion, and how much of the step lattice shows through (0 =
  smooth, 1 = stepped).
- Cheapest and least risky of the three; third only because the feedback named
  melody and modulation.

## What we do not take from Random8

- **PROB** - a channel advancing only sometimes is stochastic; we are phase
  driven and re-derive the value from the PLL every tick. The tie crossfade is
  the deterministic equivalent and we prefer it.
- **DIVIDR + STEPS** - already covered. STEPS is `sr_length_idx`, including
  their "shrink and regrow without losing the loop" (slot values do not depend on
  the length). DIVIDR is the FRQ ratio.

## How the spike runs

Branch `spike/stepped-modes`. The character is the thing to find first, and it
cannot be found while also holding every invariant, so during the spike:

**May be relaxed, temporarily and visibly** - each one noted in this file as it
happens, with what it will take to restore it:

- flash: the correction may be computed the slow way everywhere, no table;
- CPU: no budget on the stepped path;
- `tests/test_stepped_random.c` may run against one mode while the others are
  in flux, and the small-turn and level bounds may be loosened to explore;
- `just check-all` may fail on the firmware target;
- the manual and the LED language may lag.

**May not be relaxed at any point**, because these are what make the spike worth
merging rather than throwing away:

- one core, one value path per mode, no host-only behaviour;
- the loop closes: the waveform is continuous across the cycle boundary;
- neighbouring knob positions stay related - the small-turn property may be
  re-tuned but not abandoned, since it is what we have instead of stochastic
  draws.

**Before merging back to main**: firmware builds and fits, the whole suite
passes per mode with bounds set from measurement rather than loosened, the
flash and CPU numbers are recorded here, `just check-all` is green, and the
docs and manual describe what the module actually does. That last stretch -
getting an architecture that runs on the module - is expected to be the larger
half of this work.

## Staging

**Phase 0 - make the correction callable. DONE.** The body of
`gen_sr_table.c` now lives in `Core/Inc/Lib/stepped_random_norm.h`, split into
`sr_norm_centre()` and `sr_gain_for()`; the generator calls it and reproduces
the checked-in table byte for byte, so the refactor is provably nothing but a
refactor.

**Phase 1 - the architecture, before the modes. DONE.** Originally the other way
round, and swapped once the centre/gain split showed how cheap this is: an 8 KB
centring table plus a gain the channel scans for itself. Prototyping modes first
would mean either three 180 KB tables in the sim - modes designed against a
budget the module does not have, which is the one thing the spike must not
produce - or building them twice.

Behaviour is fixed and measurable here, which is what makes it the safe half to
do first: the target is the shipping build's own numbers.

Measured so far, with the correction computed exactly on every call
(`-DSR_NORM_COMPUTE`, spike scaffolding - the real one scans incrementally):

| | table | computed |
|---|---|---|
| worst length seam | 0.122 | **0.000** |
| worst span anywhere | 0.584 | **0.900** |
| character steer | 2.72 | 2.72 |
| DC swing along a sweep | 0.39 | 0.38 |
| small turn, SHP, L>=8 | 0.29-0.30 | 0.23-0.31 |

Both invariants improve rather than survive. The seam is exact because nothing
is interpolated any more. The span floor holds everywhere because it is now
applied at the point itself: in the table it only ever arrived through the
neighbourhood loop, which was written to survive interpolation and turned out to
be carrying `SR_NORM_FLOOR` as a side effect - computing without it dropped the
worst span through the 0.5 the suite holds, which is how this was found.

The whole existing suite passes against the computed path.

**The scan landed.** Each channel measures one slot of its own pattern per tick
and corrects with what the last full pass found; the length-indexed tables are
gone. The generated header went from 472 KB of text to 27 KB, and the firmware
from **322 KB of flash to 152 KB** (110 KB at `-Os`), against 512 KB.

Cost, measured on the dev machine at 4 kHz per channel:

| | L=8 | L=32 |
|---|---|---|
| the shape itself | 49 ns | 71 ns |
| scan, knob moving | +37 ns | +49 ns |
| scan, channel standing | +3.2 ns | +3.2 ns |

A standing channel costs almost nothing because re-measuring an unchanged
pattern can only give the answer it already has, so it stops. What the scan
really costs is a knob being turned or a CV moving one - which is when it is
earning it. Wants confirming on hardware with `engine_fps`/`dac_fps`, eight
stepped channels, SHP under CV.

There is no commit-on-wrap rule after all: the correction is slewed into place
over 20 ms instead. Latching to the wrap would leave a slow LFO uncorrected for
its whole cycle - seconds - after a knob move, and the table it replaces never
waited for a wrap either. What the slew is really protecting is a length change,
which swaps the pattern wholesale.

**On hardware, where the first arrangement failed.** Measured over an ST-Link
probe - the USB page's own snapshot streaming runs in the main loop and costs
0.6 kHz, so it cannot measure anything at this level.

| | main | scan, per channel | scan, one per tick |
|---|---|---|---|
| standing | 3.3 kHz | 3.2 | 3.2 |
| every parameter moving | 3.3 kHz | **2.5** | **3.2** |

The module runs at **3.3 kHz, not the nominal 4**, before any of this - it is
already dropping about one tick in six, which is pre-existing, unexplained, and
worth its own look before the modes ask for anything more. LED refresh is steady
at 300 fps throughout, so whatever that is, it is in or around the engine tick
rather than a starved main loop.

Against that, one measurement per channel per tick cost **88us of a 310us
tick**. The estimate that said 3% was out by an order of magnitude: it assumed
a slot evaluation was ~1us, when on the M4 it is ~11us - a step value walks a
run of ties, each slot of it a four-tap contour and a motif fold, so about
thirty triangles and a handful of square roots. The dev-machine ratio had it
right all along (the scan is 1.7-1.9x the shape's own cost); only the conversion
to the module was wrong.

So the engine hands out **one measurement per tick, in turn**, which bounds the
whole thing at one channel's worth however many are patched. What it costs is
latency: a channel gets every eighth tick, so a pass over a 64-step pattern
takes ~160ms rather than 20. The correction is a level, and it trails a moving
knob by that much before settling - which is the thing to listen for.

Re-measured with the rota: back to main's rate, sweeping or standing. On the
ear, at eight channels: level consistent across SHP, scene crossfades clean, no
audible lag from the ~160ms a pass now takes, no artefact from the glide a
length change produces.

One thing the rota also covers, unplanned: the crossfader is an ADC input, so
unless it is hard at an end its own noise keeps the scene blend moving, and with
it every channel's measurement. Before the rota that made the worst case the
normal case.

**Phase 2 - the modes**, now that a per-mode value path costs 8 KB rather than
180. `from_c.py` gains a mode column; the web sim is where they get played. Land
the character here: it is the only part that cannot be decided by measurement
alone. If a mode turns out to need no correction at all, say so and skip it.

**Phase 3 - the rest.** Config (`ChannelConfig`, the mode enum,
`CONFIG_STATE_VERSION`, a reset rather than a migration), the shape encoder and
five LED states (precedent is `clamp_mode_color`: hue plus brightness, so the
stepped family stays cyan and separates by brightness), `web/const.js`,
`frontend-check.mjs`'s "shapes reuse the input hues" check (its reason needs
rewriting, not deleting), `sim/src/bmcv_sim.c` names, `docs/led-language.md`,
the manual, and the test suite run per mode.

## Concerns

1. **The prototype must not become a fork.** One function, two cost strategies -
   never a behaviour the sim has and the module does not. Phase 0 exists to make
   that structural rather than a matter of discipline.
2. **CPU is a number nobody has yet.** The stepped path went 4.2x then 5.0x
   across rounds one and two and the module reported no change, but that was
   with the correction free. Measure at the start of phase 2, not the end.
3. **Three modes plus new axes is a lot of surface.** Each mode needs the whole
   invariant suite (loop closure, small turn, flat floor, level, cyclicity) plus
   a character claim of its own, or "distinct characters" is just an assertion.
4. **Breaking presets is cheap now and expensive later.** If the record is going
   to change, change it once, in phase 3, with everything the modes turned out to
   need.

## Unresolved questions

1. **Does the modulation mode want its own pattern length default?** Envelopes
   want fewer steps than melodies. Length is per channel; changing it on a mode
   switch would overwrite a user's setting.
2. **How far can the lever rates go before a click reads as re-randomising?**
   The suite measures sample movement per 1% of travel, not per click; a
   per-click figure probably needs to join it.
3. **Does the drift mode belong in the stepped family at all?** It shares the
   knobs' meaning and the phase lock but not the value path, and on the panel it
   is closer to the wavetable mode. It is grouped here because it is random.
4. Carried over, still live: is `SR_NORM_EXP` 0.7 the right amount of level
   normalisation; is 0.35 the right small-turn budget; and swing on odd lengths
   still makes step 0 and step L-1 both wide at lengths 3 and 5.
