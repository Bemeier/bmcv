# The stepped shape

A pattern of values, one per step of the cycle, eased between and locked to the
beat. `Core/Src/Lib/stepped.c` is the runtime; `stepped_pattern.h` is what each
step shows, shared with the table generator; `stepped_norm.h` is the correction
that holds the level steady.

**It is not random.** Every value is a deterministic function of the two knobs,
the pattern length and the phase - the caller re-derives phase from the PLL on
every tick and gets the same value back, which is what lets scenes crossfade
without the pattern drifting. What the knobs move through is a space of
patterns. That is also the constraint that shapes everything below: a phase-
driven module cannot draw fresh numbers, so what it has instead of stochastic
draws is the rule that **neighbouring knob positions stay related**.

For what the knobs do musically, see [shapes.md](shapes.md).

## How a value is built

1. **Slots.** Each step has a slot, and each slot rides its own slow orbit at
   its own whole-numbered rate. SHP advances all of them at once, which is why
   the knob wraps.
2. **Character levers**, all functions of the orbit: how the values are spread
   (`ST_BEND`), how melodic the contour is (`ST_CONTOUR`), whether the cycle
   repeats a quarter-length phrase (`ST_MOTIF`), which steps tie (`ST_SPIN`),
   where the beat sits (`ST_SWING`). Each has a depth and a *rate* - how many
   excursions it makes across its knob - so a given contour depth comes back
   paired with a different shaper each time.
3. **Ties.** A step either takes its own value or crossfades toward the previous
   one. MOD moves the density and rotates which steps tie, so the tied set keeps
   re-arranging instead of only growing.
4. **The correction**, `out = value * gain + offset`, which holds the level
   steady. See below.
5. **Two reshapings** of the finished value, both driven by SHP: the bias, which
   leans the distribution, and the terracing, which gathers values onto a few
   levels. Both monotone.

## The correction, and where it lives

Short patterns collapse. With a handful of steps the orbits can all cross at
once and the pattern flattens for a stretch of the knob - measured at length 2,
18% of the sweep, in runs 15% wide. The correction is an affine per setting that
lifts those without flattening the natural variation elsewhere.

It has two halves, and they have opposite shapes, which is what decides where
each one lives:

| | shape | so it is |
|---|---|---|
| the centring constant | length-*in*dependent, expensive: an average across every pattern length of what would centre each | baked, 8 KB, `st_centre_table` |
| the gain | length-dependent, cheap: one pattern's own extremes | measured at runtime |

A channel knows only its own length, so it cannot work the constant out; it
scans its own pattern for the gain, one slot per engine tick. That split is what
replaced a 180 KB table of gains and offsets - and it made two things exact that
the table could only approximate:

- **The cycle boundary.** Every length lands on the shared constant at phase 0,
  so pattern length can be switched on the wrap without a step in the signal.
  Interpolating a table put 0.044 of a jump there; computing the anchor at the
  real point puts none.
- **The floor.** `ST_NORM_FLOOR` is now applied at the point being played rather
  than arriving through a neighbourhood rule written for interpolation. Worst
  span anywhere went 0.584 to 0.900 of the 2.0 range.

### What a step costs, and why it is paid once

What a step shows, and what the next one shows, depend on which step the
playhead is in and **not on where inside it** - only the ease between them
varies within a step. The shape is sampled far faster than the steps move: at
0.5 Hz over 64 steps, ninety-one consecutive ticks were computing the same two
numbers from scratch.

So a channel keeps an `StStepCache` and pays per *step* rather than per tick.
Two things fall out of it:

- **Within a step there is nothing to compute but the ease.**
- **Crossing into the next step costs one slot evaluation, not a walk.**
  `st_step_next()` returns exactly `st_step_value(next)`, so what the last step
  eased *toward* is by definition what this one eases *from* - the pair advances
  by computing only its new far end. That is the part that helps a fast LFO,
  where the cache itself never hits.

Only a jump pays the full route: a phase correction, a knob turn, or a rate high
enough to skip a step outright. **The skipping case is not improved at all**, and
was already bounded - `st_step_value()` is O(`ST_JUMP_GRID`) regardless of
pattern length.

### And the slots underneath it

The step cache removed everything *above* the slot evaluations, which left those
as the whole cost - and they are memoisable for the same reason: what a slot
offers depends on the slot and the morph, and the morph on nothing but the two
knobs and the length. `StSlotMemo` remembers `st_slot_offer()` per slot while
the knobs are still.

It is threaded as a parameter, and `NULL` means "compute it". That is what keeps
it a memo rather than a second implementation: one value path, and a caller that
supplies no memo takes the identical route through it. The scan shares the
channel's memo, since it walks the same slots of the same pattern.

### What the two are worth

Measured on the module, seven stepped channels at MOD full (densest ties, motif
fold active), over 64 steps. `engine` avg, and `load` against the 250 µs period:

| rate | steps/tick | neither | step cache | + slot memo |
|---|---|---|---|---|
| 0.5 Hz | 1/91 | 296.5 µs (1.30) | 197.3 µs (0.90) | **184.2 µs (0.84)** |
| 62.5 Hz | ~1 | 282.8 µs (1.27) | 227.5 µs (1.02) | **192.0 µs (0.88)** |
| 129.5 Hz | ~2, skipping | 277.9 µs (1.24) | 275.4 µs (1.23) | **209.0 µs (0.95)** |

**Every column of the last row is the point.** The step cache does nothing for a
playhead that skips - the carry never applies and the cache never hits, so it
was flat at 275 µs - while the slot memo takes 24% off it, because a skipping
playhead still asks for slots it asked for a few ticks ago. Together they hold
`engine_fps` at 4000 under every condition above, with overruns at 3% and
resyncs in single figures. Before them the same patch ran at 2915 with every
tick overrunning.

Backing out the ~135 µs the engine costs before any stepped channel exists, the
per-channel stepped cost at the skipping rate goes 20.0 → 20.0 → **10.5 µs**.

The cache changes when the work happens, never what comes out.
`tests/test_stepped_cache.c` asserts the cached and uncached routes are
bit-identical, and that is not decoration: the cache key has to name every input
`st_morph()` and `st_step_pair()` read, and a lever added to `StMorph` driven by
something outside the key would leave a channel drawing its old pattern - no
crash, no drift in level, just the wrong shape, sounding plausible.

Writing that test found a real one. `st_tie_run()` *breaks* when a tie weight
reaches 1, so `st_step_value()` returns the slot's offer outright - while
`st_step_next()` computed `lerp(from, offer, 1.0f)`, and `from + (offer - from)`
is not `offer` in floating point. At length 3, where `st_tie_fade` lets a weight
reach exactly 1, the two disagreed in the sixth digit. The discrepancy predates
the cache; nothing had ever compared the two routes.

### The scan

`st_norm_scan()` measures one slot per tick and slews the result into place over
20 ms. Three things about it are load-bearing:

- **The engine hands out one measurement per tick**, in turn, across all
  channels. A slot evaluation is nearly as expensive as the shape itself - a
  step value walks a run of ties, each slot a four-tap contour and a motif fold.
  Eight channels each measuring every tick cost 88 µs of a 310 µs tick, and the
  engine answered by dropping every third one. **Measured on the `-O0` debug
  build** (`just flash-usb` flashes that one; `just flash-usb-rel` is the
  optimised firmware), so the absolute figures are 5x pessimistic - the host
  gap between -O0 and -O2 for this shape is 437 ns against 83. The rota bounds
  the cost either way and costs a standing channel nothing, but whether it is
  *needed* at -O2 has not been measured.
- **A standing channel stops measuring.** Re-measuring an unchanged pattern can
  only give the answer it already has, so what the scan costs is a knob moving.
- **A length change restarts the pass** rather than paying for a full
  measurement in one tick, which would be three ticks' worth of budget and
  happens once per encoder detent.

The slew, not a latch to the cycle wrap: a slow LFO would otherwise play a whole
cycle uncorrected after a knob move, and the table it replaced never waited for
a wrap either.

## What a small turn may do

The rule is that a small turn **deforms** the pattern rather than replacing it.
That is measured as **Spearman's rank correlation** between the patterns at *x*
and *x* + 1% of travel: the steps keep their order.

The measure matters as much as the bound. It used to be "no sample moves more
than 0.35", which is the same thing while character comes from redrawing the
pattern - and a different thing for a monotone reshaping, which cannot move a
step out of order however far it moves the values. That is true by construction,
so the old rule was charging full price for the one mechanism that is musically
free, and it was the ceiling on how much a detent could do.

| steps | worst rho, SHP | worst rho, MOD |
|---|---|---|
| 3-5 | -0.50 .. 0.70 | -0.50 .. 0.60 |
| 8-16 | 0.71 .. 0.84 | 0.79 .. 0.90 |
| 24-64 | 0.93 .. 0.96 | 0.94 .. 0.98 |

Held at 0.55 from eight steps up. Short patterns are exempt: three values cannot
change gently, and at three steps one swap inverts the order outright.

A second, looser bound stays on how far a sample may move (0.60), because
keeping the order is not sufficient on its own - a reshaping steep enough to
throw a step across the range would pass the rank test and still be heard as a
jump.

## Things that were tried and are not here

Written down because each cost a day and the numbers looked good on the way in.

- **A smooth wander** (`drift`, octaves of a smoothed triangle in the phase
  domain). Measured beautifully - DC exactly zero, no correction needed, 20 ns a
  call - and sounded like a sine with harmonics, because that is what it is. A
  waveform that closes on itself over one cycle *is* a Fourier series; smooth
  means few harmonics; octave rates are harmonics 1, 2, 4 and 8. Inside one
  phase-locked cycle the only routes to something that reads as random are
  discontinuity or high harmonic order.
- **A separate melodic mode.** Two routings of the same engine, one for notes
  and one for modulation. The notes one was where this arrived from rather than
  a second thing worth keeping - MOD left is gently eased and SHP centre is an
  even distribution, which is where it lived.
- **`ST_SPAN`**, a lever that ducked the peak-to-peak across SHP. Being a pure
  gain it moved no character measurement at all; it only made the shape flatten
  as the knob turned.
- **Cutting `ST_MAX_ORBIT_RATE` to buy small-turn budget.** The worst turn goes
  0.44 at rate 4, 0.51 at 3, 0.39 at 2, 0.37 at 1 - non-monotone, and barely
  moving even where the slot values are nearly frozen. The worst case is not the
  orbit.
- **Snapping values to levels.** A staircase applied to a moving value is a
  discontinuity - 0.23 of a jump at every crossing. The terracing compresses
  each cell toward its centre instead.
- **Caching orbit values across a run.** Slower: the compare and branch to read
  the cache cost more than the triangle they save.

## Measuring a change

`tools/stepped_explore/` is the only thing that can judge one, because the
obvious metrics mislead: the original sweep contained 400 distinct patterns at
32 steps and still read as nothing happening. `from_c.py` reads the shipping C
through `dump_pattern.c` and splits character into **steer** (what the knob
commands) and **jitter** (what merely fluctuates), alongside the level figures.

Read `char-st` rather than `steer` when only the correction changed: three of
its features are level, and flattening those on purpose lowers `steer` while
moving no character. Two features - `skew` and `clump` - exist because the rest
are ordering-based and were blind to the reshapings entirely.

## Known, and open

- **Terracing costs level consistency.** It shrinks a pattern's peak-to-peak by
  an amount depending on where its extremes fall against the cell boundaries -
  variance rather than bias, so the mean span moves only 4% and no global
  compensation reaches it. The span ratio along a SHP sweep is 2.3-2.4 against
  1.7-2.0 without it. The fix, if wanted: let the scan measure the pattern
  *through* the reshapings and set the gain from the output span.
- **Swing on odd lengths** makes step 0 and step L-1 both wide at lengths 3 and
  5 - a lopsided shuffle.
- **Every hardware measurement here was taken on the `-O0` debug build**, which
  is what `just flash-usb` flashes. That is where "the module ticks at 3.3 kHz
  rather than the nominal 4" came from, and it is a property of that build
  rather than of the module. Re-measure with `just flash-usb-rel` before
  treating any of these numbers as the module's own.
