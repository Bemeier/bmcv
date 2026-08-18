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
0.5 Hz over 64 steps, tens of consecutive ticks were computing the same two
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
fold active), over 64 steps. `engine` avg, and `load` against the 250 µs period -
**taken while the tick was still 250 µs**, so the loads are not today's. The
engine cost per tick is what is being compared and that does not move with the
period; see `hw_setup.h` for what the tick is now and why.

| rate | steps/tick | neither | step cache | + slot memo |
|---|---|---|---|---|
| 0.5 Hz | 1/91 | 296.5 µs (1.30) | 197.3 µs (0.90) | **184.2 µs (0.84)** |
| 62.5 Hz | ~1 | 282.8 µs (1.27) | 227.5 µs (1.02) | **192.0 µs (0.88)** |
| 129.5 Hz | ~2, skipping | 277.9 µs (1.24) | 275.4 µs (1.23) | **209.0 µs (0.95)** |

**Every column of the last row is the point.** The step cache does nothing for a
playhead that skips - the carry never applies and the cache never hits, so it
was flat at 275 µs - while the slot memo takes 24% off it, because a skipping
playhead still asks for slots it asked for a few ticks ago. Together they held
`engine_fps` at its nominal 4000 under every condition above, with overruns at
3% and resyncs in single figures. Before them the same patch ran at 2915 with
every tick overrunning.

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

- **Terracing moved the level, and the correction now spans it.** It shrinks a
  pattern's peak-to-peak by an amount depending on where its extremes fall
  against the cell boundaries - variance rather than bias, so the mean span
  moves only 4% and no global compensation reaches it. `st_bias_map` driven
  positive does the opposite and *widens* a span.

  So the correction is two affines with the reshapings between them. The
  pre-stage centres the pattern and scales it toward `ST_NORM_TARGET`, as it
  always did. The post-stage - `st_norm_levelled()` - measures what the
  reshapings actually left and scales it back to what the pre-stage asked for.
  Measured across the whole (SHP, MOD, length) grid, on the path a channel
  actually plays:

  | | min span | max span | ratio |
  |---|---|---|---|
  | before | 0.5640 | 1.8750 | 3.325 |
  | after | 0.5640 | **1.6217** | **2.876** |

  **Only the loud end moves, and that is the rails.** Where `st_gain_toward`'s
  clamp is already what sets the gain - the pattern is as wide as +/-1 allows -
  there is no room to lift the quiet settings, so the minimum does not budge.
  What the post-stage reaches is the settings that were coming out *too wide*.

  Two things it must not disturb, and does not. It is anchored at the reshaped
  value at phase 0, which every length shares because the centring constant is
  length-independent - so the loop still closes and pattern length still
  switches seamlessly on the wrap. And being applied *after* the maps, it scales
  the finished waveform uniformly: the plateaus stay plateaus.

  **Applying it before the maps instead does not work, and was measured.** A
  gain change ahead of the terracing moves values against its cell boundaries,
  so it changes which plateau each one lands on - the character, not the level.
  The first attempt did that, iterating on the pre-stage gain, and it also broke
  `ST_NORM_FLOOR` at two settings by removing a widening the floor had been
  carrying by luck.

  A caution for anyone measuring this: `cycle_level()` in the test suite
  evaluates through a *bare* drive - bias 0, terrace 0 - so the reshapings are
  the identity there and it cannot see any of this. An early comparison using it
  produced a confident and completely wrong conclusion. Measure through
  `st_drive(shape, mod)`, which is what a channel plays.

  It costs 3.6% of the engine at the worst case measured on the module - three
  map evaluations and a divide per completed scan pass, and one multiply-add per
  sample - taking `load` from 0.44 to 0.45.

- **Peak-to-peak normalisation flattened crest factor, and `ST_NORM_ROBUST`
  is what stopped it.** The correction measured the span from the single
  highest and single lowest slot. A pattern that sits low with one tall spike
  has the same peak-to-peak as one spread evenly across the range, so it earned
  the same gain - its spike came out no higher than the other's ordinary
  maximum and its bulk far below. A taller spike read as a wider span and
  therefore earned a *smaller* gain. Crest factor is the one thing peak-to-peak
  normalisation cannot let through.

  Reported from the module first, and it was audible before it was measured:
  at negative SHP there were few high values, and the ones there were did not
  peak enough - "now it's just largely quiet".

  The span is now measured between the *second* highest and second lowest,
  blended with the true extremes by `ST_NORM_ROBUST` (0.5). **The rail limit
  still comes from the true extremes**, which is what makes it safe: raising
  the gain lifts a spike toward +/-1 and stops it exactly there rather than
  clipping it.

  Measured across the (SHP, MOD, length) grid, through the drive a channel
  actually plays:

  | | before | after |
  |---|---|---|
  | median peak, SHP < -0.3 | +0.467 | **+0.555** |
  | settings with no peak at all (<0.40) | 36.3% | **25.1%** |
  | RMS ratio, loudest to quietest | 5.249 | **4.067** |
  | crest factor, mean | 1.331 | **1.383** |
  | neighbour rho, mean | 0.9877 | **0.9877** |
  | worst sample move, 1% turn | 0.502 | 0.515 (bound 0.60) |

  **Peak-to-peak spread goes up while RMS spread goes down.** The ear hears
  RMS, so perceived loudness got *more* consistent - the extra span is peaks
  poking out of it, which is the point. The metric that looks worse is the one
  that cannot tell a spike from a loud patch.

  **The character is untouched, and that was the thing to protect.** Spearman
  rho between a setting and the one 1% of travel away is identical to four
  decimal places, so neighbouring knob positions are exactly as related as they
  were; the worst single-sample move across that turn is slightly *larger*, not
  smaller, so there is no loss of variety between neighbours either.

  **It fades in with length**, via `st_dof_fade` - the same fade the contour
  blend uses and for the same reason. At three steps `hi2` and `lo2` are both
  the middle value, the robust span is zero, the gain runs to the rail, and a
  1% turn of SHP moved the output 1.66 of a 2.0 range against a 1.10 limit.
  With the fade it is 0.90.

  One trap worth recording: the rolling scan must seed its order statistics
  *empty* and push slot 0 like any other, exactly as `st_extent_of` does.
  Seeding them from slot 0 looks equivalent and is not - `lo2` starts equal to
  `lo` and can never be improved upward - and the rolling measurement then
  disagreed with the full one in the fourth decimal.

- **Swing on odd lengths** makes step 0 and step L-1 both wide at lengths 3 and
  5 - a lopsided shuffle.
- **Every hardware measurement here was taken on the `-O0` debug build**, which
  is what `just flash-usb` flashes. That is where "the module ticks at 3.3 kHz
  rather than the nominal 4" came from, and it is a property of that build
  rather than of the module. Re-measure with `just flash-usb-rel` before
  treating any of these numbers as the module's own.
