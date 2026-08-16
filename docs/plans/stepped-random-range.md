# Widening SHP and MOD in stepped random

Status: **round three implemented, awaiting a musical verdict.** Rounds one and
two widened what the two knobs reach; round three takes *level* back off them,
which is what the verdict on round two asked for. Decisions taken: MOD stays
monotone density rather than becoming cyclic; the small-turn budget went
0.25 -> 0.35 in round two, on the evidence that shift-mode fine adjust was going
unused; SHP's span lever is gone and the normalisation now holds both the
peak-to-peak and the DC steady (round three).

## Problem

SHP and MOD "evolve too little / too slow" at medium-to-high step divisions.

## Constraints (must survive every variant)

1. SHP is cyclic: value at -1 == value at +1, no seam.
2. MOD cyclic too (open question - today it is monotone density, not cyclic).
3. Neighbouring knob positions sound related: no re-randomising on a small turn.
4. The curve closes on itself over one cycle (loop point inaudible, PLL-locked).
5. Slot 0's value stays length-independent, so switching pattern length on the
   cycle wrap is seamless (`changing_length_at_the_cycle_boundary_is_seamless`).
6. No setting collapses to flat (span > 0.5 of 2.0).
7. O(1) per sample, ~4 kHz x channel count. A handful of extra `tri()` calls is
   affordable; an O(length) loop per sample is not.

## Evaluation function

Three layers, in `tools/sr_explore/` (`srmodel.py` is a verified replica of the
shipping C - max deviation 1.8e-6 against `stepped_random()`).

- **identity** (`metrics.py`, `musical.py`) - RMS distance and quantised-note
  Hamming between rendered cycles. Answers "how many different note sequences".
- **character** (`character.py`) - 10 features per pattern (span, roughness,
  turns, edginess, tie fraction, longest run, motif repetition, centroid,
  low-frequency energy, spread), standardised against a fixed baseline sigma.
- **steer vs jitter** - the character trajectory along the knob, split into its
  low-passed component (**steer**: character the knob commands) and the residual
  (**jitter**: character that merely fluctuates because each position is a fresh
  random draw). `local` is character movement per 1% of knob travel.

Headline figure: **steer high, local low**. `still` is the fraction of travel
where the knob does nothing.

## Baseline

| sweep | steer | jitter | ratio | chars | local | still |
|---|---|---|---|---|---|---|
| SHP L=16 | 3.16 | 0.79 | 4.02 | 64 | 1.24 | 0.00 |
| MOD L=16 | 2.49 | 0.40 | 6.16 | 18 | 0.32 | **0.90** |
| SHP L=32 | 1.70 | 0.68 | 2.48 | 26 | 1.01 | 0.00 |
| MOD L=32 | 1.93 | 0.35 | 5.56 | 16 | 0.29 | **0.80** |

Diagnosis:

- **SHP re-randomises instead of evolving.** A 1% turn moves ~1 sigma of
  character and changes half the notes; over the sweep 325-400 unique sequences
  appear. It is not short of novelty - it is short of *direction*. Every setting
  draws from the same fixed distribution (uniform values, same density, same
  span - the normalisation actively enforces the last one), so random-then-
  random-then-random reads as "nothing is happening". Steer falls from 3.16 to
  1.70 going from 16 to 32 steps: more slots average the character out.
- **MOD is a dead staircase.** 80-90% of its travel does literally nothing;
  the rest jumps discretely as the density threshold crosses a slot gate. Its
  gate set is *nested* (once a step ties it stays tied), so MOD only ever thins
  one pattern - it never makes a different one. Only 9-20 unique sequences on
  the whole knob, and the furthest two are 0.62 apart where unrelated is 1.0.

## Variants

SHP levers (add steered character without raising `local`):

- **A1 rate spread** - widen `SR_MAX_ORBIT_RATE`. Control case; predicted to
  raise churn and not steer.
- **A2 shaper** - SHP drives a cyclic path through a value-shaping curve:
  uniform <-> centre-clustered (calm, small intervals) <-> edge-clustered
  (binary, gate-like). Moves span/edginess/spread.
- **A3 contour** - SHP cyclically blends independent slot values against a
  short FIR over preceding slots: melodic walk <-> jumpy. Anchored at slot 0 so
  constraint 5 survives. Moves roughness/turns/low-frequency energy.
- **A4 motif fold** - SHP cyclically folds the slot index onto a sub-period
  (L/4, L/2): riff-like repetition <-> through-composed. Moves motif.
- **A5 combination** - SHP as a 2-D closed loop over two of the above, so the
  sweep traverses a circle in character space instead of retracing an
  out-and-back.
- **A6 staggered morph** - slot values step through their orbit on a per-slot
  staggered schedule (the module's own stepped-random idea applied in the morph
  domain) instead of all sliding at once. Lowers `local` for the same novelty.

MOD levers (kill the dead travel, reach a genuinely different pattern):

- **B1 gate phase** - MOD rotates a per-slot gate phase so the tie set
  re-arranges continuously instead of nesting.
- **B2 cyclic density** - density as a cyclic function of MOD.
- **B3 swing** - MOD cyclically skews step widths (sum stays 1, loop still
  closes).
- **B4 second morph axis** - MOD moves slot values along a second orbit set, so
  the (SHP, MOD) plane becomes a torus.
- **B5 combination** - MOD as a closed loop over density + one other lever.

Then **C**: the best SHP lever combined with the best MOD lever, re-measured
against every invariant above and against the existing test suite.

## Results

All at MOD 0 / SHP 0, `steer` in baseline sigma. Harness lives in
`tools/sr_explore/`.

### SHP levers (L=32)

| variant | steer | jitter | ratio | chars | local |
|---|---|---|---|---|---|
| baseline | 1.54 | 0.68 | 2.26 | 26 | 1.01 |
| A1 rate spread (max 10) | 1.35 | 1.08 | 1.25 | 29 | 1.48 |
| A2 shaper | 2.84 | 0.78 | 3.66 | 37 | 1.18 |
| A3 contour | 2.88 | 0.66 | 4.35 | 44 | 1.00 |
| A4 motif fold | 2.75 | 0.79 | 3.46 | 50 | 1.17 |
| A6 staggered morph | 1.90 | 0.95 | 1.99 | 35 | 1.33 |
| **A5 shaper+contour** | **3.60** | 0.66 | 5.41 | 47 | 1.00 |

A1 is the control and it behaves as predicted: widening the orbit rates buys
churn, not direction - jitter up 59%, steer *down*. More of the same does not
help. A6 is the same story in a subtler form: staggering the slot transitions
reads as jitter to the low-pass because each transition is a discrete event.

A2 and A3 are the two real levers and they are close to independent, which is
why combining them on one closed loop (A5, 90 degrees apart, so SHP traverses a
circle in character space instead of retracing an out-and-back) beats either.

### MOD levers (L=32)

| variant | steer | jitter | chars | local | still | worst 1% lurch |
|---|---|---|---|---|---|---|
| baseline | 1.75 | 0.35 | 16 | 0.29 | **0.80** | **1.45** |
| B1 gate phase (spin 2) | 2.36 | 0.85 | 32 | 1.04 | 0.21 | - |
| B2 cyclic density | 2.21 | 0.44 | 12 | 0.43 | 0.65 | - |
| B3 swing | 2.23 | 0.36 | 30 | 0.48 | 0.03 | - |
| B4 second morph axis | 2.69 | 1.01 | 50 | 1.40 | 0.00 | - |
| B7 soft ties | 1.77 | 0.24 | 15 | 0.26 | 0.08 | 0.18 |
| **B8 soft ties + gate phase + swing** | **2.91** | 0.40 | 33 | 0.58 | **0.00** | **0.11** |

The measurement that mattered most was not in the original plan. MOD does not
only sit still for 80% of its travel - when it does move it **lurches by 1.45 of
a 2.0 range in 1% of knob turn**, because a tie is a binary decision and the
density threshold crossing a slot's gate switches that step over in one knob
position. Dead, dead, dead, LURCH. The invariant "a small turn deforms the
pattern rather than re-randomising it", which the C suite asserts for SHP, fails
outright for MOD on the shipping algorithm.

Crossfading the tie instead of switching it (B7) fixes the lurch and the dead
travel at a stroke, and *improves* every other number - jitter down, minimum
span up from 0.816 to 0.864. It is the single highest-value change here and is
nearly free.

### Proposal: C5

`A5` on SHP + `B8` on MOD, with three corrections found by measurement (each
documented at the variant in `variants.py`): the contour FIR must not boost its
output, the motif fold must be zero at MOD centre, and the shaper must be a
blend rather than a power law - the power law had to be faded out on short
patterns, and that fade made slot 0 length-dependent, breaking seamless length
switching. The blend never collapses the middle, needs no fade, and replaces a
`powf()` with a multiply and a `sqrtf()`.

Tuned to `bend 0.6, contour 1.0, taps 4, tie width 0.25, spin 0.5, swing 0.4,
motif 0.7` - the binding constraint is the existing test's 0.25 limit on how far
a 1% SHP turn may move any sample.

| sweep | steer | | jitter | | chars | | still | |
|---|---|---|---|---|---|---|---|---|
| | base | C5 | base | C5 | base | C5 | base | C5 |
| SHP L=16 | 2.87 | 3.00 | 0.79 | 0.87 | 64 | 62 | 0.00 | 0.00 |
| SHP L=24 | 1.95 | **2.56** | 0.73 | 0.73 | 42 | 49 | 0.00 | 0.00 |
| SHP L=32 | 1.54 | **2.42** | 0.68 | 0.58 | 26 | 41 | 0.00 | 0.00 |
| SHP L=48 | 1.43 | **2.17** | 0.57 | 0.53 | 27 | 34 | 0.00 | 0.00 |
| MOD L=16 | 2.26 | 2.91 | 0.40 | 0.36 | 18 | 32 | 0.90 | **0.00** |
| MOD L=24 | 1.97 | 1.89 | 0.35 | 0.28 | 18 | 25 | 0.85 | **0.00** |
| MOD L=32 | 1.75 | 2.11 | 0.35 | 0.27 | 16 | 25 | 0.80 | **0.00** |
| MOD L=48 | 1.55 | 2.11 | 0.28 | 0.26 | 12 | 25 | 0.72 | **0.00** |

The gain is concentrated exactly where the complaint was: at 32 steps SHP steers
1.57x further with *less* jitter, and MOD stops being a dead staircase. MOD's
identity numbers move even further - unique sequences 20 -> 118, and the two
furthest-apart MOD settings go from 0.62 to 0.91 where 1.0 is unrelated. MOD
could previously only thin one pattern; now it reaches different ones.

All ten invariants pass, including the two the shipping algorithm fails or
never had to meet.

## As implemented

`bend 0.6, contour 1.0, taps 4, tie width 0.25, spin 0.5, swing 0.4, motif 0.7`.

- `Core/Inc/Lib/stepped_random_pattern.h` (new) - the value path, shared with
  the table generator so the correction cannot be aimed at a pattern the module
  no longer plays. The generator used to carry its own copy.
- `Core/Src/Lib/stepped_random.c` - step lookup under swing, easing, the
  correction.
- `tools/gen_sr_table.c` - new `sr_slot_gate2` table; the normalisation's second
  axis becomes MOD itself rather than the hold probability, which no longer
  determines which steps tie now that MOD rotates the gates. Gain ceiling 10 ->
  20: the shaper gives patterns new ways to compress, and 10 left 41 settings
  out of 55k below the 0.5 span floor. At 20 there are none, and the worst span
  anywhere is 0.810 against the old algorithm's 0.661.

Two changes were forced by measurement rather than chosen:

- The motif fold is applied to slot values *before* the tie chain, not to shown
  values after it. Folding afterwards needs three chains per sample instead of
  one and measured no better.
- The contour FIR must not scale its output. A gain there made slot 0 read 1.6x
  its raw value, and once the contour depth became length-dependent so did slot
  0 - which is what seamless length switching rests on.

### Verification

- All 30 test binaries pass; `tests/test_stepped_random.c` gains six cases
  covering the new properties, and `sr_step_at` gets a unit test because that is
  where the one real bug was: the odd-length branch fired for even lengths at
  phase 1.0, putting a jump at the loop point.
- `tools/sr_explore/verify_c.py` checks the shipping C against `final.py`, the
  spec it was written from: max deviation 3e-6 over 14784 samples.
- Flash 185784 -> 189232 bytes (36% of 512 KB). The normalisation table is the
  same size as before - 8 MOD bins x 128 morph bins was measured to be enough.

### Cost

4.2x the old per-sample cost (18.5 -> 78.5 ns on the dev machine; the target
number wants a module, and `tick.avg_us` reports it). Three optimisations got
it there from 5.7x: the contour's first tap is the slot's own value and was
being computed twice; the motif fold is skipped when MOD is near centre; and the
tie chain starts at the nearest step that takes its own value rather than always
at the pinned slot, which at low density is usually the step itself.

Caching orbit values across a run was tried and is *slower* - the branch to read
the cache costs more than `sr_tri()`. Noted in the header so it is not tried
again.

## Round two: layering

Feedback after playing the first version: shift-mode fine adjust was rarely
needed on either knob, which says the small-turn budget had headroom; MOD was
much better but still wanted more; and the module reported no change in engine
or DAC refresh rate, so the cost was affordable.

The structural idea was to stop giving each lever one excursion across its knob
and instead run them at different integer rates, superimposed - so a given
contour depth recurs paired with a different shaper and span each time.

### What was added

- **Rates.** Contour at 3 across SHP, the shaper at 1, span at 3, MOD's shaper
  layer at 3. Integer, so both knobs still wrap.
- **Span** as a lever: how emphatic the pattern is. Free at runtime - it folds
  into the gain the normalisation table already carries - and floored by what
  the correction actually achieved at each point rather than by a globally
  conservative depth.
- **`SR_MORPH_MOD`**: MOD gets a share of the slot orbit. This is what finally
  moved MOD, and it was the one thing rate changes alone could not buy - +6%
  from rates, +27% once MOD could reach patterns of its own.

### What it cost, and two things that broke

Giving MOD a share of the orbit makes the pattern vary quickly along MOD, and
the normalisation table's 8-bin MOD axis became far too coarse to interpolate
across - 1.3% of the parameter space fell under the flat-output floor. Two
fixes, both needed:

- The table's fine axis is now the **orbit** rather than SHP's morph. Every
  SHP-driven lever is a function of the orbit, and the orbit enters the values
  only through `tri()` at integer rates, so the whole value path is periodic in
  it and one 128-bin axis still covers it.
- Each bin's gain is fitted to the **worst point in its neighbourhood**, not to
  its centre, so interpolating between bins cannot fall short.

Even so the MOD axis needed doubling to 16 bins - 8 still failed - which is the
88 KB of flash. `SR_NORM_MAX_GAIN` went 20 to 40; below that the shortest
patterns could not be lifted clear of the floor.

### Measured, on the module's own output

Against the original algorithm, at MOD 0 / SHP 0:

| sweep | steer before | steer after | jitter before | after | chars before | after |
|---|---|---|---|---|---|---|
| SHP L=16 | 2.87 | 3.81 | 0.79 | 0.89 | 64 | 78 |
| SHP L=24 | 1.95 | 3.65 | 0.73 | 0.83 | 42 | 79 |
| SHP L=32 | 1.54 | 3.50 | 0.68 | 0.74 | 26 | 75 |
| SHP L=48 | 1.43 | 3.32 | 0.57 | 0.66 | 27 | 73 |
| MOD L=16 | 2.05 | 4.59 | 0.39 | 0.58 | 18 | 52 |
| MOD L=24 | 1.72 | 4.01 | 0.33 | 0.47 | 17 | 45 |
| MOD L=32 | 1.49 | 3.85 | 0.33 | 0.40 | 14 | 46 |
| MOD L=48 | 1.29 | 3.45 | 0.26 | 0.36 | 11 | 40 |

The fall-off with step count is gone - that was the original complaint, and it
was the law of large numbers washing the character out as slots were added.

Small-turn cost, swept finely at every length (the old test swept one length at
0.05 and could not see its own limit being exceeded):

| | SHP L>=8 | SHP L=3,4 | MOD L>=8 | MOD L=3,4 |
|---|---|---|---|---|
| original | 0.08-0.22 | 0.88-0.90 | 0.30-1.83 | 0.00 (dead) |
| now | 0.29 | 0.67-1.02 | 0.11-0.13 | 0.30-0.46 |

Flash 189 KB -> 280 KB of 512 KB. Cost 17.7 -> 88 ns/call on the dev machine
(5.0x, against 4.2x for the previous version) - wants re-checking on hardware,
though the module reported no change at 4.2x.

## Round three: level

Feedback after playing round two: the range is there, but the two knobs move the
*level* as well as the character - a given shape flattens and shifts as MOD
turns, and scrolling SHP is not "AC coupled". Amplitude and offset already have
their own controls; SHP and MOD should be spending their travel on rhythm, bend
and contour.

Measured on the shipping build, over the whole (SHP, MOD) plane, the complaint
is exactly right:

| | before | after |
|---|---|---|
| peak-to-peak across the plane | 0.58 .. 1.99 | 0.78 .. 1.73 (L>=5) |
| peak-to-peak along one sweep | 2.3x | 1.45x |
| DC walk along one sweep | 0.76 (up to 1.5) | 0.39 (up to 1.03) |
| worst DC anywhere | 0.78 | 0.64 |
| DC RMS over the plane | 0.25 | 0.15 |
| **character steer** | **2.72** | **2.72** |

Character does not move at all, and that is not luck: every character feature in
`character.py` is invariant under an affine, and both causes here were affine.

### The two causes

- **`SR_SPAN`**, round two's fourth SHP lever, ducked the peak-to-peak by up to
  0.7 three times across the sweep. Being a pure gain it moved no character
  measurement - it bought nothing but the inconsistency. Removed.
- **The correction was anchored on slot 0 and expanded about it**, so the whole
  cycle's DC followed slot 0's own value as the orbit turned. With a gain above
  1 that is an amplifier for a value that sweeps the full range: hence a DC that
  walked 1.5 of 2.0 along one sweep of SHP.

### What replaces them

`out = c + (v - anchor) * g`, as before, with both halves changed:

- **`c` centres the pattern** instead of pulling the anchor toward zero. The
  pull existed only to open up headroom for expansion; centring does that as a
  side effect of doing something musically useful, so it is gone.
- **`g` normalises two-sidedly**: `(target/span)^SR_NORM_EXP`, clamped, so a
  naturally wide pattern is brought down as well as a collapsed one lifted. At
  `SR_NORM_EXP` 0.7 a natural 2.9x range comes out as 1.37x - the ordering
  between calm and emphatic settings survives, its size does not. Target 1.5,
  min gain 0.4.
- The neighbourhood rule that keeps the interpolation between MOD bins off the
  flat floor now carries only the **floor** across the neighbourhood, not the
  full target. Carrying the target gave every bin the largest gain any neighbour
  wanted, which is a bias upward everywhere.

**One constant, not one per length.** The corrected value at the cycle boundary
is slot 0's, and every length has to agree on it or switching length on the wrap
puts a step in the signal. So `c` is an average across the lengths of what would
centre each - the minimax was tried and measured worse (0.54 of residual DC
swing against 0.41), because the two shortest patterns drag it away from where
the other nine sit. Exact per-length centring is unreachable for the same
reason: `A_L (v0 - mid_L)` would have to be length-independent, which needs
slot 0 to sit at the same relative place in every length's range.

Pinning slot 0 to the centre to buy exactly that was tried at half and full
depth. It measures **worse** (DC swing 0.39 -> 0.49) as well as costing
character, because the anchor stops tracking the pattern it is anchoring.

### Cost

None at runtime: the table is the same size, the lookup is the same two values,
the value path is untouched (`verify_c.py` still agrees with `final.py` to
1.4e-5). Flash unchanged. The generator gained a third pass and takes ~2s
longer.

The small-turn budget moved: MOD went 0.11-0.13 to 0.18-0.22 of the 0.35 limit,
because the centring constant varies along MOD. SHP is unchanged at 0.29-0.30.
The worst span anywhere is 0.584 against the floor of 0.5.

The length-switch seam is slightly worse: 0.044 of 2.0 at MOD 0 (was 0.046) but
0.122 once MOD is off centre (was 0.095). The correction cancels slot 0 exactly
only at a bin centre - between bins the gain is interpolated while slot 0's
value is computed exactly, and the residual is proportional to how far the
lengths' gains differ, which two-sided normalisation widens.

### Verification

- All 31 test binaries pass. `tests/test_stepped_random.c` gains
  `neither_knob_walks_the_pattern_off_centre` and
  `neither_knob_changes_how_loud_the_pattern_is`, which are the two properties
  this round is about, and which the shipping build before it fails.
- `tools/sr_explore/dump_pattern.c` + `from_c.py` are new: the character numbers
  computed on the C's own output plus the level ones. A change to the correction
  cannot be judged by `srmodel.py`, which does not carry the generated table.

## Unresolved questions

1. **Is `SR_NORM_EXP` 0.7 the right amount of "not a 100% rule"?** 1.0 pins
   every setting to the same peak-to-peak (span ratio 1.12 and it does sound
   levelled); 0.0 is where round two was. If settings still feel too alike in
   weight, wind it down toward 0.5; if the level still moves too much, up.
2. **The remaining DC outlier is a twelve-step pattern at high MOD** (0.64 of
   2.0, 0.9% of the plane above 0.4). Most steps tie to one value there, so
   there is genuinely no centre to find - but if it is audible as a lopsided
   patch of the knob, the fix is to bound the tie chain's contribution rather
   than to correct harder.
3. **Everything is a little quieter at its loudest** - peak-to-peak tops out at
   1.73 against 1.99 - and a little louder at its quietest. Raising
   `SR_NORM_TARGET` above 1.5 trades headroom for it.
4. **Is 0.35 the right small-turn budget now?** It was 0.25, and shift-mode
   fine adjust going unused was the signal to spend it. SHP measures 0.29 from
   eight steps up. If it now feels twitchy, `SR_BEND` and `SR_CONTOUR_RATE` are
   the two to wind back.
5. **The shortest patterns are twitchier than they were** - 1.02 against 0.90
   at three steps, a consequence of the higher gain ceiling that was needed to
   clear the flat spot. Three values cannot change gently, but it is worth an
   ear.
6. **88 KB of flash for MOD resolution.** Halving it puts 0.014% of the space
   back under the flat floor - re-checked under round three, where 8 bins still
   fails the seam test outright. Cheap to trade back if flash is ever wanted.
7. **Swing on odd lengths** still makes step 0 and step L-1 both wide at
   lengths 3 and 5 - a lopsided shuffle. Untouched this round.
