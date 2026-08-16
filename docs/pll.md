# The sync loop

How a channel's phase is locked to the incoming clock, what it measures, and
the record of the rework that got it here. `tests/test_pll.c` prints the table
below on every run; re-run it after touching the loop and diff.

The work this describes is done. It is kept because the numbers are the only
way to tell whether a future change to the loop is an improvement, and because
two of the three defects found were things the code looked correct about.

## What the loop is, as built

`channel_compute` runs a proportional controller per channel:

```
target  = ((beat_counter % gcd) + beat_phase) * freq_multiplier
diff    = phase_error(target, shared_phase, phase_length)     // wrapped, cycles
correction += (diff - correction) * k_sync                    // k_sync = 0.075/tick
phase  += dt * (freq + correction)
```

`gcd = find_denominator(freq_multiplier, 8, 0.025)` is the alignment period in
beats — the number of beats after which the channel returns to the same phase.
`phase_length = gcd * freq_multiplier` cycles is the super-period the error
wraps in.

Two things the measurements settled that were not obvious from reading it:

- **`k_sync` is not the loop's time constant.** It is an EMA coefficient per
  tick, so at 250us its time constant is ~3.3ms — three hundred times faster
  than the loop itself. `correction` therefore tracks `diff` essentially
  instantly, and the loop reduces to `d(phase)/dt = freq + 1.0 * error`: a pure
  P controller with an implicit gain of 1/second.
- **So the response is first-order with tau = 1s**, and is nearly independent of
  the control rate. Measured: a half-cycle phase step settles to 1% of a beat in
  3.90s at a 250us tick and 3.87s at 1000us. `ln(0.5/0.01) = 3.9`, which is the
  whole story.

## Step 1: the harness

- `tests/pll_metrics.{c,h}` — a clock generator that drives the fixture at a
  tempo (with optional deterministic jitter), a trace, and the metrics:
  settling time, peak error, ringing (sign changes past a deadband), peak
  frequency pull, frequency slew, steady-state RMS, and alignment-period flips.
  Errors are in **beats**, so they compare across ratios.
- `tests/test_pll.c` — twelve scenarios. Bounds are deliberately loose: the
  point is the printed table, which is the before-picture to diff against.
- `ChannelEffective.phase_error` publishes what the loop is minimising, so the
  tests measure the loop's own quantity rather than a re-derivation that could
  drift from it.

### Baseline (2026-08-04, k_sync 0.075, tau ~1s)

```
scenario                             settle     peak  cross     fdev     fslew  rms_tail   gcd
                                          s    beats      n   x rate  x rate/s     beats flips
lock, x1 @ 120bpm                     0.000   0.0027      0    0.001       0.0   0.00002     0
scene A->B, x1 -> x2 (snap)           3.212   0.2500      0    0.123      34.7   0.00002     0
scene sweep, during the 1s move       never   3.0789     12    1.398     461.9   1.46698    44
scene sweep, settling after           3.164   0.2391      0    0.120       0.1   0.00002     0
ratio sweep x1 -> x4 over 3s          never   3.5626     20    1.721     543.1   1.21583   108
phase step, half a cycle              3.902   0.5000      0    0.246      69.4   0.00002     0
tempo step, 120 -> 140bpm             3.016   0.0728      0    0.032       5.1   0.00031     0
x0.75, 240s alignment                 0.000   0.0003      0    0.000       0.0   0.00019     0
x1, 5% clock jitter                  29.988   0.0272     38    0.014       3.2   0.01048     0
clock lost 6s, then back              0.000   0.0007      0    0.000       0.1   0.00002     0
phase step @ 250us tick               3.901   0.4999      0    0.246      48.9   0.00002     0
phase step @ 1000us tick              3.868   0.4998      0    0.239      12.2   0.00000     0
```

### What it says

**The steady state is excellent.** Long-run alignment at x0.75 (gcd 4, so the
channel only meets the beat every fourth one) holds 0.0002 beats RMS over four
minutes with zero drift. Clock loss and return, tempo steps and jitter are all
absorbed cleanly. Nothing rings: every case is 0 or 1 significant crossings.
The extrapolation-to-beat-multiple approach works — this is the part to keep.

**It is slow, and deliberately over-damped.** ~3.9s to close a half-cycle
error. That is a defensible choice for a module where a lurching LFO is worse
than a late one, and it is why nothing overshoots.

**The one real defect is a moving ratio.** During a crossfade between scenes at
different rates, `find_denominator` changes its answer **44 times in a one-second
fader move** (108 times over three seconds for x1 -> x4). Every change alters
`beat_counter % gcd` by an arbitrary number of beats, so the *target teleports*
and the loop chases a step it did nothing to cause. Consequences, all measured:

- phase error excursions of **3.1 beats** where the fader move itself justifies
  a fraction of one;
- the correction pulling the oscillator to **1.4x its nominal rate** (1.7x for
  the wider sweep) — an audible speed lurch, which is exactly the artifact the
  loop is otherwise tuned to avoid;
- frequency slew of ~460 x rate/s.

Also: a large `gcd` admits a large error, because the error wraps at half the
super-period. At gcd 7 the loop can believe it is 3.5 beats out and correct at
full gain. The excursions above are that mechanism, triggered by the flipping.

## Step 2: done (1-4)

1. **The alignment period is latched.** `EngineState.channels_gcd[]`, taken once
   and re-taken only when the super-period wraps — the "do not change the grid
   under the playhead" rule `st_length_idx` already follows.
2. **The origin is latched with it** (`channels_beat_origin[]`), and the target
   is measured from it: `((beat_counter - origin) % gcd + beat_phase) * ratio`.
   This is what makes a change of period seamless. At the wrap the channel is at
   phase 0 of the period it is leaving, and taking the origin there defines the
   new period to start in the same place — so the target does not move at all.
   Re-basing anywhere else could not work: the origins differ by a whole number
   of *beats*, which is not a whole number of *cycles*, so it would step the
   output waveform.
3. **The correction is clamped** to `PLL_MAX_PULL` (0.15) of the channel's
   nominal rate. Past that the error closes at a constant rate instead of an
   exponential one — slower for a large error, and inaudible.
4. **`PLL_TAU_S`, `PLL_MAX_PULL`, `PLL_SMOOTH_S`** in `channel.h`, all in
   seconds or in fractions of the rate. The smoothing coefficient is now
   `dt / PLL_SMOOTH_S` rather than a per-tick constant.
5. **Not done, and argued against.** An integral term buys nothing measurable
   here: the steady-state residual under a tempo error is 0.0003 beats, and a
   second pole is the thing that would introduce the overshoot this loop is
   valued for not having.

### After (2026-08-04, PLL_TAU_S 1.0, PLL_MAX_PULL 0.15)

```
scenario                             settle     peak  cross     fdev     fslew  rms_tail   gcd
                                          s    beats      n   x rate  x rate/s     beats flips
lock, x1 @ 120bpm                     0.000   0.0027      0    0.001       0.0   0.00002     0
scene A->B, x1 -> x2 (snap)           3.212   0.2500      0    0.123      38.2   0.00002     0
scene sweep, during the 1s move       never   0.2858      0    0.142       0.7   0.23821     1
scene sweep, settling after           3.341   0.2852      0    0.142       0.2   0.00002     1
ratio sweep x1 -> x4 over 3s          never   0.9004      0    0.150       1.7   0.84237     2
phase step, half a cycle              4.060   0.5000      0    0.150      45.9   0.00002     0
tempo step, 120 -> 140bpm             3.017   0.0728      0    0.032       5.6   0.00031     0
x0.75, 240s alignment                 0.000   0.0003      0    0.000       0.0   0.00019     0
x1, 5% clock jitter                  29.988   0.0272     38    0.014       3.5   0.01048     0
clock lost 6s, then back              0.000   0.0007      0    0.000       0.1   0.00002     0
phase step @ 250us tick               4.059   0.4999      0    0.150      31.1   0.00002     0
phase step @ 1000us tick              4.056   0.4995      0    0.150       5.9   0.00000     0
```

| | before | after |
|---|---|---|
| crossfade, frequency pull | 1.398x | **0.142x** |
| crossfade, frequency slew | 461.9 | **0.7** |
| crossfade, peak phase error | 3.08 beats | **0.29 beats** |
| gcd flips, 1s crossfade | 44 | **1** |
| gcd flips, 3s x1→x4 sweep | 108 | **2** |
| phase step, settling | 3.90s | 4.06s |
| long-run alignment, jitter, tempo step, clock loss | — | unchanged |

The crossfade artifact is gone: what was a 1.4x speed lurch is now a bounded
0.14x lean, and the frequency slew during a fader move drops by a factor of 660.
The cost is 4% on settling time, from the clamp, exactly as predicted.
Everything that was already good is untouched, to the digit.

## Step 3: two regressions the hardware found

The first pass at (1) latched `gcd` but left the accumulator wrapping at the
super-period. Caught on hardware as an audible click through a slider move, and
then reproduced in the harness as a new metric, `jump` — the step in output
phase that the oscillator's own rate does not account for.

**The accumulator may only ever wrap at one cycle.** The waveform is a function
of phase modulo one, so that wrap is invisible; the sample either side of it is
the same sample. It used to wrap at `gcd * ratio` cycles, which is only a whole
number of cycles when the ratio really is the rational multiple
`find_denominator` claims. That held while gcd was recomputed every tick — to
within its 0.025 tolerance, so the step existed but was under 2.5% of a cycle.
Latching gcd while the ratio kept moving removed the guarantee entirely:
measured at **0.34 of a cycle** through a one-second crossfade.

The alignment period belongs in the error term, not in the accumulator. So:

- `channels_shared_phase[]` is the phase within one cycle, wrapping at one.
- `channels_cycle[]` counts which cycle of the super-period it is on, and
  `channels_period_cycles[]` how many cycles that period holds. The loop's
  error is computed against `cycle + phase` over the whole super-period.

The second regression came out of fixing the first. Reducing the error to a
single cycle is appealing — it bounds the error at half a cycle and the waveform
repeats, so whole cycles of difference are inaudible in the steady state. It is
wrong: the loop then locks to whichever of the `period_cycles` equivalent phases
is nearest, so a x2/3 channel still repeats every three beats but can sit a
third of a cycle off the bar. **Which cycle of the pattern lands on the downbeat
is the entire point of aligning to a beat multiple.** Nothing asserted it;
`the_pattern_lands_on_the_same_beat_every_time` does now.

A third change fell out of the same investigation: a new alignment period is
only taken once the ratio has held still for `PLL_RATIO_STABLE_US`. A ratio
crossing a crossfade is a rational multiple every few ticks and irrational
between them, so acquiring on the instant meant taking and discarding eleven
periods in a second, with the correction switching on and off along with them.

### Final (2026-08-04)

```
scenario                             settle     peak  cross     fdev     fslew  rms_tail      jump
                                          s    beats      n   x rate  x rate/s     beats    cycles
lock, x1 @ 120bpm                     0.000   0.0027      0    0.001       0.0   0.00002   0.00000
scene A->B, x1 -> x2 (snap)           3.212   0.2500      0    0.123      38.2   0.00002   0.00001
scene sweep, during the 1s move       never   0.2367      2    0.116      32.9   0.18348   0.00001
scene sweep, settling after           2.937   0.1902      0    0.095       0.1   0.00002   0.00001
ratio sweep x1 -> x4 over 3s          never   0.2361      4    0.116      45.4   0.05841   0.00002
fader worked back and forth           never   1.7751     12    0.154     100.4   0.37568   0.00003
phase step, half a cycle              4.060   0.5000      0    0.150      45.9   0.00002   0.00001
tempo step, 120 -> 140bpm             3.017   0.0728      0    0.032       5.6   0.00031   0.00001
x0.75, 240s alignment                 0.000   0.0000      0    0.000       0.0   0.00001   0.00000
x1, 5% clock jitter                  29.988   0.0272     38    0.014       3.5   0.01048   0.00000
clock lost 6s, then back              0.000   0.0007      0    0.000       0.1   0.00002   0.00000
phase step @ 250us tick               4.059   0.4999      0    0.150      31.1   0.00002   0.00000
phase step @ 1000us tick              4.056   0.4995      0    0.150       5.9   0.00000   0.00000
```

| | original | final |
|---|---|---|
| crossfade, frequency pull | 1.398x | **0.116x** |
| crossfade, frequency slew | 461.9 | **32.9** |
| crossfade, peak phase error | 3.08 beats | **0.24 beats** |
| crossfade, output phase step | 0.025 cycles (by luck) | **0.00001** (by construction) |
| crossfade, settling after | 3.16s | 2.94s |
| x0.75 long-run alignment, RMS | 0.00019 beats | **0.00001 beats** |
| phase step, settling | 3.90s | 4.06s |

### Re-taken at a 2kHz tick (2026-08-16)

`ENGINE_TICK_US` went 250 to 500, and the baseline moves with it. Taking it
again turned up a defect that had nothing to do with the rate.

The rate was decided by the *worst* case rather than the typical one: a moving
crossfader empties the stepped caches every tick, because every scene blend is a
new setting, and that costs 359 us however fast the tick is. At 312 us a fader
sweep took `engine_fps` to 2440; at 500 us the same case holds 2000.

**`Clock_Poll` wrapped `beat_phase` past a beat `beat_counter` had not reached.**
The phase is interpolated between pulses and was wrapped with a
`while (next_phase >= 1.0f) next_phase -= 1.0f`, while only a pulse advances the
counter. `channel.c` reads the two together - the target is
`(beats_in % gcd) + beat_phase` - so for the ticks between a beat falling due
and its pulse arriving, every locked channel was pulled by a whole beat of
target at once. It now holds at the end of the beat instead, which is the honest
reading: the next pulse is the evidence that the beat turned over, so a late
pulse stretches the beat.

It read as a single-tick excursion of 0.25 cycles on every beat boundary at
x0.75 - `max` 0.334 beats with `rms_tail` staying clean, because each one lasted
one sample. **Sweeping the tick period, the whole band from 280 to 345 µs showed
it and 250, 260, 360 and 400 µs did not**, which is why nothing had caught it in
two years of running the suite at 250. The suite now passes at every period from
250 µs to 500 µs.

```
scenario                             settle     peak  cross     fdev     fslew  rms_tail      jump
                                          s    beats      n   x rate  x rate/s     beats    cycles
lock, x1 @ 120bpm                     0.000   0.0026      0    0.001       0.0   0.00002   0.00000
scene A->B, x1 -> x2 (snap)           3.213   0.2500      0    0.123      34.7   0.00000   0.00003
scene sweep, during the 1s move       never   0.2368      2    0.116      32.8   0.18370   0.00002
scene sweep, settling after           2.938   0.1903      0    0.095       0.1   0.00000   0.00001
ratio sweep x1 -> x4 over 3s          never   0.2362      4    0.116      45.4   0.05839   0.00006
fader worked back and forth           never   1.7739     12    0.153     100.5   0.37573   0.00014
phase step, half a cycle              3.894   0.4500      0    0.150      41.7   0.00002   0.00002
tempo step, 120 -> 140bpm             2.992   0.0725      0    0.032       5.6   0.00059   0.00001
x0.75, 240s alignment                 0.000   0.0000      0    0.000       0.0   0.00003   0.00000
x1, 5% clock jitter                  29.986   0.0268     40    0.013       2.9   0.01048   0.00000
clock lost 6s, then back              0.000   0.0009      0    0.000       0.1   0.00002   0.00000
phase step @ 500us tick               4.058   0.4998      0    0.150      18.7   0.00002   0.00000
phase step @ 2000us tick              4.240   0.4973      0    0.150       0.3   0.00206   0.00000
```

What moved, and why:

- **`rms_tail` is unchanged at 0.00002 beats.** The residual is set by how
  finely the loop can place a correction, and at 500 us it is still far below
  anything audible.
- **`x0.75, 240s` peak 0.0000 → 0.0006.** Was 0.3339 before the clock fix.
- **`clock lost 6s, then back` settles in 1.25 s where it used to read 0.000.**
  The old zero was the wrap papering over the re-acquisition; holding the phase
  at the end of the beat makes the recovery visible instead of instant-looking.
- **Gates on the input are not affected by any of this.** They are latched in
  the DAC's own interrupt, which samples each converter channel at ~8 kHz, and
  consumed once per tick - so a gate shorter than a tick is still seen. What the
  longer tick costs is the *timestamp*: an edge is attributed to the tick that
  consumed it, so the clock's pulse times carry up to 500 us of quantisation.
  See "Still to check on hardware".
- **The phase-step case steps 0.45 of a cycle, not 0.50.** Exactly half sits on
  the wrap point of `phase_error`, where +0.5 and -0.5 are the same place and
  which way the loop corrects is decided by the last bit of the arithmetic. The
  pick registers as a zero crossing, and `overshoot_beats` - the largest error
  *after* the first crossing - then reports the undisturbed error as overshoot.
  Which side it picked depended on the tick period. Settling, ringing and the
  tail still cover the exactly-half case.

### Still to check on hardware

`PLL_MAX_PULL` is the knob: lower is smoother and slower to acquire, higher is
closer to the old behaviour. `PLL_RATIO_STABLE_US` decides how soon after a
fader stops the channel re-acquires. Re-run `./build-native/test_pll` after any
change and diff the table above.
