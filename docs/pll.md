# The sync loop

How a channel's phase is locked to the incoming clock, what it measures, and
the record of the rework that got it here. `tests/test_pll.c` prints the table
below on every run; re-run it after touching the loop and diff.

The work this describes is done. It is kept because the numbers are the only
way to tell whether a future change to the loop is an improvement, and because
two of the three defects found were things the code looked correct about.

## What the loop is, as built

`channel_compute` runs a proportional controller per channel. **As it stands
today**, after the four steps below:

```
q, p     = the latched rational: q beats hold p cycles      // see Step 4
target   = frac(((beat_counter % q) + beat_phase) * p / q)  // the clock's grid
diff     = phase_error(target, shared_phase, 1.0)           // cycles
pull     = clamp(diff / PLL_TAU_S, +/- PLL_MAX_PULL * freq)
correction += (pull - correction) * (dt / PLL_SMOOTH_S)
phase   += dt * (freq + correction)                         // wraps at one cycle
```

The rest of this section describes it **as it was first measured**, which is
where the record starts:

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

### Timestamping clock edges in the DAC interrupt: measured, not worth it

A tick consumes a clock edge that was *detected* up to a tick earlier - the
latch in the DAC's DMA interrupt records that an edge happened, not when - so
`Clock_Trigger` is given the tick's timestamp. Stamping the edge where it is
detected would put it on the DAC's 62.5 us chunk instead of the engine's 500 us
tick. That is a change across the seam, so it was measured first.

Driving `ClockState` with a perfectly regular clock, so every bit of the error
is quantisation, and taking the *variation* in `beat_phase` rather than its
offset (which is arbitrary - it depends where the first pulse landed):

| edge stamped at | rms, beats | peak |
|---|---|---|
| the tick, 500 us - today | 0.000551 | 0.001233 |
| the DAC chunk, 63 us - the proposal | 0.000070 | 0.000154 |
| exactly - the ceiling | 0.000001 | 0.000003 |
| a 250 us tick - the old engine rate | 0.000274 | 0.000621 |

So the proposal is real: **8x less phase jitter into the loop**, and it would
more than pay back what the 2 kHz tick cost on the input side.

**And it does not matter, because of what is downstream.** The loop's time
constant is 1 s against a pulse rate of 4-24 Hz, so it attenuates this by about
28x: `rms_tail` in the table above is **0.00002 beats with today's stamping**,
which is 10 us at 120 BPM. One DAC frame at 4032 frames/s is **248 us**. The
residual is twenty-five times smaller than the interval between two output
samples - it cannot be observed at the output even in principle.

Worth revisiting only if `PLL_TAU_S` is ever taken far below 1 s. A loop that
tracked the clock aggressively would pass this jitter through instead of
filtering it, and then where the edge is stamped starts to show.

Nothing is missed, either way: gates are latched in that interrupt at ~8 kHz and
consumed once per tick, so a gate far shorter than a tick is still seen.

## Step 4: the grid was the channel's, and it should have been the module's

Two defects, both from the Step 2/3 rework, both found on hardware and then
reproduced in the simulator. They share a cause: the thing the loop aligns to
was per-channel latched state, and per-channel latched state can go stale or
diverge from the next channel's.

### Two channels at one rate, locked exactly 180 degrees apart

The target was measured from `channels_beat_origin[]`, a beat latched per
channel. A new origin was allowed wherever the channel's *own old* period said
the move was seamless - `(beat_counter - origin) % gcd == 0`. **At gcd 1 that
test is vacuously true**, so a channel crossfaded out to x1 and back to x1/2
re-based on whatever beat the fader happened to settle on. An origin one beat
out at x1/2 is half a cycle. Two identical channels, both locked, both steady,
and exactly 180 degrees apart, for as long as the patch was left alone. A
quarter cycle at x1/4, which is the other one that was heard.

The parity is the tell: sweeping the detour length by whole beats, the gap after
came out 0.0000, 0.5000, 0.0000, 0.5000. Half the return beats landed on the
right grid by luck, which is why it looked intermittent.

There is now no per-channel origin. **The grid is `clock.beat_counter`**, which
every channel reads and none of them can disagree about. `Clock_Reset` zeroes
that counter, so a reset re-establishes the grid for everything at once - which
is what the module was already observed to do, and now it is the mechanism
rather than a coincidence.

The `clock_lock` flow moved with this, and the move is the point: the old golden
had the x1/3 channel starting its three-beat period on the beat where its FRQ
encoder happened to be turned. Replaying the same flow with that gesture 500 ms
later shifted the x2/3 channel from 1.005 to 1.505 of its period before the fix,
and does not move it at all after.

### A rate change that kept its denominator, and the wander that followed

The alignment rational is `q` beats to `p` cycles. `p` was computed only inside
the branch that saw `q` change:

- so it was **never taken on the first acquisition at all** - it sat at the
  initial 1 for the life of any channel whose ratio never moved, and was only
  ever right by luck;
- and it went **stale on every move between two ratios that share a `q`**.
  x1/4 and x3/4 both answer q 4. x1, x2, x3 and x4 all answer q 1.

A stale `p` is a super-period the channel's own cycle counter overruns: the loop
compares a target confined to one cycle against a position roaming over three.
Measured at x3/4 -> x1/4, left alone: **peak error 4.6 beats, steady-state RMS
3.19 beats, and the correction pinned on the `PLL_MAX_PULL` clamp with the sign
crossing back and forth indefinitely.** That is the slow wander heard on the
module. It now reads 0.00006 beats RMS with the correction at zero.

`q` and `p` are one fact and are latched together, whenever either half of them
changes.

### The cycle counter is gone, and it cost nothing

Step 3 argued that reducing the error to a single cycle would let a x2/3 channel
"sit a third of a cycle off the bar", and added `channels_cycle[]` and
`channels_period_cycles[]` to measure the error over the whole super-period.

**The argument does not hold.** `find_denominator` returns the *smallest* q it
can, so `p/q` is in lowest terms, so the q beat positions map to q *distinct*
phases within one cycle - for x2/3 they are 0, 2/3 and 1/3. The fraction alone
says which beat of the period the channel is on; there is nothing to confuse.

The evidence that it was never load-bearing: because of the bug above, `p` was 1
on every channel that had not changed rate, so the error *was* being measured
over one cycle on the common path, and had been for the life of the feature.
`the_pattern_lands_on_the_same_beat_every_time` passed throughout, and passes
now, and is the guard for exactly this.

So the error is taken over one cycle, `channels_cycle[]` and
`channels_beat_origin[]` are gone, and `BmcvInstance` is 80 bytes smaller.

### The target is p/q, not the live ratio

The remaining piece. The target used to be `beat_mode * freq_multiplier`, which
steps at the wrap of `beat_counter % q` by `(q * ratio) mod p` whenever the live
ratio and the latched rational disagree - which is every crossfade, and every
moment a stale rational is in force. Using `p/q` instead, the target has advanced
by exactly `q * p/q = p` cycles at the wrap, so its fraction is unchanged and the
target is **continuous by construction**, however far the ratio has moved away.

That is what makes it safe to drop Step 2's "wait for the super-period to wrap"
gate: there is nothing discontinuous left to protect. What replaces it is far
cheaper - the rational is swapped within `PLL_RETAKE_WINDOW` of the top of a
beat, where the old grid and the new one agree. Swapping mid-beat instead costs
22 x rate/s of frequency slew on a scene snap; swapping at the beat top costs
0.5.

### After (2026-08-18)

```
scenario                             settle     peak  cross     fdev     fslew  rms_tail      jump
                                          s    beats      n   x rate  x rate/s     beats    cycles
lock, x1 @ 120bpm                     0.000   0.0026      0    0.001       0.0   0.00002   0.00000
scene A->B, x1 -> x2 (snap)           3.340   0.2217      0    0.110       0.5   0.00000   0.00001
scene sweep, during the 1s move       never   0.2777      1    0.138      92.3   0.20495   0.00008
scene sweep, settling after           3.181   0.1891      1    0.094       0.5   0.00000   0.00001
ratio sweep x1 -> x4 over 3s          never   0.2777     17    0.138      92.3   0.08261   0.00009
fader worked back and forth           never   0.6663     51    0.153     101.1   0.18585   0.00009
phase step, half a cycle              3.894   0.4500      0    0.150      41.7   0.00002   0.00002
tempo step, 120 -> 140bpm             2.992   0.0725      0    0.032       5.6   0.00059   0.00001
x0.75, 240s alignment                 0.000   0.0000      0    0.000       0.0   0.00003   0.00000
x1, 5% clock jitter                  29.986   0.0268     40    0.013       2.9   0.01048   0.00000
clock lost 6s, then back              0.000   0.0009      0    0.000       0.1   0.00002   0.00000
phase step @ 500us tick               4.061   0.5000      0    0.150      41.7   0.00002   0.00002
phase step @ 2000us tick              4.248   0.4989      0    0.150      16.7   0.00207   0.00014
```

| | 2026-08-16 | now |
|---|---|---|
| two channels at one rate, after a detour | **0.5 cycles apart** | 0.0000 |
| x3/4 -> x1/4, steady-state RMS | **3.19 beats**, on the clamp | 0.00006 beats |
| scene snap, frequency slew | 34.7 | **0.5** |
| scene snap, peak error | 0.2500 | **0.2217** |
| crossfade settling after, slew | 0.1 | 0.5 |
| fader worked back and forth, peak | 1.7739 | **0.6663** |
| fader worked back and forth, RMS | 0.37573 | **0.18585** |
| sweep, peak error / slew | 0.2362 / 45.4 | 0.2777 / 92.3 |
| steady state, jitter, tempo, clock loss | — | unchanged to the digit |

**What got worse, and why it is accepted.** During a fader that is still moving,
peak error goes 0.236 -> 0.278 beats and slew 45 -> 92 x rate/s. A sweep never
holds a ratio still for `PLL_RATIO_STABLE_US`, so the rational stays at the
pre-sweep value throughout and the loop leans on the clamp against a rate that
is running away from it. The old code tracked the sweep because its target used
the live ratio - which is the same property that made the target step at every
period wrap. The trade is a slightly firmer lean *while a hand is on the fader*
against a target that is continuous and a grid that is shared. Peak error stays
a quarter of a beat, an eighth of what the pre-rework loop did.

## Step 5: retuned faster, once it was correct

With the grid fixed, the remaining complaint was speed: channels drifted into
line rather than snapping to it. `PLL_TAU_S` had been 1.0s since it was named,
and that number was never chosen - it is what the hand-tuned gain happened to
be. Swept properly, it turns out to have been leaving most of the range unused.

**Nothing in this loop rings, at any speed.** It is first order - one pole, a
pure P controller - so it cannot overshoot in continuous time, and at a 500 us
tick even tau 0.15s is 300 ticks per time constant, nowhere near where
discretisation would change that. The phase-step case reports **0 crossings at
every tau measured from 1.0 down to 0.15**. So ringing is not what bounds the
speed, and the intuition that a faster loop must be a twitchier one is wrong
here.

**What bounds it is jitter.** This loop is the only thing filtering the incoming
clock, and its rejection goes as tau. Measured on the oscillator's own rate
under a 5% jittery clock:

| PLL_TAU_S | 1.0 | 0.5 | 0.35 | 0.25 | 0.2 |
|---|---|---|---|---|---|
| half-cycle step, settling | 3.89s | 2.35s | **1.97s** | 1.75s | 1.66s |
| rate wobble under 5% clock jitter | 0.013 | 0.023 | **0.029** | 0.039 | 0.046 |
| tempo step 120 -> 140, settling | 2.99s | 1.74s | **1.37s** | 1.15s | 0.98s |

Note that `rms_tail` *improves* as tau falls and is no guide here: it measures
phase error, which a faster loop always tracks down. The cost lands on the
output, as rate wobble, which is what the middle row measures.

**`PLL_MAX_PULL` is the separate knob, and it is the one transitions feel.** A
large error spends most of its life on the clamp, so the limit sets how fast it
closes and how far off-rate the channel leans while doing it. At tau 0.35, the
half-cycle step settles in 1.97s at pull 0.15, 1.55s at 0.25 and 1.41s at 0.35.

**Taken: tau 0.35, pull 0.25.** 0.35 is where buying more tail speed starts
costing visible wobble rather than nothing. A quarter off rate is a lean you can
see on a slow LFO and not a lurch; pull 0.35 was measured and rejected because
it pegs the clamp all the way through the settle after a fader move, which is
the artifact the limit exists to prevent.

### After (2026-08-18, PLL_TAU_S 0.35, PLL_MAX_PULL 0.25)

```
scenario                             settle     peak  cross     fdev     fslew  rms_tail      jump
                                          s    beats      n   x rate  x rate/s     beats    cycles
lock, x1 @ 120bpm                     0.000   0.0000      0    0.000       0.0   0.00001   0.00000
scene A->B, x1 -> x2 (snap)           1.253   0.1797      0    0.250       1.4   0.00000   0.00001
scene sweep, during the 1s move       never   0.2519      1    0.250     166.5   0.20411   0.00016
scene sweep, settling after           0.896   0.2321      1    0.250       2.1   0.00000   0.00001
ratio sweep x1 -> x4 over 3s          never   0.2514     16    0.250     166.7   0.08591   0.00024
fader worked back and forth          17.760   0.6664     46    0.256     168.2   0.17508   0.00024
phase step, half a cycle              1.546   0.4500      0    0.250      69.5   0.00001   0.00004
tempo step, 120 -> 140bpm             1.365   0.0509      0    0.067      15.9   0.00058   0.00001
x0.75, 240s alignment                 0.000   0.0000      0    0.000       0.0   0.00001   0.00000
x1, 5% clock jitter                  29.986   0.0207     42    0.029       8.2   0.00889   0.00000
clock lost 6s, then back              0.000   0.0009      0    0.001       0.4   0.00001   0.00000
phase step @ 500us tick               1.646   0.5000      0    0.250      69.5   0.00001   0.00004
phase step @ 2000us tick              1.748   0.4986      0    0.250      27.8   0.00206   0.00022
```

| | tau 1.0 / pull 0.15 | tau 0.35 / pull 0.25 |
|---|---|---|
| half-cycle phase step, settling | 3.894s | **1.546s** |
| scene snap x1 -> x2, settling | 3.340s | **1.253s** |
| crossfade, settling after | 3.181s | **0.896s** |
| tempo step, settling | 2.992s | **1.365s** |
| ringing, every case | 0-1 crossings | 0-1 crossings |
| rate wobble, 5% clock jitter | 0.013 | 0.029 |
| lean during a fader move | 0.138 | 0.250 |
| steady-state RMS, long-run alignment | 0.00003 | **0.00001** |

Nothing got slower and nothing started ringing. What was bought with a 2.5x
faster loop is a doubled rate wobble under a clock that is itself 5% wobbly, and
a quarter-rate rather than a seventh-rate lean while a fader is moving.

**`phase step, half a cycle` now asserts `settle_s < 2.5`.** The suite's bounds
are otherwise deliberately loose, and this one still is in the direction that
matters - it cannot fail on an improvement - but the speed is now a decision
rather than an accident, and something should hold it.

**Where the clock edge is stamped still does not matter**, though this is the
change that would have made it. The argument earlier in this document turned on
`rms_tail` being 0.00002 beats against a 248 us DAC frame; at tau 0.35 it reads
0.00001. Revisit at tau far below 0.2s, not before.

## Step 6: the grid the ratio has left

Step 4 made the target a function of the latched rational `p/q` rather than of
the live ratio, which is what makes it continuous across the wrap of
`beat_counter % q`. The corollary went unhandled, and the module found it.

**A ratio that has moved away from `p/q` describes a grid the oscillator cannot
be on.** A ratio in motion never holds still for `PLL_RATIO_STABLE_US`, so the
rational cannot be re-latched until the fader stops. Until then the error runs
off, wraps at half a cycle, and the correction slams sign on every wrap.

Found on a patch whose two scenes were **x64 and x32** - a whole FRQ sweep apart
rather than a detent, which is what a channel used as an audio-rate LFO looks
like. A one-second fader move **wrapped 49 times, and kept crossing 63 more
times after the fader stopped**: a ~50 Hz square-wave frequency modulation, heard
as grit through the transition rather than as pitch.

This document already predicted the mechanism in Step 4 - "the rational stays at
the pre-sweep value throughout and the loop leans on the clamp against a rate
that is running away from it" - and judged the cost acceptable. At x1 -> x4 it
is. At x64 -> x32 the divergence is thirty times larger, and the loop stops
leaning and starts wrapping. **The suite could not have caught it: every sweep
case in it was a low ratio, which drifts too slowly to wrap inside the move.**

So the correction runs only while the latched rational still describes the rate
the channel is running at - `|ratio * q - p| <= 0.05`, find_denominator's own
tolerance with slack. When it does not, the channel free-runs at the rate asked
for, which is exactly what an unrecognised ratio already got, and acquires the
moment the rate holds still.

`a_sweep_between_distant_ratios_does_not_fight_the_stale_grid` is the guard, and
it pins the crossings that were 49.

### After (2026-08-18)

```
scenario                             settle     peak  cross     fdev     fslew  rms_tail      jump
                                          s    beats      n   x rate  x rate/s     beats    cycles
lock, x1 @ 120bpm                     0.000   0.0000      0    0.000       0.0   0.00001   0.00000
scene A->B, x1 -> x2 (snap)           1.396   0.2500      0    0.250      83.4   0.00000   0.00008
scene sweep, during the 1s move       0.000   0.0022      0    0.003       0.9   0.00000   0.00000
scene sweep, settling after           1.395   0.2498      0    0.250      83.4   0.00000   0.00008
ratio sweep x1 -> x4 over 3s          0.000   0.0025      0    0.003       1.1   0.00000   0.00001
fader worked back and forth           never   0.4728      3    0.250      83.4   0.11087   0.00008
phase step, half a cycle              1.546   0.4500      0    0.250      69.5   0.00001   0.00004
tempo step, 120 -> 140bpm             1.365   0.0509      0    0.067      15.9   0.00058   0.00001
x0.75, 240s alignment                 0.000   0.0000      0    0.000       0.0   0.00001   0.00000
x1, 5% clock jitter                  29.986   0.0207     42    0.029       8.2   0.00889   0.00000
clock lost 6s, then back              0.000   0.0009      0    0.001       0.4   0.00001   0.00000
x64 -> x32, during the 1s move        0.000   0.0000      0    0.000       0.0   0.00000   0.00009
x64 -> x32, settling after            0.000   0.0004      0    0.001       0.2   0.00000   0.00010
phase step @ 500us tick               1.646   0.5000      0    0.250      69.5   0.00001   0.00004
phase step @ 2000us tick              1.748   0.4986      0    0.250      27.8   0.00206   0.00022
```

| | Step 5 | now |
|---|---|---|
| x64 -> x32 sweep, sign crossings | **49 in one second** | 0 |
| x64 -> x32, crossings after it stops | **63** | 0 |
| scene sweep during the move, peak | 0.2519 | **0.0022** |
| scene sweep during the move, slew | 166.5 | **0.9** |
| ratio sweep x1 -> x4, crossings | 16 | **0** |
| ratio sweep x1 -> x4, slew | 166.7 | **1.1** |
| fader worked back and forth, crossings | 46 | **3** |
| fader worked back and forth, RMS | 0.17508 | **0.11087** |
| scene snap x1 -> x2, settling | 1.253s | 1.396s |
| steady state, jitter, tempo, clock loss | — | unchanged |

Every sweep row improves, because the loop is no longer fighting a grid it
cannot reach in any of them. What it costs is 0.14s on a scene *snap*: the
correction is off while the ratio transitions, so acquisition starts later. That
is still 2.4x faster than the 3.34s this loop took before Step 5.

### Still to check on hardware

`PLL_MAX_PULL` is the knob: lower is smoother and slower to acquire, higher is
closer to the old behaviour. `PLL_RATIO_STABLE_US` decides how soon after a
fader stops the channel re-acquires. Re-run `./build-native/test_pll` after any
change and diff the table above.
