# Stepped random: measuring what SHP and MOD actually do

An offline harness for judging changes to `stepped_random()` before writing any
C. `srmodel.py` is a replica of the shipping algorithm, verified against it to
1.8e-6 (float32 rounding) - regenerate the reference with `dump_c.c`:

    cc -O2 -I Core/Inc/Lib -o /tmp/dump_c tools/sr_explore/dump_c.c \
        Core/Src/Lib/stepped_random.c -lm && /tmp/dump_c > /tmp/c_ref.txt

## What it measures

- `metrics.py` - RMS distance between rendered cycles. "How different is this
  signal."
- `musical.py` - patterns quantised to note-sized levels, compared by Hamming
  distance. "How many different note sequences, and how many notes change per
  1% of knob travel."
- `character.py` - eleven features per pattern (span, roughness, turns,
  edginess, tie fraction, longest run, motif repetition, centroid, low-frequency
  energy, spread, swing), then the trajectory along the knob split into
  **steer** (its low-passed component: character the knob commands) and
  **jitter** (the residual: character that merely fluctuates because each
  position is a fresh random draw).
- `invariants.py` - the properties a variant may not break, mirroring
  `tests/test_stepped_random.c` plus the two the shipping algorithm cannot
  violate (parameter cyclicity, slot 0 length-independence).

The identity metrics come first historically and are the ones that mislead: the
shipping sweep contains 400 distinct sequences at 32 steps and still feels like
it does nothing. `steer` is what separates a knob that goes somewhere from one
that reshuffles.

`layered.py` is the round-two engine: every lever a (rate, depth) pair, on
whichever knob drives it. `final.py` is the spec the shipping C was written
from, and `verify_c.py` checks the two agree - the normalisation comes from the
generated header, so what it really tests is the value path.

## Use

    cd tools/sr_explore
    python3 -c "import srmodel as M, character as C, variants as V; \
                C.scale(); C.report(M.Engine()); C.report(V.C5())"
    python3 -c "import invariants as I, variants as V; I.check(V.C5())"

`C.scale()` must be called before the first variant so every run is standardised
against the same baseline sigma.
