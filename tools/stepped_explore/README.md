# Stepped: measuring what SHP and MOD actually do

An offline harness for judging changes to `stepped_shape()` before writing any
C. `stmodel.py` is a replica of the shipping algorithm, verified against it to
1.8e-6 (float32 rounding) - regenerate the reference with `dump_c.c`:

    cc -O2 -I Core/Inc/Lib -o /tmp/dump_c tools/stepped_explore/dump_c.c \
        Core/Src/Lib/stepped.c -lm && /tmp/dump_c > /tmp/c_ref.txt

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
  `tests/test_stepped.c` plus the two the shipping algorithm cannot
  violate (parameter cyclicity, slot 0 length-independence).
- `dump_pattern.c` + `from_c.py` - the same character numbers plus **level**
  (how far the pattern's centre and its peak-to-peak move along a knob),
  computed on the C's own output rather than on the replica. Use these for any
  change to the *correction*: it lives in the generated table, which the replica
  does not carry.

The identity metrics come first historically and are the ones that mislead: the
shipping sweep contains 400 distinct sequences at 32 steps and still feels like
it does nothing. `steer` is what separates a knob that goes somewhere from one
that reshuffles.

`layered.py` is the round-two engine: every lever a (rate, depth) pair, on
whichever knob drives it. `final.py` is the spec the shipping C was written
from, and `verify_c.py` checks the two agree - the normalisation comes from the
generated header, so what it really tests is the value path.

## Use

    cd tools/stepped_explore
    python3 -c "import stmodel as M, character as C, variants as V; \
                C.scale(); C.report(M.Engine()); C.report(V.C5())"
    python3 -c "import invariants as I, variants as V; I.check(V.C5())"

`C.scale()` must be called before the first variant so every run is standardised
against the same baseline sigma.

For a change to the normalisation table, dump the C before and after instead -
the first file given sets the scale, so the columns are comparable:

    cc -O2 -I Core/Inc/Lib -o /tmp/dump_pat tools/stepped_explore/dump_pattern.c -lm
    /tmp/dump_pat > /tmp/after.txt      # `just stepped-table` in between
    python3 tools/stepped_explore/from_c.py /tmp/before.txt /tmp/after.txt

Read `char-st` there, not `steer`: three of the eleven features are level, so a
change that deliberately flattens level lowers `steer` while moving no
character at all.
