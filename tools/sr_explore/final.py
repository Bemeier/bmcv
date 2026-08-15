"""The proposal, written the way the C will be written.

C5 used sin/cos to drive the character levers. Those are calls on this target
and the hot path cannot afford them, so every drive here is a triangle put
through a polynomial S-curve instead. The shapes are close enough that none of
the measurements move, and the file is now a spec: the C must reproduce it.
"""
import numpy as np
import srmodel as M
from srmodel import tri, SLOT_BASE, SLOT_RATE, SLOT_GATE, SR_MAX_LENGTH, hash01, _seed

# Second gate table, for the rotation MOD applies to which steps tie.
SLOT_GATE2 = np.array([hash01(_seed(i) ^ 0x165667B1) for i in range(SR_MAX_LENGTH)])

CONTOUR = 1.0     # depth of the melodic-contour blend
CONTOUR_RATE = 3
TAPS = 4          # slots the contour averages over
BEND = 0.8        # depth of the value shaper, on SHP
BEND_RATE = 1
BEND_MOD = 0.45   # and the layer of it MOD drives
BEND_MOD_RATE = 3
BEND_LIMIT = 0.95
SPAN = 0.7        # how far the peak-to-peak may duck
SPAN_RATE = 3
SPAN_FLOOR = 0.75
MORPH_MOD = 0.4   # MOD's share of the orbit
TIE_WIDTH = 0.25  # how wide, in gate units, a tie crossfades over
SPIN = 0.5        # turns of the gate rotation across the MOD sweep
SWING = 0.4       # step-width skew at full MOD
SWING_RATE = 1
MOTIF = 0.7       # depth of the sub-cycle fold at the MOD extremes
MOTIF_RATE = 1
MOTIF_MIN_LENGTH = 16  # below this a quarter-cycle is not a phrase
NORM_MAX_GAIN = 40.0
NORM_TARGET = 1.3
PULL_MAX = 1.0
GAIN_SUBS = 3
MOD_BINS = 16
NORM_BINS = 128


def scurve(t):
    """Odd S-curve on [-1,1]: value and slope match a sine at both peaks."""
    return t * (1.5 - 0.5 * t * t)


def bump(x, rate=1):
    """A train of `rate` humps: 0 at every cycle end, 1 in every middle."""
    f = np.mod(x * rate, 1.0)
    return M.smoothstep(1.0 - np.abs(2.0 * f - 1.0))


def swingf(x, rate=1):
    """The same train, signed: `rate` excursions, zero at every boundary."""
    return scurve(-tri(x * rate + 0.25))


def orbit_of(shape, mod):
    """Where the slot values sit. MOD gets a share of it, which is what lets it
    reach patterns SHP never lands on. Everything downstream is periodic in it
    with period 1, which is what the normalisation table's axis relies on."""
    return 0.5 * (shape + 1.0) + MORPH_MOD * 0.5 * (mod + 1.0)


def shape_blend(v, bend):
    """Odd, monotone, fixes -1/0/+1. bend > 0 compresses the middle (calm,
    small intervals), bend < 0 expands it (gate-like). Slope at zero is
    1 - |bend|, so it never collapses the pattern however far it is driven."""
    if bend >= 0.0:
        return (1.0 - bend) * v + bend * v * np.abs(v)
    b = -bend
    return (1.0 - b) * v + b * np.sign(v) * np.sqrt(np.abs(v))


def dof_fade(length):
    """The contour needs slots to work on; four of them averaged out of a
    three-step cycle leaves nothing behind."""
    return float(np.clip((length - 4) / 8.0, 0.0, 1.0))


def tie_fade(length):
    return float(np.clip((length - 2) / M.SR_HOLD_FADE_IN_STEPS, 0.0, 1.0))


class Final(M.Engine):
    name = "FINAL (C spec)"

    # --- the value each slot carries -------------------------------------
    def slot_values(self, slots, shape, mod):
        o = orbit_of(shape, mod)
        f = np.mod(o, 1.0)
        d = 0.5 * (mod + 1.0)
        slots = np.asarray(slots)
        raw = tri(SLOT_BASE[slots] + o * SLOT_RATE[slots])
        acc = np.zeros(len(slots), dtype=float)
        for t in range(TAPS):
            j = np.maximum(slots - t, 0)
            acc += tri(SLOT_BASE[j] + o * SLOT_RATE[j])
        smoothed = acc / TAPS
        w = CONTOUR * dof_fade(self._length) * bump(f, CONTOUR_RATE)
        v = raw * (1.0 - w) + smoothed * w
        bend = np.clip(BEND * swingf(f, BEND_RATE) + BEND_MOD * swingf(d, BEND_MOD_RATE),
                       -BEND_LIMIT, BEND_LIMIT)
        return shape_blend(v, float(bend))

    # --- which steps tie, and how much ------------------------------------
    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * SPIN * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def tie_weights(self, shape, mod, length):
        g = self.gates(shape, mod, length)
        p = self.hold_probability(mod, length)
        w = M.smoothstep(np.clip((g - p) / TIE_WIDTH + 0.5, 0.0, 1.0))
        w = 1.0 - (1.0 - w) * tie_fade(length)
        w[::M.SR_JUMP_GRID] = 1.0
        return w

    # --- where the steps sit in the cycle ---------------------------------
    def step_widths(self, shape, mod, length):
        a = -SWING * swingf(0.5 * (mod + 1.0), SWING_RATE)
        k = np.arange(length)
        w = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return w / w.sum()

    # --- the pattern -------------------------------------------------------
    #
    # The motif fold is applied to the slot values *before* the tie chain, not
    # to the shown values after it. Folding afterwards needs three chains per
    # sample instead of one, for measurements that come out the same or worse -
    # and folding first is arguably the better idea anyway, since the riff then
    # repeats while the tie pattern stays full-length, so the repeat is varied
    # rhythmically rather than being a literal copy.
    def steps(self, shape, mod, length):
        self._length = length
        w = self.tie_weights(shape, mod, length)
        val = self.slot_values(np.arange(length), shape, mod)
        if length >= MOTIF_MIN_LENGTH and MOTIF > 0.0:
            mw = MOTIF * (1.0 - bump(0.5 * (mod + 1.0), MOTIF_RATE))
            folded = np.mod(np.arange(length), length // 4)
            val = val + (val[folded] - val) * mw
        v = np.empty(length)
        v[0] = val[0]
        for i in range(1, length):
            v[i] = v[i - 1] + (val[i] - v[i - 1]) * w[i]
        return v, np.arange(length), w > 0.5

    def norm_for(self, shape, mod, length_idx):
        self._length = M.SR_LENGTHS[length_idx]
        saved, M.SR_NORM_MAX_GAIN = M.SR_NORM_MAX_GAIN, NORM_MAX_GAIN
        try:
            return super().norm_for(shape, mod, length_idx)
        finally:
            M.SR_NORM_MAX_GAIN = saved

    def __init__(self):
        super().__init__()
        self._length = 16
