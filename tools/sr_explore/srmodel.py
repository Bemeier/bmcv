"""Python replica of Core/Src/Lib/stepped_random.c + tools/gen_sr_table.c.

Exact bit-for-bit replica of the hashes so the baseline metrics describe the
firmware that ships, not an approximation of it.
"""
import numpy as np

SR_MAX_LENGTH = 64
SR_JUMP_GRID = 4
SR_MAX_ORBIT_RATE = 4
SR_NORM_BINS = 128
SR_NORM_TARGET = 1.3
SR_NORM_MAX_GAIN = 10.0
SR_PROB_BINS = 8
SR_HOLD_MAX = 0.85
SR_HOLD_NEUTRAL = 0.30
SR_HOLD_FADE_IN_STEPS = 6.0
SR_LENGTHS = [3, 4, 5, 6, 8, 12, 16, 24, 32, 48, 64]

M32 = 0xFFFFFFFF


def hash_u32(x):
    x &= M32
    x ^= x >> 16
    x = (x * 0x7FEB352D) & M32
    x ^= x >> 15
    x = (x * 0x846CA68B) & M32
    x ^= x >> 16
    return x


def hash01(x):
    return (hash_u32(x) & 0x00FFFFFF) * (1.0 / 16777216.0)


def _seed(i):
    return hash_u32((i * 0x9E3779B9 + 0x6D2B79F5) & M32)


SLOT_BASE = np.array([hash01(_seed(i)) for i in range(SR_MAX_LENGTH)], dtype=np.float64)
SLOT_RATE = np.array(
    [min(SR_MAX_ORBIT_RATE, 1 + int(hash01(_seed(i) ^ 0x85EBCA6B) * SR_MAX_ORBIT_RATE))
     for i in range(SR_MAX_LENGTH)], dtype=np.float64)
SLOT_GATE = np.array(
    [1.0 if i % SR_JUMP_GRID == 0 else hash01(_seed(i) ^ 0xC2B2AE35)
     for i in range(SR_MAX_LENGTH)], dtype=np.float64)


def tri(x):
    f = np.mod(x, 1.0)
    return 4.0 * np.abs(f - 0.5) - 1.0


def smoothstep(x):
    return x * x * (3.0 - 2.0 * x)


# --------------------------------------------------------------------------
# Baseline engine, parameterised so variants can swap pieces out.
# --------------------------------------------------------------------------

class Engine:
    """Baseline stepped_random. Subclasses override slot_value/source_slot/etc."""

    name = "baseline"

    def __init__(self):
        self._norm_cache = {}

    # --- pieces a variant may override -------------------------------------
    def slot_values(self, slots, shape, mod):
        """Value of each slot index (array) at this shape/mod."""
        return tri(SLOT_BASE[slots] + self.morph(shape) * SLOT_RATE[slots])

    def morph(self, shape):
        return 0.5 * (shape + 1.0)

    def hold_probability(self, mod, length):
        mod = float(np.clip(mod, -1.0, 1.0))
        if mod <= 0.0:
            p = SR_HOLD_NEUTRAL * (1.0 + mod)
        else:
            p = SR_HOLD_NEUTRAL + mod * (SR_HOLD_MAX - SR_HOLD_NEUTRAL)
        return p * float(np.clip((length - 2) / SR_HOLD_FADE_IN_STEPS, 0.0, 1.0))

    def gates(self, shape, mod, length):
        """Per-slot gate values; a slot jumps when gate >= hold_probability."""
        return SLOT_GATE[:length]

    def step_widths(self, shape, mod, length):
        """Fraction of the cycle each step occupies; must sum to 1."""
        return np.full(length, 1.0 / length)

    # --- machinery ---------------------------------------------------------
    def source_slots(self, shape, mod, length):
        g = self.gates(shape, mod, length)
        p = self.hold_probability(mod, length)
        jumps = g >= p
        jumps[0] = True  # slot 0 always jumps: the loop point must close
        src = np.empty(length, dtype=int)
        last = 0
        for i in range(length):
            if jumps[i]:
                last = i
            src[i] = last
        return src, jumps

    def steps(self, shape, mod, length):
        """The value shown at each step, before normalisation."""
        src, jumps = self.source_slots(shape, mod, length)
        return self.slot_values(src, shape, mod), src, jumps

    # --- normalisation (recomputed, matching gen_sr_table.c) ---------------
    def norm_for(self, shape, mod, length_idx):
        length = SR_LENGTHS[length_idx]
        vals, _, _ = self.steps(shape, mod, length)
        lo, hi = vals.min(), vals.max()
        span = hi - lo
        anchor = vals[0]
        g = SR_NORM_MAX_GAIN if span < 1e-6 else float(np.clip(SR_NORM_TARGET / span, 1.0, SR_NORM_MAX_GAIN))
        if hi > anchor:
            g = min(g, (1.0 - anchor) / (hi - anchor))
        if lo < anchor:
            g = min(g, (1.0 + anchor) / (anchor - lo))
        g = max(g, 1.0)
        return g, anchor * (1.0 - g)

    def pattern(self, shape, mod, length_idx):
        """The note sequence: the value shown at each step, normalised. This is
        what the ear latches onto - the curve between the steps is joinery."""
        length = SR_LENGTHS[length_idx]
        vals, _, _ = self.steps(shape, mod, length)
        g, off = self.norm_for(shape, mod, length_idx)
        return np.clip(vals * g + off, -1.0, 1.0)

    def render(self, shape, mod, length_idx, n=256, hold=0.0, normalise=True):
        """One cycle, n samples, as the module would output it."""
        length = SR_LENGTHS[length_idx]
        vals, src, jumps = self.steps(shape, mod, length)
        widths = self.step_widths(shape, mod, length)
        edges = np.concatenate([[0.0], np.cumsum(widths)])
        phase = np.arange(n) / n
        step = np.clip(np.searchsorted(edges, phase, side="right") - 1, 0, length - 1)
        within = (phase - edges[step]) / widths[step]
        nxt = (step + 1) % length
        # the next step's shown value
        to = vals[nxt]
        frm = vals[step]
        span = 1.0 - hold
        ease = 1.0 if span <= 0 else np.clip((within - hold) / span, 0.0, 1.0)
        out = frm + (to - frm) * smoothstep(ease)
        if normalise:
            g, off = self.norm_for(shape, mod, length_idx)
            out = np.clip(out * g + off, -1.0, 1.0)
        return out
