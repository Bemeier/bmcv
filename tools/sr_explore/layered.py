"""Levers at their own rates, superimposed, instead of one out-and-back each.

Today every character lever makes exactly one excursion across its knob: the
contour humps once, the shaper swings once. That spends the whole sweep on a
single traversal of character space, which is why a small turn is safely gentle
and why the knob still runs out of things to say.

Give each lever its own integer number of cycles across the sweep and they phase
against one another: a given contour depth comes back several times, but paired
with a different shaper and a different span each time. Cyclicity survives
because the rates are integers - every lever is home again at the end of the
knob.

The drives are all computed once per sample, in sr_morph(); the expensive work
is per-slot. So layering costs essentially nothing, and the real budget being
spent is how far a small turn of the knob is allowed to move the pattern.

New here beyond rates: `span`. The pattern's peak-to-peak is currently pinned to
1.3-2.0 by the normalisation at every setting, which is one of the reasons every
setting sounds equally emphatic. Letting it breathe folds into the normalisation
table at generation time, so it costs nothing at runtime and keeps slot 0 exact.
"""
import numpy as np
import srmodel as M
from srmodel import tri, SLOT_BASE, SLOT_RATE, SLOT_GATE, SR_MAX_LENGTH, hash01, _seed
from final import (SLOT_GATE2, scurve, shape_blend, dof_fade, tie_fade,
                   TAPS, TIE_WIDTH, MOTIF_MIN_LENGTH)


def bump_at(x, rate):
    """A train of `rate` humps across [0,1], 0 at every end, smooth throughout."""
    f = np.mod(x * rate, 1.0)
    return M.smoothstep(1.0 - np.abs(2.0 * f - 1.0))


def swing_at(x, rate):
    """A train of `rate` signed excursions, zero at every hump boundary."""
    return scurve(-tri(x * rate + 0.25))


def dip_at(x, rate):
    """bump_at inverted: 1 at every end of a cycle, 0 in its middle. Which of
    the two a lever wants is arbitrary; this is the one that puts the motif
    fold at the ends of the MOD knob, where it was."""
    return 1.0 - bump_at(x, rate)


class Layered(M.Engine):
    """Every lever a (rate, depth) pair, on whichever knob drives it."""

    def __init__(self, name="layered", **kw):
        super().__init__()
        self.name = name
        p = dict(
            # SHP levers
            contour=1.0, contour_rate=1,
            bend_s=0.6, bend_s_rate=1,
            span_s=0.0, span_s_rate=1,
            # MOD levers
            spin=0.5,
            swing=0.4, swing_rate=1,
            motif=0.7, motif_rate=1,
            bend_m=0.0, bend_m_rate=1, morph_m=0.0,
            span_m=0.0, span_m_rate=1,
            norm_max_gain=20.0, span_floor=0.75,
        )
        p.update(kw)
        self.p = p
        self._length = 16

    # --- drives -----------------------------------------------------------
    def _bend(self, m, d):
        p = self.p
        return (p["bend_s"] * swing_at(m, p["bend_s_rate"])
                + p["bend_m"] * swing_at(d, p["bend_m_rate"]))

    def _span(self, m, d):
        p = self.p
        return (1.0 - p["span_s"] * bump_at(m, p["span_s_rate"])
                - p["span_m"] * bump_at(d, p["span_m_rate"]))

    # --- value path -------------------------------------------------------
    def slot_values(self, slots, shape, mod):
        p = self.p
        m = self.morph(shape)
        d = 0.5 * (mod + 1.0)
        slots = np.asarray(slots)
        # MOD gets a share of the orbit itself, not just of the character
        # levers. Without it MOD can only rearrange and reshape the pattern SHP
        # picked; with it, MOD reaches patterns SHP never lands on.
        mm = p["morph_m"] * d
        raw = tri(SLOT_BASE[slots] + (m + mm) * SLOT_RATE[slots])
        acc = np.zeros(len(slots))
        for t in range(TAPS):
            j = np.maximum(slots - t, 0)
            acc += tri(SLOT_BASE[j] + (m + mm) * SLOT_RATE[j])
        w = p["contour"] * dof_fade(self._length) * bump_at(m, p["contour_rate"])
        v = raw * (1.0 - w) + (acc / TAPS) * w
        return shape_blend(v, np.clip(self._bend(m, d), -0.95, 0.95))

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.p["spin"] * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def tie_weights(self, shape, mod, length):
        g = self.gates(shape, mod, length)
        p = self.hold_probability(mod, length)
        w = M.smoothstep(np.clip((g - p) / TIE_WIDTH + 0.5, 0.0, 1.0))
        w = 1.0 - (1.0 - w) * tie_fade(length)
        w[::M.SR_JUMP_GRID] = 1.0
        return w

    def step_widths(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        a = -self.p["swing"] * swing_at(d, self.p["swing_rate"])
        k = np.arange(length)
        w = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return w / w.sum()

    def steps(self, shape, mod, length):
        self._length = length
        w = self.tie_weights(shape, mod, length)
        val = self.slot_values(np.arange(length), shape, mod)
        d = 0.5 * (mod + 1.0)
        mw = self.p["motif"] * dip_at(d, self.p["motif_rate"]) if length >= MOTIF_MIN_LENGTH else 0.0
        if mw > 0.0:
            folded = np.mod(np.arange(length), length // 4)
            val = val + (val[folded] - val) * mw
        v = np.empty(length)
        v[0] = val[0]
        for i in range(1, length):
            v[i] = v[i - 1] + (val[i] - v[i - 1]) * w[i]
        return v, np.arange(length), w > 0.5

    # --- normalisation, with span folded in -------------------------------
    def norm_for(self, shape, mod, length_idx):
        self._length = M.SR_LENGTHS[length_idx]
        saved, M.SR_NORM_MAX_GAIN = M.SR_NORM_MAX_GAIN, self.p["norm_max_gain"]
        try:
            g, off = super().norm_for(shape, mod, length_idx)
        finally:
            M.SR_NORM_MAX_GAIN = saved
        # Span scales about the anchor, which is slot 0, so slot 0 is untouched
        # and length switching stays seamless. Folded into the same affine the
        # table already carries: free at runtime.
        anchor = off / (1.0 - g) if abs(1.0 - g) > 1e-9 else self.pattern_anchor(shape, mod, length_idx)
        s = self._span(self.morph(shape), 0.5 * (mod + 1.0))

        # Never shrink a pattern past the floor. The generator knows the span it
        # is working with, so the lever can be held to what is actually
        # available here instead of being tuned down globally to whatever the
        # worst corner of the parameter space could survive. Where there is
        # headroom the lever gets its full depth; where there is not, it gets
        # what there is.
        vals, _, _ = self.steps(shape, mod, M.SR_LENGTHS[length_idx])
        raw = float(np.ptp(vals))
        if raw * g > 1e-6:
            s = max(s, min(1.0, self.p["span_floor"] / (raw * g)))
        return g * s, anchor * (1.0 - g * s)

    def pattern_anchor(self, shape, mod, length_idx):
        vals, _, _ = self.steps(shape, mod, M.SR_LENGTHS[length_idx])
        return vals[0]
