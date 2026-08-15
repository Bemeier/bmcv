"""Variants A1-A6 (SHP) and B1-B5 (MOD).

Every one keeps: cyclic in the parameter, slot 0 length-independent, the cycle
closing on itself, O(1) work per sample.
"""
import numpy as np
import srmodel as M
from srmodel import Engine, tri, SLOT_BASE, SLOT_RATE, SLOT_GATE, SR_MAX_LENGTH, hash01, _seed

M32 = 0xFFFFFFFF

# Extra per-slot tables the variants need. Same generator style as the shipping
# table, so they cost a build-time addition to stepped_random_table.h.
SLOT_STAGGER = np.array([hash01(_seed(i) ^ 0x27D4EB2F) for i in range(SR_MAX_LENGTH)])
SLOT_GATE2 = np.array([hash01(_seed(i) ^ 0x165667B1) for i in range(SR_MAX_LENGTH)])
SLOT_BASE2 = np.array([hash01(_seed(i) ^ 0x9E3779B1) for i in range(SR_MAX_LENGTH)])
SLOT_RATE2 = np.array([1 + int(hash01(_seed(i) ^ 0x2545F491) * 4) for i in range(SR_MAX_LENGTH)],
                      dtype=float)


# --------------------------------------------------------------------------
# A1 - wider orbit rates. The control case.
# --------------------------------------------------------------------------
class A1(Engine):
    def __init__(self, max_rate=10):
        super().__init__()
        self.name = f"A1 rate spread (max {max_rate})"
        self.rate = np.array([min(max_rate, 1 + int(hash01(_seed(i) ^ 0x85EBCA6B) * max_rate))
                              for i in range(SR_MAX_LENGTH)], dtype=float)

    def slot_values(self, slots, shape, mod):
        return tri(SLOT_BASE[slots] + self.morph(shape) * self.rate[slots])


# --------------------------------------------------------------------------
# A2 - shaper. SHP walks a circle through a value-shaping curve, so the
# distribution of levels changes rather than only which levels come up.
#
#   bend > 0 pulls values toward the centre (calm, small intervals)
#   bend < 0 pushes them to the edges (binary, gate-like)
#
# Cyclic because the drive is cos/sin of the morph, so -1 and +1 land together.
# Slot 0 keeps a length-independent value: the shaper is pointwise.
# --------------------------------------------------------------------------
def shape_curve(v, bend):
    """Odd, monotone, fixes -1/0/+1. bend in [-1,1]."""
    a = np.abs(v)
    if bend >= 0:
        out = a ** (1.0 + 2.0 * bend)
    else:
        out = 1.0 - (1.0 - a) ** (1.0 - 2.0 * bend)
    return np.sign(v) * out


class A2(Engine):
    def __init__(self, depth=1.0):
        super().__init__()
        self.name = f"A2 shaper (depth {depth})"
        self.depth = depth

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        v = tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])
        bend = self.depth * np.cos(2.0 * np.pi * m)
        return shape_curve(v, bend)


# --------------------------------------------------------------------------
# A3 - contour. Blend each slot against a short FIR over the slots before it:
# a melodic walk at one end, independent leaps at the other.
#
# The FIR is anchored (indices clamp at 0, never wrap), so slot 0 still reads
# its own raw value and length switching stays seamless. O(1): FIR_N extra
# tri() calls.
# --------------------------------------------------------------------------
FIR_N = 4


class A3(Engine):
    def __init__(self, depth=1.0, taps=FIR_N):
        super().__init__()
        self.name = f"A3 contour (depth {depth}, {taps} taps)"
        self.depth = depth
        self.taps = taps

    def _raw(self, slots, m):
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        slots = np.asarray(slots)
        acc = np.zeros(len(slots))
        for t in range(self.taps):
            acc += self._raw(np.maximum(slots - t, 0), m)
        smoothed = acc / self.taps
        # the walk loses span; put it back so the blend does not just fade out
        smoothed *= 1.6
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.depth
        return np.clip(self._raw(slots, m) * (1.0 - w) + smoothed * w, -1.0, 1.0)


# --------------------------------------------------------------------------
# A4 - motif fold. Fold the slot index onto a sub-period so the cycle repeats a
# riff, blended cyclically against the through-composed reading.
#
# Folding is index-only, so slot 0 is untouched. The fold period divides the
# length, keeping the cycle closed.
# --------------------------------------------------------------------------
class A4(Engine):
    def __init__(self, depth=1.0):
        super().__init__()
        self.name = f"A4 motif fold (depth {depth})"
        self.depth = depth

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def steps(self, shape, mod, length):
        src, jumps = self.source_slots(shape, mod, length)
        m = self.morph(shape)
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.depth
        period = max(2, length // 4)
        folded = np.mod(src, period)
        a = self.slot_values(src, shape, mod)
        b = self.slot_values(folded, shape, mod)
        return a * (1.0 - w) + b * w, src, jumps


# --------------------------------------------------------------------------
# A6 - staggered morph. Each slot steps through its orbit on its own schedule
# instead of every slot sliding at once: the module's own stepped-random idea,
# applied in the morph domain.
#
#   u = m*K + stagger_i, q = (floor(u) + eased frac) / K, v = tri(base + q*rate)
#
# At m=0 and m=1, q differs by exactly 1 and rate is an integer, so the sweep is
# still seamless.
# --------------------------------------------------------------------------
class A6(Engine):
    def __init__(self, K=6, hold=0.55):
        super().__init__()
        self.name = f"A6 staggered morph (K={K}, hold={hold})"
        self.K = K
        self.hold = hold

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        u = m * self.K + SLOT_STAGGER[slots]
        n = np.floor(u)
        f = u - n
        e = np.clip((f - self.hold) / (1.0 - self.hold), 0.0, 1.0)
        q = (n - SLOT_STAGGER[slots] + M.smoothstep(e)) / self.K
        return tri(SLOT_BASE[slots] + q * SLOT_RATE[slots])


# --------------------------------------------------------------------------
# A5 - combination: shaper + contour on one closed loop, 90 degrees apart, so
# SHP traverses a circle in character space instead of retracing an out-and-back.
# --------------------------------------------------------------------------
class A5(Engine):
    def __init__(self, bend=1.0, contour=1.0, taps=FIR_N):
        super().__init__()
        self.name = f"A5 shaper+contour (bend {bend}, contour {contour})"
        self.bend, self.contour, self.taps = bend, contour, taps

    def _raw(self, slots, m):
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        slots = np.asarray(slots)
        raw = self._raw(slots, m)
        acc = np.zeros(len(slots))
        for t in range(self.taps):
            acc += self._raw(np.maximum(slots - t, 0), m)
        smoothed = 1.6 * acc / self.taps
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.contour
        v = np.clip(raw * (1.0 - w) + smoothed * w, -1.0, 1.0)
        return shape_curve(v, self.bend * np.sin(2.0 * np.pi * m))


# --------------------------------------------------------------------------
# MOD variants.
# --------------------------------------------------------------------------

# B1 - gate phase. Instead of one threshold against a fixed per-slot gate, give
# each slot a moving gate: gate_i(mod) = tri-ish orbit of its own. The tie set
# then re-arranges continuously as MOD turns instead of nesting, so no part of
# the travel is dead.
class B1(Engine):
    def __init__(self, spin=2.0, density=True):
        super().__init__()
        self.name = f"B1 gate phase (spin {spin})"
        self.spin = spin
        self.density = density

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g


# B2 - cyclic density: density out and back over the knob, so MOD wraps.
class B2(Engine):
    name = "B2 cyclic density"

    def hold_probability(self, mod, length):
        d = 0.5 * (1.0 - np.cos(np.pi * (mod + 1.0)))  # 0 at -1 and +1, 1 in the middle
        p = M.SR_HOLD_MAX * d
        return float(p * np.clip((length - 2) / M.SR_HOLD_FADE_IN_STEPS, 0.0, 1.0))


# B3 - swing: MOD skews step widths. Widths still sum to 1, so the loop closes.
class B3(Engine):
    def __init__(self, depth=0.6):
        super().__init__()
        self.name = f"B3 swing (depth {depth})"
        self.depth = depth

    def step_widths(self, shape, mod, length):
        a = self.depth * np.sin(np.pi * mod)
        k = np.arange(length)
        w = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return w / w.sum()


# B4 - second morph axis: MOD moves the slot values along an orbit set of its
# own, so (SHP, MOD) is a torus rather than a line with a density trim.
class B4(Engine):
    def __init__(self, depth=1.0, keep_density=True):
        super().__init__()
        self.name = f"B4 second axis (depth {depth}, density {keep_density})"
        self.depth = depth
        self.keep_density = keep_density

    def hold_probability(self, mod, length):
        if self.keep_density:
            return super().hold_probability(mod, length)
        return 0.30 * float(np.clip((length - 2) / M.SR_HOLD_FADE_IN_STEPS, 0.0, 1.0))

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        m2 = 0.5 * (mod + 1.0)
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots]
                   + self.depth * m2 * SLOT_RATE2[slots])


# B5 - combination: gate phase (re-arranging ties) plus a density trend.
class B5(Engine):
    def __init__(self, spin=2.0):
        super().__init__()
        self.name = f"B5 gate phase + density (spin {spin})"
        self.spin = spin

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g


# --------------------------------------------------------------------------
# B6 - MOD as a linear combination: the density it already has, plus a slow
# rotation of *which* slots tie, plus swing. Density keeps its documented
# monotone meaning; the other two remove the dead travel between gate crossings
# and give MOD a character of its own instead of only thinning SHP's pattern.
# --------------------------------------------------------------------------
class B6(Engine):
    def __init__(self, spin=1.5, swing=0.5, motif=0.0):
        super().__init__()
        self.name = f"B6 density+gatephase+swing (spin {spin}, swing {swing}, motif {motif})"
        self.spin, self.swing, self.motif = spin, swing, motif

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def step_widths(self, shape, mod, length):
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        w = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return w / w.sum()

    def steps(self, shape, mod, length):
        vals, src, jumps = super().steps(shape, mod, length)
        if self.motif <= 0.0:
            return vals, src, jumps
        w = self.motif * 0.5 * (1.0 - np.cos(2.0 * np.pi * 0.5 * (mod + 1.0)))
        period = max(2, length // 4)
        b = self.slot_values(np.mod(src, period), shape, mod)
        return vals * (1.0 - w) + b * w, src, jumps


# --------------------------------------------------------------------------
# C - the combination: A5 on SHP (shaper + contour on one closed loop) and B6
# on MOD (density + gate phase + swing).
# --------------------------------------------------------------------------
class C(Engine):
    def __init__(self, bend=1.0, contour=1.0, taps=FIR_N, spin=1.5, swing=0.5, motif=0.6):
        super().__init__()
        self.name = "C  A5(SHP) + B6(MOD)"
        self.bend, self.contour, self.taps = bend, contour, taps
        self.spin, self.swing, self.motif = spin, swing, motif

    def _raw(self, slots, m):
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        slots = np.asarray(slots)
        raw = self._raw(slots, m)
        acc = np.zeros(len(slots))
        for t in range(self.taps):
            acc += self._raw(np.maximum(slots - t, 0), m)
        smoothed = 1.6 * acc / self.taps
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.contour
        v = np.clip(raw * (1.0 - w) + smoothed * w, -1.0, 1.0)
        return shape_curve(v, self.bend * np.sin(2.0 * np.pi * m))

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def step_widths(self, shape, mod, length):
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        w = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return w / w.sum()

    def steps(self, shape, mod, length):
        vals, src, jumps = super().steps(shape, mod, length)
        if self.motif <= 0.0:
            return vals, src, jumps
        w = self.motif * 0.5 * (1.0 - np.cos(2.0 * np.pi * 0.5 * (mod + 1.0)))
        period = max(2, length // 4)
        b = self.slot_values(np.mod(src, period), shape, mod)
        return vals * (1.0 - w) + b * w, src, jumps


# --------------------------------------------------------------------------
# B7 - soft ties. The reason MOD lurches is that a tie is a binary decision:
# as the density threshold crosses a slot's gate, that step switches from its
# own value to the previous one in a single knob position, jumping up to 1.45
# of a 2.0 range. Between crossings nothing happens at all. Dead, dead, LURCH.
#
# So crossfade the tie instead of switching it: a step blends from its own
# value toward the previous one as the threshold approaches its gate. MOD then
# moves the pattern continuously everywhere on its travel, and a half-tied step
# is musically a smaller interval rather than a glitch.
#
# The look-back stays O(1): every SR_JUMP_GRID-th slot is still pinned fully
# open, so a chain of partial ties is at most SR_JUMP_GRID long.
# --------------------------------------------------------------------------
TIE_WIDTH = 0.35


class SoftTieMixin:
    tie_width = TIE_WIDTH

    def tie_weights(self, shape, mod, length):
        g = self.gates(shape, mod, length)
        p = self.hold_probability(mod, length)
        w = M.smoothstep(np.clip((g - p) / self.tie_width + 0.5, 0.0, 1.0))
        # Short patterns fade the tie out, for the same reason the probability
        # is faded: with three steps in a cycle even a partial tie flattens too
        # much of it to be worth having.
        fade = float(np.clip((length - 2) / M.SR_HOLD_FADE_IN_STEPS, 0.0, 1.0))
        w = 1.0 - (1.0 - w) * fade
        w[::M.SR_JUMP_GRID] = 1.0   # pinned slots always take their own value
        return w

    def steps(self, shape, mod, length):
        w = self.tie_weights(shape, mod, length)
        raw = self.slot_values(np.arange(length), shape, mod)
        v = np.empty(length)
        v[0] = raw[0]
        for i in range(1, length):
            v[i] = v[i - 1] + (raw[i] - v[i - 1]) * w[i]
        return v, np.arange(length), w > 0.5


class B7(SoftTieMixin, Engine):
    def __init__(self, width=TIE_WIDTH, swing=0.0):
        super().__init__()
        self.name = f"B7 soft ties (width {width}, swing {swing})"
        self.tie_width = width
        self.swing = swing

    def step_widths(self, shape, mod, length):
        if self.swing <= 0.0:
            return super().step_widths(shape, mod, length)
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        wd = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return wd / wd.sum()


# B8 - soft ties plus the gate phase rotation and swing from B6: MOD as a
# linear combination of density, which steps tie, and where the beat sits.
class B8(SoftTieMixin, Engine):
    def __init__(self, width=TIE_WIDTH, spin=1.0, swing=0.4):
        super().__init__()
        self.name = f"B8 soft ties + gate phase + swing (spin {spin}, swing {swing})"
        self.tie_width, self.spin, self.swing = width, spin, swing

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def step_widths(self, shape, mod, length):
        if self.swing <= 0.0:
            return super().step_widths(shape, mod, length)
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        wd = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return wd / wd.sum()


# --------------------------------------------------------------------------
# C2 - the proposal: A5 on SHP, B8 on MOD.
#
# SHP: value morph as today, plus a closed loop through shaper (distribution)
#      and contour (melodic walk vs leaps), 90 degrees apart.
# MOD: density as today but crossfaded rather than switched, plus a rotation of
#      which steps tie, plus swing.
# --------------------------------------------------------------------------
class C2(SoftTieMixin, Engine):
    def __init__(self, bend=1.0, contour=1.0, taps=FIR_N,
                 width=TIE_WIDTH, spin=1.0, swing=0.4):
        super().__init__()
        self.name = "C2  A5(SHP) + B8(MOD)"
        self.bend, self.contour, self.taps = bend, contour, taps
        self.tie_width, self.spin, self.swing = width, spin, swing

    def _raw(self, slots, m):
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        slots = np.asarray(slots)
        raw = self._raw(slots, m)
        acc = np.zeros(len(slots))
        for t in range(self.taps):
            acc += self._raw(np.maximum(slots - t, 0), m)
        smoothed = 1.6 * acc / self.taps
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.contour
        v = np.clip(raw * (1.0 - w) + smoothed * w, -1.0, 1.0)
        return shape_curve(v, self.bend * np.sin(2.0 * np.pi * m))

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def step_widths(self, shape, mod, length):
        if self.swing <= 0.0:
            return Engine.step_widths(self, shape, mod, length)
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        wd = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return wd / wd.sum()


# --------------------------------------------------------------------------
# C3 - C2 with two corrections found by measurement:
#
#  * the contour FIR is faded out on short patterns. Averaging four slots of a
#    three-slot cycle leaves them all equal and the pattern goes flat, and the
#    normalisation cannot lift it because the anchor cap binds first.
#  * MOD gains a motif fold, which is the one lever whose effect grows with the
#    step count. Gate phase and density both wash out above 24 steps - the very
#    range the complaint is about - because pattern statistics average out as
#    slots are added. Folding the slot index onto a quarter-length sub-cycle is
#    structural, so it reads at any length, and it is only enabled where a
#    quarter is a musically meaningful phrase.
# --------------------------------------------------------------------------
class C3(SoftTieMixin, Engine):
    def __init__(self, bend=0.7, contour=1.0, taps=FIR_N,
                 width=0.25, spin=0.5, swing=0.4, motif=0.7):
        super().__init__()
        self.name = "C3 proposal"
        self.bend, self.contour, self.taps = bend, contour, taps
        self.tie_width, self.spin, self.swing, self.motif = width, spin, swing, motif
        self._length = 16

    @staticmethod
    def _contour_fade(length):
        return float(np.clip((length - 4) / 8.0, 0.0, 1.0))

    def _raw(self, slots, m):
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        slots = np.asarray(slots)
        raw = self._raw(slots, m)
        acc = np.zeros(len(slots))
        for t in range(self.taps):
            acc += self._raw(np.maximum(slots - t, 0), m)
        smoothed = 1.6 * acc / self.taps
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.contour * self._contour_fade(self._length)
        v = np.clip(raw * (1.0 - w) + smoothed * w, -1.0, 1.0)
        return shape_curve(v, self.bend * np.sin(2.0 * np.pi * m))

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def step_widths(self, shape, mod, length):
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        wd = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return wd / wd.sum()

    def steps(self, shape, mod, length):
        self._length = length
        v, src, jumps = super().steps(shape, mod, length)
        if self.motif <= 0.0 or length < 16:
            return v, src, jumps
        w = self.motif * 0.5 * (1.0 - np.cos(np.pi * (mod + 1.0)))
        period = length // 4
        folded = np.mod(np.arange(length), period)
        return v * (1.0 - w) + v[folded] * w, src, jumps


# --------------------------------------------------------------------------
# C4 - C3 with the three invariant breaks fixed.
#
#  * The FIR no longer boosts its output by a fudge factor to recover span.
#    That factor made slot 0 read 1.6x its raw value, and once the contour
#    depth became length-dependent so did slot 0 - which is exactly the value
#    seamless length switching rests on. Averaging with clamped indices leaves
#    slot 0 equal to its own raw value, so with no boost the anchor is exact and
#    the existing normalisation restores the span, which is what it is for.
#  * The motif fold is driven by 1-cos(pi*mod), which is 0 at MOD centre and
#    equal at both extremes. The previous drive was fully folded at centre.
#  * The shaper is faded out on short patterns alongside the contour: with three
#    slots it can pull all of them to zero, and the normalisation cannot lift
#    that back because the anchor cap binds before the gain does.
# --------------------------------------------------------------------------
class C4(SoftTieMixin, Engine):
    def __init__(self, bend=0.7, contour=1.0, taps=FIR_N,
                 width=0.25, spin=0.5, swing=0.4, motif=0.7):
        super().__init__()
        self.name = "C4 proposal"
        self.bend, self.contour, self.taps = bend, contour, taps
        self.tie_width, self.spin, self.swing, self.motif = width, spin, swing, motif
        self._length = 16

    @staticmethod
    def _dof_fade(length):
        """Both value-domain levers need slots to work on."""
        return float(np.clip((length - 4) / 8.0, 0.0, 1.0))

    def _raw(self, slots, m):
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        slots = np.asarray(slots)
        fade = self._dof_fade(self._length)
        raw = self._raw(slots, m)
        acc = np.zeros(len(slots))
        for t in range(self.taps):
            acc += self._raw(np.maximum(slots - t, 0), m)
        smoothed = acc / self.taps
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.contour * fade
        v = raw * (1.0 - w) + smoothed * w
        return shape_curve(v, self.bend * fade * np.sin(2.0 * np.pi * m))

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def step_widths(self, shape, mod, length):
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        wd = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return wd / wd.sum()

    def steps(self, shape, mod, length):
        self._length = length
        v, src, jumps = super().steps(shape, mod, length)
        if self.motif <= 0.0 or length < 16:
            return v, src, jumps
        w = self.motif * 0.5 * (1.0 - np.cos(np.pi * mod))
        period = length // 4
        folded = np.mod(np.arange(length), period)
        return v * (1.0 - w) + v[folded] * w, src, jumps


# --------------------------------------------------------------------------
# C5 - the shaper as a blend rather than a power.
#
# The power-law shaper had to be faded out on short patterns or it crushed
# every value to zero, and that fade made slot 0 length-dependent, which is
# what seamless length switching rests on. A blend fixes both: its slope at
# zero is (1 - bend), so it can compress the middle without ever collapsing it,
# no fade is needed, and slot 0 stays length-independent.
#
# It is also the affordable form. powf() at 4 kHz per channel is not; v*|v| is
# one multiply and sqrtf() is a single instruction on this core.
# --------------------------------------------------------------------------
def shape_blend(v, bend):
    """Odd, monotone, fixes -1/0/+1. bend > 0 compresses the middle (calm,
    small intervals), bend < 0 expands it (gate-like)."""
    if bend >= 0.0:
        return (1.0 - bend) * v + bend * v * np.abs(v)
    b = -bend
    return (1.0 - b) * v + b * np.sign(v) * np.sqrt(np.abs(v))


class C5(SoftTieMixin, Engine):
    def __init__(self, bend=0.7, contour=1.0, taps=FIR_N,
                 width=0.25, spin=0.5, swing=0.4, motif=0.7):
        super().__init__()
        self.name = "C5 proposal"
        self.bend, self.contour, self.taps = bend, contour, taps
        self.tie_width, self.spin, self.swing, self.motif = width, spin, swing, motif
        self._length = 16

    @staticmethod
    def _dof_fade(length):
        return float(np.clip((length - 4) / 8.0, 0.0, 1.0))

    def _raw(self, slots, m):
        return tri(SLOT_BASE[slots] + m * SLOT_RATE[slots])

    def slot_values(self, slots, shape, mod):
        m = self.morph(shape)
        slots = np.asarray(slots)
        raw = self._raw(slots, m)
        acc = np.zeros(len(slots))
        for t in range(self.taps):
            acc += self._raw(np.maximum(slots - t, 0), m)
        smoothed = acc / self.taps
        # The contour blend leaves slot 0 alone on its own (its clamped taps are
        # all slot 0), so this fade costs nothing at the anchor.
        w = 0.5 * (1.0 - np.cos(2.0 * np.pi * m)) * self.contour * self._dof_fade(self._length)
        v = raw * (1.0 - w) + smoothed * w
        return shape_blend(v, self.bend * np.sin(2.0 * np.pi * m))

    def gates(self, shape, mod, length):
        d = 0.5 * (mod + 1.0)
        g = 0.5 * (1.0 + tri(SLOT_GATE[:length] + d * self.spin * (1.0 + SLOT_GATE2[:length])))
        g[::M.SR_JUMP_GRID] = 1.0
        return g

    def step_widths(self, shape, mod, length):
        a = self.swing * np.sin(np.pi * mod)
        k = np.arange(length)
        wd = 1.0 + a * np.where(k % 2 == 0, 1.0, -1.0)
        return wd / wd.sum()

    def steps(self, shape, mod, length):
        self._length = length
        v, src, jumps = super().steps(shape, mod, length)
        if self.motif <= 0.0 or length < 16:
            return v, src, jumps
        w = self.motif * 0.5 * (1.0 - np.cos(np.pi * mod))
        period = length // 4
        folded = np.mod(np.arange(length), period)
        return v * (1.0 - w) + v[folded] * w, src, jumps
