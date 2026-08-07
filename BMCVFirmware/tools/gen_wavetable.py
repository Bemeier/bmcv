#!/usr/bin/env python3
"""Generate Core/Inc/Lib/wavetables.h - the SHAPE_LFO morphing wavetable.

    python3 tools/gen_wavetable.py            # write the header
    python3 tools/gen_wavetable.py --report   # print what it would generate

Run via `just wavetable`. The output is checked in, so this only needs running
after editing the shape family below - review the diff, then `just test`.

--------------------------------------------------------------------------
Why the table is built rather than drawn

The first table was sketched by hand in a visual editor, which is the right way
to find shapes and the wrong way to keep three properties that have to hold
everywhere at once:

  1. every slice reaches the full output swing,
  2. so does every *blend* of two neighbouring slices, because the runtime
     interpolates between them and a knob sitting between two entries is the
     normal case, not an edge case,
  3. the shape axis is a closed loop - SHP wraps at the parameter (`value +=
     delta * 256` on an int16_t) and the lookup wraps with `% WT_SLICES`, so
     the last slice blends back into the first.

The drawn table met (1) - every slice was full scale - and failed (2)
spectacularly: slices 13 and 14 were the same wave in opposite phase, so at the
midpoint between them they cancelled and the channel went **completely flat**.
That is not a shape anyone drew; it is what linear interpolation does between
two shapes nobody checked against each other.

--------------------------------------------------------------------------
How the invariant is made structural

Every wave here is built from a rise curve S(u), an odd monotone map of
[-1, 1] onto itself with S(+-1) = +-1. The wave is

    w(p) = S(4p - 1)          for p in [0, 0.5]
    w(p) = -w(p - 0.5)        for p in [0.5, 1]

and then rotated a quarter cycle on the way into the table, so that phase 0 is
the rising edge rather than the trough - see build_slice for why.

which gives, for free and for every member of the family:

  * it reaches -1 and +1 exactly - full swing, in every slice;
  * |w| <= 1 everywhere, because S maps into [-1, 1];
  * half-wave antisymmetry, so the DC offset is exactly zero. Sweeping SHP no
    longer walks the average level of the channel, which the drawn table did by
    as much as 0.7 of full scale;
  * and the property that actually matters: **a convex combination of monotone
    rise curves is a monotone rise curve.** Any blend of any two slices is
    therefore another member of the family, hits -1 at p=0 and +1 at p=0.5, and
    stays inside [-1, 1]. Full amplitude between slices is not tuned for, it is
    arithmetic - and it holds across the wrap as well as everywhere else.

--------------------------------------------------------------------------
The shape axis

A loop, from the flat extreme through the two canonical shapes to the pointy
extreme and back:

    shape -1 .......... 0 ......... +0.375 ... +0.625 ..... +1 (= -1)
    square           SINE         TRIANGLE    pointy       square

SINE sits at shape 0 because that is what a channel resets to, and it is exact:
S(u) = sin(pi*u/2) gives w(p) = -cos(2*pi*p) to the last bit of int16. TRIANGLE
is exact too, S(u) = u.

Not here, deliberately:

  * **saw / ramp** - MOD already makes one. The skew in phase_mod() is a
    monotone phase warp that fixes both cycle endpoints, and applied to the
    triangle it produces a ramp. A saw slice would be a second way to reach a
    shape that is already a knob turn away, and it cannot satisfy the anchoring
    above (its extreme is at the cycle boundary, not at the half cycle).
  * **stepped / staircase shapes** - SHAPE_STEPPED is an entire mode for these
    now, with a pattern length and a density of its own. The drawn table spent
    roughly thirty of its slices on them.
  * **pulse widths** - SHAPE_PWM is that mode, with a width and an envelope.
    Those slices were also where the drawn table's DC offset came from.
"""

import argparse
import math
import os

WT_LEN = 256  # samples per cycle. Power of two: the lookup masks with WT_LEN-1.
WT_SLICES = 64  # shape slices. Even, so shape 0 lands exactly on one.

FULL = 32767

# Where the named shapes sit on the axis, as slice indices.
SLICE_SQUARE = 0  # shape -1, and shape +1: the wrap point
SLICE_SINE = 32  # shape 0, a channel's default
SLICE_TRIANGLE = 44  # shape +0.375
SLICE_POINTY = 52  # shape +0.625
SLICE_SEAM = 56  # shape +0.75, where the return leg picks up

# How square the square is. Both families reach a true square only in the
# limit; at these exponents the edge takes about one sample of WT_LEN, so the
# wave has no vertical step in it and is a square for every practical purpose.
# A real gate is SHAPE_PWM's job.
SQUARE_H = 0.045

# How pointy the pointy extreme is, and the matching parameter in the other
# family. The two are chosen to have the same RMS, which is as close as curves
# from different families get; the remaining difference is bridged by the seam.
POINTY_G = 2.5


def s_sine_to_square(u, h):
    """square at h -> 0, sine at h = 1, sharper as h grows."""
    return math.copysign(math.sin(math.pi * abs(u) ** h / 2.0), u)


def s_sine_to_triangle(u, g):
    """sine at g = 1, triangle in the limit g -> 0."""
    if g < 1e-6:
        return u
    return math.sin(g * math.pi * u / 2.0) / math.sin(g * math.pi / 2.0)


def s_triangle_to_pointy(u, g):
    """square at g -> 0, triangle at g = 1, sharper as g grows."""
    return math.copysign(abs(u) ** g, u)


def lerp(a, b, t):
    return a + (b - a) * t


def rms_of(s):
    """RMS of a rise curve, which is the RMS of the wave it generates.

    Both halves of the wave are S over u spread evenly, so the wave's RMS is the
    curve's. It runs 1.0 for a square, 0.7071 for a sine, 0.5774 for a triangle
    and lower as the peak sharpens - monotone along every family here, which is
    what makes it usable as an axis.
    """
    n = 512
    acc = 0.0
    for k in range(n):
        u = -1.0 + 2.0 * (k + 0.5) / n
        v = s(u)
        acc += v * v
    return math.sqrt(acc / n)


def solve_for_rms(family, lo, hi, target):
    """The family parameter whose curve has the given RMS.

    `lo` and `hi` must bracket it with rms(lo) > rms(hi) - the direction the
    parameter is *travelled* is the caller's business and not this function's.
    Getting that backwards silently returns a bound rather than a solution, and
    did: the return leg came out as six identical slices and one square in the
    middle of the pointy end. Hence the assert.

    Bisection rather than
    a closed form: the map from exponent to RMS is monotone but not usefully
    invertible, and this runs once at generation time.

    Spacing slices evenly in RMS rather than evenly in the exponent is what
    stops a third of the table looking identical - an exponent axis crowds all
    of its visible change into one end, which the first draft of this generator
    did, giving fifteen consecutive slices that were the same square.
    """
    r_lo, r_hi = rms_of(lambda u: family(u, lo)), rms_of(lambda u: family(u, hi))
    assert r_lo > r_hi, f"bracket is the wrong way round: rms({lo})={r_lo} !> rms({hi})={r_hi}"
    assert r_hi - 1e-9 <= target <= r_lo + 1e-9, f"target {target} outside [{r_hi}, {r_lo}]"

    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if rms_of(lambda u: family(u, mid)) > target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


RMS_SQUARE = rms_of(lambda u: s_sine_to_square(u, SQUARE_H))
RMS_SINE = math.sqrt(0.5)
RMS_TRIANGLE = math.sqrt(1.0 / 3.0)
RMS_POINTY = math.sqrt(1.0 / (2.0 * POINTY_G + 1.0))

# The h that matches the pointy extreme, so the seam bridges the smallest gap
# the two families allow.
POINTY_H = solve_for_rms(s_sine_to_square, 1.0, 12.0, RMS_POINTY)


def curve_square():
    return lambda u: s_sine_to_square(u, SQUARE_H)


def curve_pointy_g():
    return lambda u: s_triangle_to_pointy(u, POINTY_G)


def curve_pointy_h():
    return lambda u: s_sine_to_square(u, POINTY_H)


def rise_curve(i):
    """The rise curve S(u) for one slice.

    The axis is a loop, and it is built from two families that share their
    canonical shapes exactly:

        s_sine_to_square   square <- SINE -> pointy
        s_triangle_to_pointy  square <- TRIANGLE -> pointy

    Out along the first to the sine, across to the triangle on the family that
    joins them (both ends of which are exact), on to the pointy extreme, and
    home along the first family again. Every join but one is an identity - the
    two families agree exactly at the sine and at the triangle - and the one
    that is not, at the pointy end, is spread over the seam slices below.

    Going out and coming back means the return leg passes shapes resembling
    ones already seen. That is the price of a closed loop with a distinguished
    centre, and it is the right price: the encoders are endless and the
    parameter wraps, so turning SHP forever should cycle rather than stop.
    """
    if i <= SLICE_SINE:
        # square -> sine
        if i == SLICE_SINE:
            return lambda u: s_sine_to_square(u, 1.0)  # exact sine
        t = i / SLICE_SINE
        h = solve_for_rms(s_sine_to_square, SQUARE_H, 1.0, lerp(RMS_SQUARE, RMS_SINE, t))
        return lambda u: s_sine_to_square(u, h)

    if i <= SLICE_TRIANGLE:
        # sine -> triangle, on the family that has both ends exactly
        if i == SLICE_TRIANGLE:
            return lambda u: u  # exact triangle
        t = (i - SLICE_SINE) / (SLICE_TRIANGLE - SLICE_SINE)
        g = solve_for_rms(s_sine_to_triangle, 1.0, 0.0, lerp(RMS_SINE, RMS_TRIANGLE, t))
        return lambda u: s_sine_to_triangle(u, g)

    if i <= SLICE_POINTY:
        # triangle -> pointy
        t = (i - SLICE_TRIANGLE) / (SLICE_POINTY - SLICE_TRIANGLE)
        g = solve_for_rms(s_triangle_to_pointy, 1.0, POINTY_G, lerp(RMS_TRIANGLE, RMS_POINTY, t))
        return lambda u: s_triangle_to_pointy(u, g)

    if i <= SLICE_SEAM:
        # The one place the two families have to be joined rather than shared.
        # They differ by about 3% of the swing at equal RMS, which is spread
        # across these slices instead of landing in one step. Both are pointy,
        # so the blend is between two similar shapes - blending the *ends* of
        # the axis instead, which is what closing the loop naively would do,
        # gives plateaus at intermediate levels and reads as a stepped wave.
        t = (i - SLICE_POINTY) / (SLICE_SEAM - SLICE_POINTY)
        a, b = curve_pointy_g(), curve_pointy_h()
        return lambda u: lerp(a(u), b(u), t)

    # pointy -> square, closing the loop back onto slice 0.
    t = (i - SLICE_SEAM) / (WT_SLICES - SLICE_SEAM)
    h = solve_for_rms(s_sine_to_square, SQUARE_H, POINTY_H, lerp(RMS_POINTY, RMS_SQUARE, t))
    return lambda u: s_sine_to_square(u, h)


def build_slice(slice_idx):
    """One slice, with phase 0 on the rising edge.

    The family is defined with its minimum at phase 0 - that is the natural way
    to write it, and it is what makes the anchoring argument easy to state. The
    stored table is that rotated a quarter cycle, so what sits at phase 0 is the
    rising zero crossing rather than the trough.

    This is a convention, and the module already had one: SHAPE_PWM opens its
    gate at phase 0 and SHAPE_STEPPED begins step 0 at phase 0, so in both of
    those the waveform's *event* is on the beat. The wavetable was the odd one
    out, putting its trough on the beat and its rising edge a quarter of a cycle
    late - so a square channel used as a clock divider fired late, and PHS 0
    meant something different depending on which shape mode a channel was in.

    It also puts the canonical shapes in their canonical form: the sine becomes
    sin(2*pi*p) rather than -cos(2*pi*p), and the triangle rises from zero.

    A rotation preserves everything the family guarantees, because those
    guarantees are about all slices sharing the *same* anchor phases and not
    about which phases those are. WT_LEN is a multiple of four, so the rotation
    is an exact shift of whole samples with nothing resampled.
    """
    s = rise_curve(slice_idx)
    out = []
    for n in range(WT_LEN):
        p = n / WT_LEN
        if p < 0.5:
            v = s(4.0 * p - 1.0)
        else:
            v = -s(4.0 * (p - 0.5) - 1.0)
        out.append(max(-FULL, min(FULL, int(round(v * FULL)))))

    quarter = WT_LEN // 4
    return out[quarter:] + out[:quarter]


def build_table():
    return [build_slice(i) for i in range(WT_SLICES)]


HEADER = '''#ifndef INC_WAVETABLES_H_
#define INC_WAVETABLES_H_

#include <stdint.h>

// GENERATED by tools/gen_wavetable.py - do not edit. `just wavetable`.
//
// The SHAPE_LFO morphing wavetable: WT_SLICES shapes of WT_LEN samples, read
// with a bilinear lookup over phase and shape (wavetable.c).
//
// Every slice is built from an odd monotone rise curve, which is what makes
// three things true of every slice AND of every blend between neighbouring
// slices - the runtime interpolates, so a knob between two entries is the
// normal case:
//
//   * it reaches the full swing, -32767 at phase 0 and +32767 at phase 0.5;
//   * it has no DC offset, by half-wave antisymmetry;
//   * it stays inside the range.
//
// The shape axis is a loop, because SHP wraps at both the parameter and the
// lookup:
//
//   shape {sq_s:+.3f} ...... {sine_s:+.3f} ..... {tri_s:+.3f} ... {pointy_s:+.3f} ... +1.000 (= {sq_s:+.3f})
//   square          SINE       TRIANGLE    pointy       square
//
// SINE is at shape 0, a channel's default, and is exact. So is TRIANGLE. See
// the generator for why saw, stepped and pulse shapes are deliberately absent.

#define WT_LEN {len}
#define WT_SLICES {slices}

// Where the named shapes are, for tests and for anyone reading a value back.
#define WT_SLICE_SQUARE {sq}
#define WT_SLICE_SINE {sine}
#define WT_SLICE_TRIANGLE {tri}
#define WT_SLICE_POINTY {pointy}

// Declared, not defined: the table lives in wavetables.c so that including this
// from a second translation unit links. It used to be defined here, which was
// invisible while exactly one file included it and a multiple-definition error
// the moment a test did.
extern const int16_t shape_table[WT_SLICES][WT_LEN];

#endif /* INC_WAVETABLES_H_ */
'''

SOURCE = '''// GENERATED by tools/gen_wavetable.py - do not edit. `just wavetable`.
//
// The data for wavetables.h. See the generator for how it is built and why.

#include "wavetables.h"

const int16_t shape_table[WT_SLICES][WT_LEN] = {
'''


def emit_source(table):
    out = SOURCE
    for sl in table:
        out += "    {"
        for n, v in enumerate(sl):
            if n % 16 == 0 and n:
                out = out.rstrip(" ") + "\n     "
            out += f"{v}, " if n < WT_LEN - 1 else f"{v}"
        out += "},\n"
    out += "};\n"
    return out


def emit_header():
    def shape_of(i):
        return 2.0 * i / WT_SLICES - 1.0

    return HEADER.format(
        len=WT_LEN,
        slices=WT_SLICES,
        sq=SLICE_SQUARE,
        sine=SLICE_SINE,
        tri=SLICE_TRIANGLE,
        pointy=SLICE_POINTY,
        sq_s=shape_of(SLICE_SQUARE),
        sine_s=shape_of(SLICE_SINE),
        tri_s=shape_of(SLICE_TRIANGLE),
        pointy_s=shape_of(SLICE_POINTY),
    )


# ---------------------------------------------------------------------------
# Pictures for the documentation
#
# Drawn from the same build_table() the header is written from, so a plot can
# never show a shape the firmware does not have. SVG because it is text - a
# reviewer sees a shape change as a diff rather than as a new binary - and
# because it costs nothing to render at any size.
#
# The colours are chosen to read on a white page and on a dark one, since that
# is the same file either way; viewers that honour prefers-color-scheme get a
# better pair, and the ones that do not still get a legible picture.

SVG_STYLE = """  <style>
    .frame { fill: none; stroke: #b6bec9; stroke-width: 1; }
    .axis  { stroke: #b6bec9; stroke-width: 1; stroke-dasharray: 3 3; }
    .trace { fill: none; stroke: #2f7fd4; stroke-width: 2.5;
             stroke-linejoin: round; stroke-linecap: round; }
    .name  { fill: #24292f; font: 600 13px -apple-system, system-ui, sans-serif; }
    .sub   { fill: #6b7480; font: 400 11px -apple-system, system-ui, sans-serif; }
    .key   { stroke: #2f7fd4; stroke-width: 1.75; }
    .key-label { fill: #2f7fd4; font-weight: 600; }
    @media (prefers-color-scheme: dark) {
      .frame, .axis { stroke: #464f5b; }
      .trace { stroke: #59a5f5; }
      .name  { fill: #e6edf3; }
      .sub   { fill: #98a1ab; }
      .key   { stroke: #59a5f5; }
      .key-label { fill: #59a5f5; }
    }
  </style>
"""


def trace_path(samples, x, y, w, h, points):
    """One panel's worth of waveform as an SVG polyline `points` string.

    Decimated to `points`, which is about the panel's width in pixels - a plot
    cannot show more than that and every extra pair is bytes in a file someone
    has to review. The full table stays 256 samples; this is only the picture.
    """
    pts = []
    for k in range(points + 1):
        src = round(k * len(samples) / points) % len(samples)
        v = samples[src] / FULL
        px = x + w * k / points
        py = y + h / 2.0 - v * (h / 2.0)
        pts.append(f"{px:.1f},{py:.1f}")
    return " ".join(pts)


def svg_panel(samples, x, y, w, h, name, sub, points=128, key=False):
    frame_cls = "frame key" if key else "frame"
    sub_cls = "sub key-label" if key else "sub"
    out = f'  <rect class="{frame_cls}" x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="3"/>\n'
    out += f'  <line class="axis" x1="{x:.1f}" y1="{y + h / 2:.1f}" x2="{x + w:.1f}" y2="{y + h / 2:.1f}"/>\n'
    out += f'  <polyline class="trace" points="{trace_path(samples, x, y, w, h, points)}"/>\n'
    if name:
        out += f'  <text class="name" x="{x:.1f}" y="{y - 14:.1f}">{name}</text>\n'
    if sub:
        out += f'  <text class="{sub_cls}" x="{x:.1f}" y="{y - 2:.1f}">{sub}</text>\n'
    return out


def svg_keyframes(table):
    """The named shapes, side by side, at a size worth looking at."""
    keys = [
        (SLICE_SQUARE, "square", "SHP -1.000 / +1.000"),
        (SLICE_SINE, "sine", "SHP 0.000 - the default"),
        (SLICE_TRIANGLE, "triangle", "SHP +0.375"),
        (SLICE_POINTY, "pointy", "SHP +0.625"),
    ]
    pw, ph, gap, pad = 190, 110, 26, 16
    top = 46
    width = pad * 2 + pw * len(keys) + gap * (len(keys) - 1)
    height = top + ph + pad

    out = f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">\n'
    out += SVG_STYLE
    for i, (idx, name, sub) in enumerate(keys):
        x = pad + i * (pw + gap)
        out += svg_panel(table[idx], x, top, pw, ph, name, sub, points=192)
    out += "</svg>\n"
    return out


KEYFRAMES = {}  # filled below, once the slice constants are known


def keyframe_names():
    return {
        SLICE_SQUARE: "square",
        SLICE_SINE: "sine",
        SLICE_TRIANGLE: "triangle",
        SLICE_POINTY: "pointy",
    }


def axis_sample_indices(frames):
    """Which slices the sweep strip shows.

    An even grid of `frames` slices, **unioned with the keyframes**. The union
    is the point: the named shapes are the ones a reader is looking for, and
    whether an even grid happens to land on them is an accident of where they
    sit. They all fall on multiples of four today, so a grid of 16 or 32 catches
    them by luck - move SLICE_TRIANGLE by one and the strip would quietly stop
    showing the triangle.
    """
    step = max(1, WT_SLICES // frames)
    idxs = set(range(0, WT_SLICES, step)) | set(keyframe_names())
    return sorted(idxs)


def svg_axis(table, frames=32):
    """The whole shape axis, in reading order.

    The loop is the point: the last frame is a neighbour of the first, so the
    strip is meant to be read as a ring rather than as a line.
    """
    idxs = axis_sample_indices(frames)
    keys = keyframe_names()

    cols = 8
    rows = (len(idxs) + cols - 1) // cols
    pw, ph, gx, gy, pad = 96, 56, 12, 28, 14
    top = 20

    width = pad * 2 + cols * pw + (cols - 1) * gx
    height = top + rows * (ph + gy) + pad

    out = f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">\n'
    out += SVG_STYLE
    for k, idx in enumerate(idxs):
        r, c = divmod(k, cols)
        x = pad + c * (pw + gx)
        y = top + r * (ph + gy)
        shape = 2.0 * idx / WT_SLICES - 1.0
        name = keys.get(idx)
        sub = f"{shape:+.3f}" + (f"  {name}" if name else "")
        out += svg_panel(table[idx], x, y, pw, ph, None, sub, points=64, key=name is not None)
    out += "</svg>\n"
    return out


def report(table):
    ch = " .:-=+*#%@"

    def spark(w, cols=64):
        return "".join(
            ch[min(9, max(0, int(((w[int(k * (WT_LEN - 1) / (cols - 1))] / FULL) + 1) / 2 * 9.99)))]
            for k in range(cols)
        )

    print(f"{'i':>3} {'shape':>6}  {'wave':<64} {'p-p':>5} {'dc':>7}")
    for i, sl in enumerate(table):
        pp = (max(sl) - min(sl)) / FULL
        dc = sum(sl) / len(sl) / FULL
        name = ""
        if i == SLICE_SQUARE:
            name = " <- square"
        if i == SLICE_SINE:
            name = " <- SINE (shape 0)"
        if i == SLICE_TRIANGLE:
            name = " <- TRIANGLE"
        if i == SLICE_POINTY:
            name = " <- pointy"
        print(f"{i:>3} {2*i/WT_SLICES-1:+6.3f}  {spark(sl):<64} {pp:5.3f} {dc:+7.4f}{name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", action="store_true", help="print the table instead of writing it")
    args = ap.parse_args()

    table = build_table()

    if args.report:
        report(table)
        return

    core = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Core")
    h = os.path.normpath(os.path.join(core, "Inc", "Lib", "wavetables.h"))
    c = os.path.normpath(os.path.join(core, "Src", "Lib", "wavetables.c"))

    with open(h, "w") as f:
        f.write(emit_header())
    with open(c, "w") as f:
        f.write(emit_source(table))

    print(f"wrote {h}")
    print(f"wrote {c}: {WT_SLICES} slices x {WT_LEN} samples ({WT_SLICES * WT_LEN * 2} bytes)")

    # The documentation pictures come out of the same run, so there is no state
    # in which the header and the plots disagree about what a shape looks like.
    images = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "images"))
    os.makedirs(images, exist_ok=True)

    for fname, body in (
        ("wavetable-keyframes.svg", svg_keyframes(table)),
        ("wavetable-axis.svg", svg_axis(table)),
    ):
        path = os.path.join(images, fname)
        with open(path, "w") as f:
            f.write(body)
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
