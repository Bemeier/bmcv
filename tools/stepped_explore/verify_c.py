"""Cross-check the shipping C against final.py, the spec it was written from.

The correction is no longer a table to parse: the module measures its own
pattern and reads only the centring constant out of the header. So this repeats
that arithmetic - it is a dozen lines - and what stays under test is the value
path, which is the part a spec is worth having for.
"""
import re
import subprocess
import sys
import numpy as np
import stmodel as M
import final as F

ROOT = "/home/jan/workspace/nvim/bmcv"
HDR = f"{ROOT}/Core/Inc/Lib/stepped_table.h"

SHAPES = [-1.0, -0.6, -0.13, 0.0, 0.31, 0.7, 1.0]
MODS = [-1.0, -0.4, 0.0, 0.25, 0.8, 1.0]


def parse_table(name, src):
    i = src.index(f"{name}[ST_MOD_BINS]")
    body = src[i:src.index("};", i)]
    rows = re.findall(r"\{([-0-9.eEf,\s]+)\},", body)
    return np.array([[float(x.rstrip("f")) for x in r.replace("\n", "").split(",") if x.strip()]
                     for r in rows])


# stepped_norm.h
NORM_TARGET, NORM_EXP, NORM_FLOOR = 1.5, 0.7, 0.9
MIN_GAIN, MAX_GAIN = 0.4, 40.0


def gain_toward(target, expo, lo, hi, anchor, centre):
    span = hi - lo
    g = MAX_GAIN if span < 1e-6 else min(max((target / span) ** expo, MIN_GAIN), MAX_GAIN)
    if hi > anchor:
        g = min(g, (1.0 - centre) / (hi - anchor))
    if lo < anchor:
        g = min(g, (1.0 + centre) / (anchor - lo))
    return max(g, 0.05)


def correction(vals, centre):
    """st_norm_at(): the gain the pattern's own extremes ask for, floored, as an
    affine about slot 0."""
    lo, hi, anchor = vals.min(), vals.max(), vals[0]
    g = max(gain_toward(NORM_TARGET, NORM_EXP, lo, hi, anchor, centre),
            gain_toward(NORM_FLOOR, 1.0, lo, hi, anchor, centre))
    return g, centre - anchor * g


def main():
    src = open(HDR).read()
    C = parse_table("st_centre_table", src)

    def centre_at(mod, morph):
        bp = morph * F.NORM_BINS
        b = int(bp) % F.NORM_BINS
        bf = bp - int(bp)
        bn = (b + 1) % F.NORM_BINS
        mp = (min(max(mod, -1.0), 1.0) + 1.0) * 0.5 * (F.MOD_BINS - 1)
        mb = int(mp)
        mf = mp - mb
        if mb >= F.MOD_BINS - 1:
            mb, mf = F.MOD_BINS - 2, 1.0
        lo = C[mb, b] * (1 - bf) + C[mb, bn] * bf
        hi = C[mb + 1, b] * (1 - bf) + C[mb + 1, bn] * bf
        return lo * (1 - mf) + hi * mf

    e = F.Final()
    out = []
    for s in SHAPES:
        for m in MODS:
            for li in range(len(M.ST_LENGTHS)):
                raw = e.render(s, m, li, n=32, hold=0.0, normalise=False)
                # the axis is the orbit, which the value path is periodic in -
                # not SHP's own morph
                morph = F.orbit_of(s, m) % 1.0
                vals, _, _ = e.steps(s, m, M.ST_LENGTHS[li])
                g, off = correction(vals, centre_at(m, morph))
                out.append(np.clip(raw * g + off, -1, 1))
    py = np.concatenate(out)

    ref = np.loadtxt(sys.argv[1])
    d = np.abs(py - ref)
    print(f"samples {len(py)}  max |C - spec| {d.max():.3e}  mean {d.mean():.3e}")
    worst = int(np.argmax(d))
    print(f"worst at index {worst}: C {ref[worst]:.6f} vs spec {py[worst]:.6f}")
    return 0 if d.max() < 2e-4 else 1


if __name__ == "__main__":
    sys.exit(main())
