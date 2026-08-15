"""Cross-check the shipping C against final.py, the spec it was written from.

Parses the generated normalisation table out of the header so both sides use
the same binned correction, leaving only the value path itself under test.
"""
import re
import subprocess
import sys
import numpy as np
import srmodel as M
import final as F

ROOT = "/home/jan/workspace/nvim/bmcv"
HDR = f"{ROOT}/Core/Inc/Lib/stepped_random_table.h"

SHAPES = [-1.0, -0.6, -0.13, 0.0, 0.31, 0.7, 1.0]
MODS = [-1.0, -0.4, 0.0, 0.25, 0.8, 1.0]


def parse_table(name, src):
    i = src.index(f"{name}[SR_LENGTH_COUNT]")
    body = src[i:src.index("};", i)]
    rows = re.findall(r"\{([-0-9.eEf,\s]+)\},", body)
    vals = [[float(x.rstrip("f")) for x in r.replace("\n", "").split(",") if x.strip()]
            for r in rows]
    a = np.array(vals)
    return a.reshape(len(M.SR_LENGTHS), F.MOD_BINS, F.NORM_BINS)


def main():
    src = open(HDR).read()
    G, O = parse_table("sr_norm_gain", src), parse_table("sr_norm_offset", src)

    def lookup(T, li, mod, morph):
        bp = morph * F.NORM_BINS
        bi = int(bp)
        bf = bp - bi
        b = bi % F.NORM_BINS
        bn = (b + 1) % F.NORM_BINS
        mp = (min(max(mod, -1.0), 1.0) + 1.0) * 0.5 * (F.MOD_BINS - 1)
        mb = int(mp)
        mf = mp - mb
        if mb >= F.MOD_BINS - 1:
            mb, mf = F.MOD_BINS - 2, 1.0
        lo = T[li, mb, b] * (1 - bf) + T[li, mb, bn] * bf
        hi = T[li, mb + 1, b] * (1 - bf) + T[li, mb + 1, bn] * bf
        return lo * (1 - mf) + hi * mf

    e = F.Final()
    out = []
    for s in SHAPES:
        for m in MODS:
            for li in range(len(M.SR_LENGTHS)):
                raw = e.render(s, m, li, n=32, hold=0.0, normalise=False)
                # the table's axis is the orbit, which the value path is
                # periodic in - not SHP's own morph
                morph = F.orbit_of(s, m) % 1.0
                out.append(np.clip(raw * lookup(G, li, m, morph) + lookup(O, li, m, morph), -1, 1))
    py = np.concatenate(out)

    ref = np.loadtxt(sys.argv[1])
    d = np.abs(py - ref)
    print(f"samples {len(py)}  max |C - spec| {d.max():.3e}  mean {d.mean():.3e}")
    worst = int(np.argmax(d))
    print(f"worst at index {worst}: C {ref[worst]:.6f} vs spec {py[worst]:.6f}")
    return 0 if d.max() < 2e-4 else 1


if __name__ == "__main__":
    sys.exit(main())
