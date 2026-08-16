"""character.py's numbers, computed on the C's own output, plus level.

Fed by dump_pattern.c rather than by srmodel.py, because the question this was
built for is a change to the *correction*, and the correction lives in the
generated table - a Python re-derivation of it measures a module that does not
exist.

Two figures beyond character.py's:

- **dc swing** - how far the pattern's centre walks along one sweep of a knob.
  Level is what AMP and OFFSET are for; a knob that moves it is a knob spending
  its travel on the wrong thing.
- **span lo..hi** - the same for peak-to-peak.

And `char-st`: steer over the eight features that are *not* level. Three of the
eleven - span, spread, centroid - are level, so flattening them lowers `steer`
by construction. char-st is what says whether the knob still goes anywhere, and
it is invariant under the affine correction, so it should not move at all when
only the table changes.

    cc -O2 -I Core/Inc/Lib -o /tmp/dump_pat tools/sr_explore/dump_pattern.c -lm
    /tmp/dump_pat > /tmp/after.txt
    python3 tools/sr_explore/from_c.py /tmp/before.txt /tmp/after.txt

The first file given sets the standardisation scale, so every column is
comparable across the runs in one invocation.
"""
import sys
import numpy as np

FEATURES = ["span", "rough", "turns", "edginess", "tie_frac", "maxrun",
            "motif", "centroid", "lowfreq", "spread", "swing"]
LEVEL = ("span", "spread", "centroid")
SR_LENGTHS = [3, 4, 5, 6, 8, 12, 16, 24, 32, 48, 64]
SMALL_MOVE = 0.02


def load(path):
    rows = []
    for line in open(path):
        head, v, w, wd = line.split(":")
        li, axis, fixed, pos = head.split()
        rows.append((int(li), axis, float(fixed), float(pos),
                     np.array([float(x) for x in v.split()]),
                     np.array([float(x) for x in w.split()]),
                     np.array([float(x) for x in wd.split()])))
    return rows


def features(v, tied, widths):
    """character.py's features, on the value shown at each step."""
    n = len(v)
    span = v.max() - v.min()
    d = np.abs(np.diff(np.concatenate([v, v[:1]])))
    rough = d.mean() / max(span, 1e-6)
    dv = np.diff(np.concatenate([v, v[:2]]))
    turns = float((dv[:-1] * dv[1:] < 0).mean())
    edginess = float((np.abs(v - v.mean()) > 0.5 * span * 0.5).mean())
    runs, cur = [], 1
    for i in range(1, n):
        if abs(v[i] - v[i - 1]) < 1e-6:
            cur += 1
        else:
            runs.append(cur)
            cur = 1
    runs.append(cur)
    x = v - v.mean()
    denom = max((x * x).sum(), 1e-9)
    motif = 0.0
    for p in range(2, n // 2 + 1):
        if n % p == 0:
            motif = max(motif, float((x * np.roll(x, p)).sum() / denom))
    F = np.abs(np.fft.rfft(x))
    return np.array([span, rough, turns, edginess, float(tied.mean()),
                     max(runs) / n, motif, v.mean(),
                     float(F[1:3].sum() / max(F[1:].sum(), 1e-9)), v.std(),
                     float(widths.std() / widths.mean())])


def feats(rows):
    return np.array([features(r[4], r[5] < 0.5, r[6]) for r in rows])


_SCALE = None


def scale(rows=None):
    """One fixed scale, from the first run given, so a variant that widens a
    feature shows a bigger number instead of being renormalised back to 1."""
    global _SCALE
    if _SCALE is None:
        F = feats([r for r in rows if r[0] == SR_LENGTHS.index(32)])
        _SCALE = np.maximum(F.std(0), 1e-3)
        _SCALE[FEATURES.index("swing")] = 0.10  # the swing-free baseline cannot set its own
    return _SCALE


def smooth_cyclic(Z, frac=0.10):
    n = len(Z)
    w = max(3, int(n * frac) | 1)
    k = np.hanning(w)
    k /= k.sum()
    out = np.empty_like(Z)
    for j in range(Z.shape[1]):
        pad = np.concatenate([Z[-w:, j], Z[:, j], Z[:w, j]])
        out[:, j] = np.convolve(pad, k, mode="same")[w:w + n]
    return out


def packing(P, eps=1.0):
    chosen = [P[0]]
    for p in P[1:]:
        if all(np.linalg.norm(p - c) >= eps for c in chosen):
            chosen.append(p)
    return len(chosen)


def axis_report(rows):
    F = feats(rows)
    Z = F / scale()
    S = smooth_cyclic(Z)
    stride = max(1, int(round(SMALL_MOVE / (2.0 / len(rows)))))
    local = np.linalg.norm(Z - np.roll(Z, -stride, axis=0), axis=1)
    drift = np.percentile(S, 95, axis=0) - np.percentile(S, 5, axis=0)
    char = [i for i, n in enumerate(FEATURES) if n not in LEVEL]
    span, dc = F[:, 0], F[:, FEATURES.index("centroid")]
    return dict(steer=float(drift.mean()), steer_c=float(drift[char].mean()),
                jitter=float(np.linalg.norm(Z - S, axis=1).mean()),
                chars=packing(S), local=float(local.mean()),
                span_lo=float(span.min()), span_hi=float(span.max()),
                dc_swing=float(dc.max() - dc.min()), dc_abs=float(np.abs(dc).max()))


HDR = "                  steer char-st  jit chars local |  span lo..hi ratio | dc swing |dc|max"


def report(path, lengths=(8, 16, 32), axes=(("shape", -0.5), ("shape", 0.0), ("mod", 0.0), ("mod", 0.4))):
    rows = load(path)
    scale(rows)
    print(f"=== {path} ===")
    print(HDR)
    agg = []
    for length in lengths:
        li = SR_LENGTHS.index(length)
        for axis, fixed in axes:
            g = [r for r in rows if r[0] == li and r[1] == axis and abs(r[2] - fixed) < 1e-6]
            if not g:
                continue
            r = axis_report(g)
            agg.append(r)
            line(f"{axis.upper():5s} L={length:<2d} f={fixed:+.1f}", r)
    line("MEAN", {k: float(np.mean([a[k] for a in agg])) for k in agg[0]})
    print()


def line(tag, r):
    print(f"{tag:17s} {r['steer']:5.2f} {r['steer_c']:6.2f} {r['jitter']:5.2f} {r['chars']:5.0f} "
          f"{r['local']:5.2f} | {r['span_lo']:5.2f}..{r['span_hi']:4.2f} "
          f"{r['span_hi'] / max(r['span_lo'], 1e-9):5.2f} | {r['dc_swing']:8.2f} {r['dc_abs']:6.2f}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        report(p)
