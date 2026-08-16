"""What the knob is actually short of.

The pattern-identity numbers say the baseline sweep contains hundreds of
different note sequences. It still feels like it does nothing, and the reason
is visible in the same numbers: every one of those sequences has the same
*character*. Uniformly spread values, the same jaggedness, the same density,
the same span - the sweep draws a fresh sample from one fixed distribution over
and over. Random, then random, then random, reads as "no change".

So the figure of merit is coverage of character space: how many qualitatively
different behaviours the knob reaches, under the same constraint as before that
neighbouring positions stay related.
"""
import numpy as np
import stmodel as M

SWEEP = 400
SMALL_MOVE = 0.02

FEATURES = ["span", "rough", "turns", "edginess", "tie_frac", "maxrun",
            "motif", "centroid", "lowfreq", "spread", "swing"]

# The baseline has no timing variation at all, so its sigma for `swing` is zero
# and cannot set the scale. Pin it: 0.10 of relative step-width spread is about
# where swing stops being subtle.
FIXED_SCALE = {"swing": 0.10}


def features(v, tied, widths=None):
    """Character of one pattern: v = value shown at each step, tied = mask."""
    L = len(v)
    span = v.max() - v.min()
    d = np.abs(np.diff(np.concatenate([v, v[:1]])))
    rough = d.mean() / max(span, 1e-6)
    dv = np.diff(np.concatenate([v, v[:2]]))
    turns = float((dv[:-1] * dv[1:] < 0).mean())
    edginess = float((np.abs(v - v.mean()) > 0.5 * span * 0.5).mean())
    tie_frac = float(tied.mean())
    runs, cur = [], 1
    for i in range(1, L):
        if abs(v[i] - v[i - 1]) < 1e-6:
            cur += 1
        else:
            runs.append(cur)
            cur = 1
    runs.append(cur)
    maxrun = max(runs) / L
    # strongest sub-cycle repetition: does the pattern contain a riff?
    x = v - v.mean()
    denom = max((x * x).sum(), 1e-9)
    motif = 0.0
    for p in range(2, L // 2 + 1):
        if L % p == 0:
            motif = max(motif, float((x * np.roll(x, p)).sum() / denom))
    F = np.abs(np.fft.rfft(x))
    lowfreq = float(F[1:3].sum() / max(F[1:].sum(), 1e-9))
    swing = 0.0 if widths is None else float(widths.std() / widths.mean())
    return np.array([span, rough, turns, edginess, tie_frac, maxrun, motif,
                     v.mean(), lowfreq, v.std(), swing])


def feat_at(eng, s, m, li):
    L = M.ST_LENGTHS[li]
    vals, src, jumps = eng.steps(s, m, L)
    g, off = eng.norm_for(s, m, li)
    return features(np.clip(vals * g + off, -1.0, 1.0), ~jumps,
                    eng.step_widths(s, m, L))


def feat_sweep(eng, axis, fixed, li, n=SWEEP):
    pos = np.linspace(-1.0, 1.0, n, endpoint=False)
    return pos, np.array([feat_at(eng, *((p, fixed) if axis == "shape" else (fixed, p)), li)
                          for p in pos])


def feat_plane(eng, li, n=48):
    g = np.linspace(-1.0, 1.0, n, endpoint=False)
    return np.array([feat_at(eng, s, m, li) for s in g for m in g])


# Standardisation: one fixed scale, taken from the baseline engine, so a
# variant that widens a feature shows up as a bigger number instead of being
# renormalised back to 1.
_SCALE = None


def scale(li=8):
    global _SCALE
    if _SCALE is None:
        F = feat_plane(M.Engine(), li, n=32)
        _SCALE = np.maximum(F.std(0), 1e-3)
        for k, v in FIXED_SCALE.items():
            _SCALE[FEATURES.index(k)] = v
    return _SCALE


def smooth_cyclic(Z, frac=0.10):
    """Low-pass the character trajectory along the knob. What survives is the
    character the knob *steers*; what it removes is the character that merely
    fluctuates because each position draws a fresh random pattern."""
    n = len(Z)
    w = max(3, int(n * frac) | 1)
    k = np.hanning(w)
    k /= k.sum()
    out = np.empty_like(Z)
    for j in range(Z.shape[1]):
        pad = np.concatenate([Z[-w:, j], Z[:, j], Z[:w, j]])
        out[:, j] = np.convolve(pad, k, mode="same")[w:w + n]
    return out


def packing(P, eps):
    chosen = [P[0]]
    for p in P[1:]:
        if all(np.linalg.norm(p - c) >= eps for c in chosen):
            chosen.append(p)
    return len(chosen)


def axis_report(eng, axis, fixed, li):
    pos, F = feat_sweep(eng, axis, fixed, li)
    Z = F / scale()
    S = smooth_cyclic(Z)
    stride = max(1, int(round(SMALL_MOVE / (2.0 / len(pos)))))
    local = np.linalg.norm(Z - np.roll(Z, -stride, axis=0), axis=1)
    drift = np.percentile(S, 95, axis=0) - np.percentile(S, 5, axis=0)
    jitter = float(np.linalg.norm(Z - S, axis=1).mean())
    return {
        "steer": float(drift.mean()),          # character the knob commands
        "jitter": jitter,                      # character that merely fluctuates
        "ratio": float(drift.mean()) / max(jitter, 1e-6),
        "chars": packing(S, 1.0),              # distinct characters dialable
        "local": float(local.mean()),
        "still": float((local < 0.02).mean()),
        "drift": drift,
    }


def plane_report(eng, li, n=48):
    F = feat_plane(eng, li, n)
    Z = F / scale()
    reach = np.percentile(F, 95, axis=0) - np.percentile(F, 5, axis=0)
    return {"chars": packing(Z, 1.0), "coverage": float((reach / scale()).mean()), "reach": reach}


HDR = "                steer  jitter  ratio  chars  local  still"


def line(tag, r):
    print(f"{tag:15s} {r['steer']:5.2f}   {r['jitter']:5.2f}  {r['ratio']:5.2f}  "
          f"{r['chars']:4d}   {r['local']:5.2f}  {r['still']:.2f}")


def report(eng, lengths=(6, 8), mods=(0.0, -0.5), shapes=(0.0, 0.4), plane=True, detail=False):
    print(f"=== {eng.name} (character) ===")
    print(HDR)
    for li in lengths:
        L = M.ST_LENGTHS[li]
        for m in mods:
            line(f"SHP L={L} m={m:+.1f}", axis_report(eng, "shape", m, li))
        for s in shapes:
            line(f"MOD L={L} s={s:+.1f}", axis_report(eng, "mod", s, li))
        if plane:
            p = plane_report(eng, li)
            print(f"{'PLANE L=' + str(L):15s} {p['chars']:5d}   {p['coverage']:6.2f}")
    if detail:
        for li in lengths:
            for ax, fx in [("shape", 0.0), ("mod", 0.0)]:
                r = axis_report(eng, ax, fx, li)
                print(f"  steered character, {ax.upper()} L={M.ST_LENGTHS[li]} (sigma):")
                print("    " + "  ".join(f"{n}={d:.2f}" for n, d in zip(FEATURES, r["drift"])))
    print()
