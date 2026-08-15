"""The musical yardstick.

RMS distance between two rendered cycles answers "how different is this signal",
which is not the question. Turning every step of a 32-step pattern by 5% of the
range is a large RMS move and sounds like the same pattern slightly detuned;
changing one step outright is a smaller RMS move and sounds like a new note.
What the ear counts is *how many steps changed*, and it stops counting once a
step has moved far enough to read as a different note.

So: quantise the pattern to note-sized levels and compare sequences by Hamming
distance. A sweep's worth is then two numbers - how many mutually distinct
sequences it contains, and how many notes change per 1% turn of the knob.
"""
import numpy as np
import srmodel as M

LEVELS = 12          # quantiser steps over the [-1,1] output: "different note"
SWEEP = 400
SMALL_MOVE = 0.02    # 1% of knob travel


def quantise(p):
    return np.clip(((p + 1.0) * 0.5 * LEVELS).astype(int), 0, LEVELS - 1)


def seq_sweep(eng, axis, fixed, length_idx, n=SWEEP):
    pos = np.linspace(-1.0, 1.0, n, endpoint=False)
    L = M.SR_LENGTHS[length_idx]
    Q = np.empty((n, L), dtype=int)
    for i, p in enumerate(pos):
        s, m = (p, fixed) if axis == "shape" else (fixed, p)
        Q[i] = quantise(eng.pattern(s, m, length_idx))
    return pos, Q


def hamming(Q):
    """Fraction of steps that differ, for every pair of sequences."""
    n, L = Q.shape
    return (Q[:, None, :] != Q[None, :, :]).sum(2) / L


def packing(D, eps):
    chosen = [0]
    for i in range(1, D.shape[0]):
        if all(D[i, c] >= eps for c in chosen):
            chosen.append(i)
    return len(chosen)


def axis_report(eng, axis, fixed, length_idx):
    pos, Q = seq_sweep(eng, axis, fixed, length_idx)
    L = Q.shape[1]
    D = hamming(Q)
    # adjacent-position change, expressed in steps-changed per 1% of travel
    stride = max(1, int(round(SMALL_MOVE / (2.0 / len(pos)))))
    adj = np.array([D[i, (i + stride) % len(pos)] for i in range(len(pos))]) * L
    uniq = len(np.unique(Q, axis=0))
    return {
        "unique": uniq,
        "distinct_33": packing(D, 1.0 / 3.0),
        "distinct_50": packing(D, 0.5),
        "notes_per_1pct": adj.mean(),
        "notes_p95": np.percentile(adj, 95),
        "still_frac": float((adj == 0).mean()),
        "reach": D.max(),
    }


def plane_report(eng, length_idx, n=64):
    grid = np.linspace(-1.0, 1.0, n, endpoint=False)
    L = M.SR_LENGTHS[length_idx]
    Q = np.empty((n * n, L), dtype=int)
    k = 0
    for s in grid:
        for m in grid:
            Q[k] = quantise(eng.pattern(s, m, length_idx))
            k += 1
    D = hamming(Q)
    return {"unique": len(np.unique(Q, axis=0)), "distinct_33": packing(D, 1.0 / 3.0),
            "distinct_50": packing(D, 0.5), "reach": D.max()}


HDR = "                unique  d33   d50   notes/1%  p95   still  reach"


def line(tag, r):
    print(f"{tag:15s} {r['unique']:5d} {r['distinct_33']:5d} {r['distinct_50']:5d}   "
          f"{r['notes_per_1pct']:6.2f} {r['notes_p95']:5.1f}   {r['still_frac']:.2f}  {r['reach']:.2f}")


def report(eng, lengths=(6, 8), mods=(0.0, -0.5), shapes=(0.0, 0.4), plane=True):
    print(f"=== {eng.name} (musical) ===")
    print(HDR)
    for li in lengths:
        L = M.SR_LENGTHS[li]
        for m in mods:
            line(f"SHP L={L} m={m:+.1f}", axis_report(eng, "shape", m, li))
        for s in shapes:
            line(f"MOD L={L} s={s:+.1f}", axis_report(eng, "mod", s, li))
        if plane:
            p = plane_report(eng, li)
            print(f"{'PLANE L=' + str(L):15s} {p['unique']:5d} {p['distinct_33']:5d} "
                  f"{p['distinct_50']:5d}                          {p['reach']:.2f}")
    print()
