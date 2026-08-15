"""How much does a knob actually do?

The figure of merit is the number of mutually distinct patterns a sweep offers,
measured under the constraint that neighbouring knob positions still sound
related. Both halves matter: re-randomising every 1% of travel would score well
on diversity and be useless to play.
"""
import numpy as np
import srmodel as M

N_SAMPLES = 512          # samples per rendered cycle
SWEEP = 400              # knob positions per sweep
SMALL_MOVE = 0.02        # knob units counted as "a small turn" (1% of travel)


def render_sweep(eng, axis, fixed, length_idx, n=SWEEP):
    """Render one cycle at each of n positions along `axis` ('shape'|'mod')."""
    pos = np.linspace(-1.0, 1.0, n, endpoint=False)
    out = np.empty((n, N_SAMPLES))
    for i, p in enumerate(pos):
        s, m = (p, fixed) if axis == "shape" else (fixed, p)
        out[i] = eng.render(s, m, length_idx, n=N_SAMPLES)
    return pos, out


def dist_matrix(P):
    """RMS distance between every pair of rendered cycles."""
    sq = (P * P).sum(1)
    d2 = sq[:, None] + sq[None, :] - 2.0 * (P @ P.T)
    return np.sqrt(np.maximum(d2, 0.0) / P.shape[1])


def ref_distance(P):
    """Distance between two unrelated patterns from this family: the scale on
    which everything else is read. 1.0 means 'sounds like a different pattern'."""
    D = dist_matrix(P)
    return np.percentile(D[np.triu_indices_from(D, 1)], 90)


def packing(D, eps):
    """Greedy count of positions that are pairwise >= eps apart: how many
    genuinely distinct patterns the sweep contains."""
    chosen = [0]
    for i in range(1, D.shape[0]):
        if all(D[i, c] >= eps for c in chosen):
            chosen.append(i)
    return len(chosen)


def _wrap(v):
    return ((v + 1.0) % 2.0) - 1.0


def local_step(eng, axis, fixed, length_idx, delta=SMALL_MOVE, n=200):
    """Distance travelled by a small turn of the knob, at n places on the dial."""
    pos = np.linspace(-1.0, 1.0, n, endpoint=False)
    out = np.empty(n)
    for i, p in enumerate(pos):
        a, b = p, _wrap(p + delta)
        pa = (a, fixed) if axis == "shape" else (fixed, a)
        pb = (b, fixed) if axis == "shape" else (fixed, b)
        A = eng.render(pa[0], pa[1], length_idx, n=N_SAMPLES)
        B = eng.render(pb[0], pb[1], length_idx, n=N_SAMPLES)
        out[i] = np.sqrt(((A - B) ** 2).mean())
    return out


def eff_dim(P):
    """Participation ratio of the sweep's covariance spectrum: how many
    independent degrees of freedom the knob exercises."""
    X = P - P.mean(0)
    lam = np.linalg.svd(X, compute_uv=False) ** 2
    return (lam.sum() ** 2) / (lam ** 2).sum()


def spans(eng, axis, fixed, length_idx, n=200):
    pos = np.linspace(-1.0, 1.0, n, endpoint=False)
    out = np.empty(n)
    for i, p in enumerate(pos):
        s, m = (p, fixed) if axis == "shape" else (fixed, p)
        c = eng.render(s, m, length_idx, n=N_SAMPLES)
        out[i] = c.max() - c.min()
    return out


def axis_report(eng, axis, fixed, length_idx):
    pos, P = render_sweep(eng, axis, fixed, length_idx)
    D = dist_matrix(P)
    ref = ref_distance(P)
    ls = local_step(eng, axis, fixed, length_idx)
    sp = spans(eng, axis, fixed, length_idx)
    return {
        "ref": ref,
        "distinct_50": packing(D, 0.50 * ref),
        "distinct_75": packing(D, 0.75 * ref),
        "local_med": np.median(ls) / ref,
        "local_p95": np.percentile(ls, 95) / ref,
        "dead_frac": float((ls < 0.15 * np.median(ls)).mean()),
        "eff_dim": eff_dim(P),
        "span_min": sp.min(),
        "span_med": np.median(sp),
    }


def plane_report(eng, length_idx, n=48):
    """The (shape, mod) plane as a whole: distinct patterns reachable in total."""
    grid = np.linspace(-1.0, 1.0, n, endpoint=False)
    P = np.empty((n * n, N_SAMPLES))
    k = 0
    for s in grid:
        for m in grid:
            P[k] = eng.render(s, m, length_idx, n=N_SAMPLES)
            k += 1
    D = dist_matrix(P)
    ref = ref_distance(P)
    return {"ref": ref, "distinct_50": packing(D, 0.50 * ref),
            "distinct_75": packing(D, 0.75 * ref), "eff_dim": eff_dim(P)}


HDR = ("        ref  dist50 dist75  local_med local_p95  dead  effdim  span_min span_med")


def print_axis(tag, r):
    print(f"{tag:14s} {r['ref']:.3f}  {r['distinct_50']:5d}  {r['distinct_75']:5d}   "
          f"{r['local_med']:7.3f}   {r['local_p95']:7.3f}  {r['dead_frac']:.2f}  "
          f"{r['eff_dim']:5.1f}   {r['span_min']:6.2f}   {r['span_med']:.2f}")


def report(eng, lengths=(6, 8), mods=(0.0,), shapes=(0.0,), plane=True):
    print(f"=== {eng.name} ===")
    print(HDR)
    for li in lengths:
        L = M.SR_LENGTHS[li]
        for m in mods:
            print_axis(f"SHP L={L} m={m:+.1f}", axis_report(eng, "shape", m, li))
        for s in shapes:
            print_axis(f"MOD L={L} s={s:+.1f}", axis_report(eng, "mod", s, li))
        if plane:
            p = plane_report(eng, li)
            print(f"{'PLANE L=' + str(L):14s} {p['ref']:.3f}  {p['distinct_50']:5d}  "
                  f"{p['distinct_75']:5d}                              {p['eff_dim']:5.1f}")
    print()
