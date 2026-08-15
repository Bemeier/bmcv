"""The properties a variant is not allowed to break.

These mirror tests/test_stepped_random.c, plus the two the C tests take for
granted because the shipping algorithm cannot violate them.
"""
import numpy as np
import srmodel as M

LI_ALL = range(len(M.SR_LENGTHS))


def check(eng, verbose=True):
    fails = []

    def rec(name, ok, detail):
        if not ok:
            fails.append(name)
        if verbose:
            print(f"  {'ok  ' if ok else 'FAIL'} {name:38s} {detail}")

    # 1. cyclic in SHP: -1 and +1 must land on the same pattern
    worst = 0.0
    for m in np.linspace(-1, 1, 9):
        for li in LI_ALL:
            a = eng.render(-1.0, m, li, n=256)
            b = eng.render(1.0, m, li, n=256)
            worst = max(worst, np.abs(a - b).max())
    rec("SHP cyclic at the wrap", worst < 0.02, f"max |diff| {worst:.4f}")

    # 2. cyclic in MOD (informational: the shipping algorithm is not)
    worst_mod = 0.0
    for s in np.linspace(-1, 1, 9):
        for li in LI_ALL:
            a = eng.render(s, -1.0, li, n=256)
            b = eng.render(s, 1.0, li, n=256)
            worst_mod = max(worst_mod, np.abs(a - b).max())
    rec("MOD cyclic at the wrap (info)", True, f"max |diff| {worst_mod:.4f}")

    # 3. the cycle closes on itself
    worst = 0.0
    for s in np.linspace(-1, 1, 9):
        for m in np.linspace(-1, 1, 5):
            for li in LI_ALL:
                c = eng.render(s, m, li, n=2048)
                worst = max(worst, abs(c[0] - c[-1]) - abs(c[1] - c[0]))
    rec("loop point closes", worst < 0.02, f"excess jump {worst:.4f}")

    # 4. slot 0 length-independent: switching length on the wrap is seamless
    worst = 0.0
    for s in np.linspace(-1, 1, 21):
        for m in np.linspace(-1, 1, 5):
            v = [eng.render(s, m, li, n=256)[0] for li in LI_ALL]
            worst = max(worst, max(v) - min(v))
    rec("length switch seamless", worst < 0.05, f"max spread {worst:.4f}")

    # 5. no dead settings
    worst = 9.0
    where = None
    for s in np.linspace(-1, 1, 41):
        for m in np.linspace(-1, 1, 11):
            for li in LI_ALL:
                c = eng.render(s, m, li, n=512)
                sp = c.max() - c.min()
                if sp < worst:
                    worst, where = sp, (s, m, M.SR_LENGTHS[li])
    rec("no flat settings", worst > 0.5, f"min span {worst:.3f} at {where}")

    # 6. output in range
    worst = 0.0
    for s in np.linspace(-1, 1, 21):
        for m in np.linspace(-1, 1, 9):
            for li in LI_ALL:
                worst = max(worst, np.abs(eng.render(s, m, li, n=256)).max())
    rec("stays in [-1,1]", worst <= 1.0001, f"max |v| {worst:.4f}")

    # 7. a small turn deforms the pattern, never re-randomises it
    #    (test_stepped_random.c: worst per-sample diff < 0.25 for ds = 0.01)
    worst = 0.0
    for s in np.linspace(-1, 0.99, 60):
        a = eng.render(s, 0.0, 6, n=128)
        b = eng.render(s + 0.01, 0.0, 6, n=128)
        worst = max(worst, np.abs(a - b).max())
    rec("small SHP turn deforms only", worst < 0.25, f"worst sample diff {worst:.4f}")

    worst = 0.0
    for m in np.linspace(-1, 0.99, 60):
        a = eng.render(0.3, m, 6, n=128)
        b = eng.render(0.3, m + 0.01, 6, n=128)
        worst = max(worst, np.abs(a - b).max())
    rec("small MOD turn deforms only", worst < 0.25, f"worst sample diff {worst:.4f}")

    # 8. the curve has no step discontinuity anywhere
    worst = 0.0
    for s in np.linspace(-1, 1, 9):
        for m in np.linspace(-1, 1, 5):
            for li in LI_ALL:
                c = eng.render(s, m, li, n=4000)
                worst = max(worst, np.abs(np.diff(np.concatenate([c, c[:1]]))).max())
    rec("curve continuous", worst < 0.15, f"max sample-to-sample {worst:.4f}")

    # 9. MOD still thins the pattern out as it turns up
    def distinct(m, li):
        L = M.SR_LENGTHS[li]
        _, _, jumps = eng.steps(0.3, m, L)
        return int(jumps.sum())

    li = 6
    busy, neutral, sparse = distinct(-1.0, li), distinct(0.0, li), distinct(1.0, li)
    rec("MOD density monotone", busy >= neutral >= sparse and busy > sparse,
        f"L=16 new-value steps: {busy} -> {neutral} -> {sparse}")

    return fails
