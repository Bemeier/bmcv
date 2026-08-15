#!/usr/bin/env python3
"""Turn a layout dump into C static assertions. `just layout-check`.

The generated header only compiles where BmcvInstance has the same shape it has
on the module, so a divergence is a compile error naming the field rather than
wrong numbers on a screen. sim/src/bmcv_sim.c includes it, which puts the check
in `just check` by way of the wasm build.

Why this exists: a debug-probe bridge reads `bmcv` out of the module's RAM as
one raw blob and hands it to the wasm build to decode. That only works while the
two agree byte for byte, and they are built by different compilers - see
docs/live-module.md.
"""

import sys


def main() -> int:
    lines = [ln.rstrip("\n") for ln in sys.stdin if ln.strip()]
    if not lines:
        print("no layout on stdin", file=sys.stderr)
        return 1

    out = [
        "// BmcvInstance, as the module lays it out in RAM.",
        "//",
        "// Generated from the firmware ELF's own debug info by",
        "// tools/gen_layout_asserts.py. Do not edit; regenerate with `just layout-check`",
        "// and read the diff - a field that moved is either an intended change to the",
        "// struct or a compiler disagreeing with the module about one.",
        "//",
        "// Only asserted where pointers are four bytes, which is the module and the wasm",
        "// build. A native x86-64 build has eight-byte pointers, so every offset past",
        "// UxState differs there and always will; it is not a target a probe snapshot is",
        "// ever decoded in.",
        "",
        "#ifndef LAYOUT_TARGET_H_",
        "#define LAYOUT_TARGET_H_",
        "",
        '#include "instance.h"',
        "#include <stddef.h>",
        "",
        "#if defined(__wasm32__) || defined(__arm__)",
        "",
    ]

    for ln in lines:
        if ln.startswith("#"):
            _, _, size = ln.lstrip("# ").split("\t")
            out.append(f'_Static_assert(sizeof(BmcvInstance) == {size}, "BmcvInstance size");')
            continue

        path, off, size = ln.split("\t")
        member = path.split(".", 1)[1]  # drop the "m." root
        out.append(f'_Static_assert(offsetof(BmcvInstance, {member}) == {off}, "offset: {member}");')
        out.append(f'_Static_assert(sizeof(((BmcvInstance*) 0)->{member}) == {size}, "size: {member}");')

    out += [
        "",
        "#endif /* four-byte pointers */",
        "",
        "#endif /* LAYOUT_TARGET_H_ */",
        "",
    ]
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
