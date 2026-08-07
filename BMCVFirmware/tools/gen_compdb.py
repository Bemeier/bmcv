#!/usr/bin/env python3
"""Write a compile_commands.json for a Make-driven build.

The other three targets are CMake and produce one for free. The Rack plugin
cannot be: Rack supplies plugin.mk and a plugin has to include it. So this asks
make what it *would* run and records that, which keeps the flags - including
the Rack SDK's include paths, which live outside the repo and differ per
machine - out of any checked-in file.

    tools/gen_compdb.py --dir vcv --out vcv/compile_commands.json
"""

import argparse
import json
import os
import shlex
import subprocess
import sys

# The compiler drivers a Rack plugin build uses, native or cross.
DRIVERS = ("cc", "gcc", "g++", "clang", "clang++")

# GCC-only flags. clangd parses with clang, and an argument it does not know is
# a hard error that hides every real diagnostic in the file behind it.
DROP = {"-fno-gnu-unique", "-Wsuggest-override", "-fno-omit-frame-pointer"}


def is_compile(argv):
    if not argv or "-c" not in argv:
        return False
    exe = os.path.basename(argv[0])
    return exe in DRIVERS or exe.endswith(tuple("-" + d for d in DRIVERS))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="directory holding the Makefile")
    ap.add_argument("--out", required=True)
    args, make_args = ap.parse_known_args()

    d = os.path.abspath(args.dir)
    # --always-make so every rule is reported even when the tree is built;
    # --dry-run so nothing is.
    proc = subprocess.run(
        ["make", "--always-make", "--dry-run"] + make_args,
        cwd=d, capture_output=True, text=True,
    )
    if proc.returncode != 0:
        sys.exit(f"make failed:\n{proc.stderr.strip()}")

    entries = []
    for line in proc.stdout.splitlines():
        try:
            argv = shlex.split(line)
        except ValueError:
            continue
        if not is_compile(argv):
            continue
        src = argv[-1]
        if not src.endswith((".c", ".cpp", ".cc")):
            continue
        entries.append({
            "directory": d,
            "file": os.path.normpath(os.path.join(d, src)),
            "arguments": [a for a in argv if a not in DROP],
        })

    if not entries:
        sys.exit("no compile commands found - is the Makefile configured?")

    with open(args.out, "w") as fh:
        json.dump(entries, fh, indent=1)
        fh.write("\n")
    print(f"wrote {args.out} ({len(entries)} entries)")


if __name__ == "__main__":
    main()
