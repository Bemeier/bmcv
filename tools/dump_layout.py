# Walk BmcvInstance in an ELF's debug info and print every leaf field as
#   path <TAB> byte offset <TAB> byte size
#
# Run under gdb, not python:
#
#   arm-none-eabi-gdb-py3 -batch -x tools/dump_layout.py build-rel/BMCVFirmware.elf
#
# The point is to have the target's real layout - the one a debug probe reads
# out of RAM - in a form another toolchain's layout can be diffed against.
# tools/gen_layout_asserts.py turns the output into C static assertions.

import sys

import gdb


def walk(t, path, base, out):
    t = t.strip_typedefs()

    if t.code == gdb.TYPE_CODE_ARRAY:
        elem = t.target()
        n = t.sizeof // elem.sizeof if elem.sizeof else 0
        # Element 0 only: the rest are the same shape at a fixed stride, and
        # the stride is pinned by asserting the whole array's size.
        out.append((path, base, t.sizeof))
        if n:
            walk(elem, f"{path}[0]", base, out)
        return

    if t.code in (gdb.TYPE_CODE_STRUCT, gdb.TYPE_CODE_UNION):
        for f in t.fields():
            if f.is_base_class or f.name is None:
                continue
            if getattr(f, "bitsize", 0):
                # A bitfield has no byte offset to assert. None of the module's
                # structs use one; if that changes this must grow a case rather
                # than silently skip it.
                raise SystemExit(f"bitfield in {path}.{f.name} - unhandled")
            walk(f.type, f"{path}.{f.name}", base + f.bitpos // 8, out)
        return

    out.append((path, base, t.sizeof))


root = gdb.lookup_type("BmcvInstance")
leaves = []
walk(root, "m", 0, leaves)

print(f"# BmcvInstance\t0\t{root.sizeof}")
for path, off, size in leaves:
    print(f"{path}\t{off}\t{size}")

sys.stdout.flush()
