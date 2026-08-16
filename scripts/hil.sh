#!/usr/bin/env bash
# Sample named fields of a running module over SWD, as TSV.
#
# The whole point of the live link is that `BmcvInstance` is the module - so
# this reads all of it in one SWD burst and decodes fields out of the copy,
# rather than reading each field separately. That is not an optimisation for its
# own sake: a full-instance read and a 16-byte read both measure 0.13s, because
# every invocation of STM32_Programmer_CLI pays a fresh connect and the transfer
# itself is noise next to it. So one burst per sample costs nothing extra and
# gives fields that were read within microseconds of each other - which is what
# makes correlating two of them mean anything.
#
# Offsets and types come from the ELF's debug info, never from counting bytes
# here - the same rule sim/include/layout_target.h and scripts/profile.sh follow,
# and for the same reason: a struct that gains a field turns hand-counted
# offsets into plausible nonsense rather than into an error.
#
#   just hil bmcv.engine_state.dac_fps bmcv.engine_state.engine_fps
#   just hil -n 60 'bmcv.engine_state.channels_gated_level[0]'
#
# Needs the ST-Link attached and an ELF matching what is flashed.
set -uo pipefail

ELF="build-rel/BMCVFirmware.elf"
N=1
INTERVAL=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n) N="$2"; shift 2 ;;
    -i|--interval) INTERVAL="$2"; shift 2 ;;
    -e|--elf) ELF="$2"; shift 2 ;;
    --) shift; break ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *) break ;;
  esac
done

if [[ $# -eq 0 ]]; then
  echo "usage: hil.sh [-n COUNT] [-i SECONDS] [-e ELF] EXPR..." >&2
  echo "  EXPR is any scalar inside bmcv, e.g. bmcv.engine_state.dac_fps" >&2
  exit 2
fi

if [[ ! -f "$ELF" ]]; then
  echo "no such ELF: $ELF" >&2
  exit 1
fi

GDB="${ARM_GCC_DIR:-$HOME/arm-gcc-xpack}/bin/arm-none-eabi-gdb-py3"

# One gdb invocation for the whole layout, because gdb costs more to start than
# a sample costs to take. Three questions per expression: where it sits relative
# to the instance, how wide it is, and how to read the bytes back.
gdb_args=(-batch -ex 'print (int)sizeof(bmcv)' -ex 'print/x (int)&bmcv')
for e in "$@"; do
  gdb_args+=(-ex "print (int)((char *)&($e) - (char *)&bmcv)")
  gdb_args+=(-ex "print (int)sizeof($e)")
  gdb_args+=(-ex "whatis $e")
done

layout=$("$GDB" "${gdb_args[@]}" "$ELF" 2>&1)
if grep -qE "No symbol|no member" <<<"$layout"; then
  echo "the ELF does not know one of those expressions:" >&2
  grep -E "No symbol|no member" <<<"$layout" | sort -u >&2
  exit 1
fi

# gdb answers "$1 = 2736" and "type = float", one per line and nothing else on
# them. The two orders are independent, so they are collected separately and
# zipped below rather than parsed as one interleaved stream.
mapfile -t vals < <(grep -oP '^\$\d+ = \K.*' <<<"$layout")
mapfile -t types < <(grep -oP '^type = \K.*' <<<"$layout")

SIZE="${vals[0]}"
BASE="${vals[1]}"
if [[ -z "${BASE:-}" || "${SIZE:-0}" -lt 4 ]]; then
  echo "could not read bmcv's layout from $ELF" >&2
  echo "$layout" | tail -8 >&2
  exit 1
fi
SIZE=$(( (SIZE + 3) / 4 * 4 )) # -r32 wants a whole number of words

# offset<TAB>width<TAB>type, one line per expression, in the order they were
# asked for.
HIL_LAYOUT=""
for i in "${!types[@]}"; do
  HIL_LAYOUT+="${vals[$((2 + i * 2))]}	${vals[$((3 + i * 2))]}	${types[$i]}"$'\n'
done

export SIZE BASE N INTERVAL HIL_LAYOUT
python3 - "$@" <<'PY'
import os, re, struct, subprocess, sys, time

size     = int(os.environ["SIZE"])
base     = os.environ["BASE"]
n        = int(os.environ["N"])
interval = float(os.environ["INTERVAL"])
exprs    = sys.argv[1:]

# Widths are checked against the ELF's below, so a type whose size differs
# between the two builds is an error here rather than a silently shifted read.
FMT = {
    "int8_t": "b", "signed char": "b", "char": "b",
    "uint8_t": "B", "unsigned char": "B", "_Bool": "B",
    "int16_t": "h", "short": "h", "short int": "h",
    "uint16_t": "H", "short unsigned int": "H",
    "int32_t": "i", "int": "i", "long": "i", "long int": "i",
    "uint32_t": "I", "unsigned int": "I", "long unsigned int": "I",
    "float": "f", "double": "d",
}

fields = []
for line in os.environ["HIL_LAYOUT"].strip().splitlines():
    off, width, typ = line.split("\t")
    fmt = FMT.get(typ.strip())
    if fmt is None:
        sys.exit(f"hil: no idea how to read a {typ!r} - add it to FMT in scripts/hil.sh")
    if struct.calcsize(fmt) != int(width):
        sys.exit(f"hil: {typ} is {width} bytes in the ELF and {struct.calcsize(fmt)} here")
    fields.append((int(off), fmt))

WORD = re.compile(r"^0x[0-9A-Fa-f]+\s*:\s*(.*)$", re.M)

def snapshot():
    out = subprocess.run(
        ["cmd.exe", "/C", f"STM32_Programmer_CLI -c port=SWD mode=hotplug -r32 {base} {hex(size)}"],
        capture_output=True, text=True,
    ).stdout.replace("\r", "")
    words = []
    for m in WORD.finditer(out):
        words += [w for w in m.group(1).split() if len(w) == 8]
    # The reply is words in address order; short means the read was cut off, and
    # decoding it would give plausible numbers for the fields that did arrive.
    if len(words) * 4 < size:
        return None
    return b"".join(struct.pack("<I", int(w, 16)) for w in words)

print("t\t" + "\t".join(exprs))
t0 = time.time()
for i in range(n):
    if i and interval:
        time.sleep(max(0.0, t0 + i * interval - time.time()))
    t = time.time() - t0
    raw = snapshot()
    if raw is None:
        print("hil: no answer from the module - is it powered, and the ST-Link attached?", file=sys.stderr)
        sys.exit(1)
    vals = [struct.unpack_from("<" + f, raw, o)[0] for o, f in fields]
    print(f"{t:.3f}\t" + "\t".join(f"{v:g}" if isinstance(v, float) else str(v) for v in vals))
    sys.stdout.flush()
PY
