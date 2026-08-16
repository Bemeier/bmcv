#!/usr/bin/env bash
# Write scalar fields of a running module over SWD.
#
#   just hil-poke 'bmcv.config.channel_state[0].st_length_idx=10'
#   just hil-poke 'bmcv.config.channel_state[0].params[0][0]=31620'
#
# The read half of the live link - `just hil` - turned one-off readings into
# repeatable ones. This is the other half, and it exists for the same reason:
# a measurement that depends on where somebody left the knobs is an anecdote.
# Setting the module from here is what makes "eight stepped channels wound up
# past the tick rate" a condition two builds can be compared under rather than
# a patch that has to be recreated by hand.
#
# **This writes RAM under a running engine, and the module may autosave what it
# finds there.** It is a debugging instrument, not a configuration tool: point
# it at a module whose preset you are willing to lose.
#
# Addresses and widths come from the ELF, like everything else here. Writes are
# read-modify-write on the containing 32-bit word, because the programmer writes
# words and half the fields worth setting are int16_t or int8_t.
set -uo pipefail

ELF="build-rel/BMCVFirmware.elf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -e|--elf) ELF="$2"; shift 2 ;;
    --) shift; break ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *) break ;;
  esac
done

if [[ $# -eq 0 ]]; then
  echo "usage: hil-poke.sh [-e ELF] EXPR=VALUE..." >&2
  exit 2
fi

if [[ ! -f "$ELF" ]]; then
  echo "no such ELF: $ELF" >&2
  exit 1
fi

GDB="${ARM_GCC_DIR:-$HOME/arm-gcc-xpack}/bin/arm-none-eabi-gdb-py3"

exprs=()
values=()
for a in "$@"; do
  if [[ "$a" != *=* ]]; then
    echo "not an assignment: $a" >&2
    exit 2
  fi
  exprs+=("${a%%=*}")
  values+=("${a#*=}")
done

gdb_args=(-batch)
for e in "${exprs[@]}"; do
  gdb_args+=(-ex "print/x (int)&($e)")
  gdb_args+=(-ex "print (int)sizeof($e)")
done

layout=$("$GDB" "${gdb_args[@]}" "$ELF" 2>&1)
if grep -qE "No symbol|no member" <<<"$layout"; then
  echo "the ELF does not know one of those expressions:" >&2
  grep -E "No symbol|no member" <<<"$layout" | sort -u >&2
  exit 1
fi
mapfile -t vals < <(grep -oP '^\$\d+ = \K.*' <<<"$layout")

HIL_POKES=""
for i in "${!exprs[@]}"; do
  HIL_POKES+="${exprs[$i]}	${vals[$((i * 2))]}	${vals[$((i * 2 + 1))]}	${values[$i]}"$'\n'
done

export HIL_POKES
python3 - <<'PY'
import os, re, struct, subprocess, sys

WORD = re.compile(r"^0x[0-9A-Fa-f]+\s*:\s*(.*)$", re.M)

def prog(*cmd):
    # bytes, not text: the programmer prints a degree sign in its banner under
    # some locales, and decoding as UTF-8 throws before a single word is read.
    out = subprocess.run(["cmd.exe", "/C", "STM32_Programmer_CLI -c port=SWD mode=hotplug " + " ".join(cmd)],
                         capture_output=True).stdout
    return out.decode("utf-8", "replace").replace("\r", "")

def read_word(addr):
    out = prog(f"-r32 {hex(addr)} 0x4")
    for m in WORD.finditer(out):
        for w in m.group(1).split():
            if len(w) == 8:
                return int(w, 16)
    return None

failed = 0
for line in os.environ["HIL_POKES"].strip().splitlines():
    expr, addr_s, width_s, value_s = line.split("\t")
    addr, width, value = int(addr_s, 16), int(width_s), int(value_s)

    # The containing word, then the field patched inside it. Anything else would
    # zero whatever shares those four bytes - and in ChannelConfig that is
    # another parameter.
    base  = addr & ~3
    shift = (addr - base) * 8
    mask  = ((1 << (width * 8)) - 1) << shift

    old = read_word(base)
    if old is None:
        print(f"hil-poke: could not read {expr} at {hex(base)}", file=sys.stderr)
        failed = 1
        continue

    new = (old & ~mask) | ((value & ((1 << (width * 8)) - 1)) << shift)
    prog(f"-w32 {hex(base)} {hex(new)}")

    back = read_word(base)
    if back != new:
        # Read twice before calling it a failure. The engine writes its own
        # counters four thousand times a second, so zeroing bmcv_profile.ticks
        # reads back as a few hundred by the time the verify lands - a write
        # that worked, reported as one that did not. Two reads that differ from
        # each other say the field is live rather than unwritten.
        again = read_word(base)
        if again == back:
            print(f"hil-poke: {expr} did not take (wrote {hex(new)}, read {hex(back)})", file=sys.stderr)
            failed = 1
            continue
        print(f"{expr} = {value}  (live field, already moved on)")
        continue
    print(f"{expr} = {value}")

sys.exit(failed)
PY
