#!/usr/bin/env bash
# Where the module's time actually goes, read off a running one over SWD.
#
# bmcv_profile is a global at a fixed address precisely so it can be read
# without stopping anything: engine_tick() on its own, the whole gated block
# around it, how many ticks ran past their period, and how many were dropped.
# Nothing in the snapshot the browser decodes carries it, so this is the way to
# see it.
#
# What to do with it: `engine` is what the shapes cost, `tick` is that plus the
# input read and the DAC writes, and `load` is `tick` against the tick period.
# A load near 1 means the loop has nothing left for the DAC service between
# ticks, which is what makes dac_fps collapse toward engine_fps.
#
# The ELF has to be the one that was flashed, or the symbol address is nonsense.
#
#   just profile                       # against build-rel
#   just profile build/BMCVFirmware.elf
set -uo pipefail

ELF="${1:-build-rel/BMCVFirmware.elf}"
TOOLS=/home/jan/st/stm32cubeclt_1.18.0/GNU-tools-for-STM32/bin
NM="$TOOLS/arm-none-eabi-nm"

if [[ ! -f "$ELF" ]]; then
  echo "no such ELF: $ELF" >&2
  exit 1
fi

addr=$("$NM" "$ELF" | awk '$3 == "bmcv_profile" { print "0x"$1 }')
if [[ -z "${addr:-}" ]]; then
  echo "bmcv_profile is not in $ELF - was it built with BMCV_PROFILE?" >&2
  exit 1
fi

# Offsets from the ELF's own debug info rather than counted by hand here, for
# the same reason sim/include/layout_target.h is generated: a struct that gains
# a field silently turns hand-counted offsets into plausible nonsense, which is
# exactly what happened the first time this was written.
# Cast to int so gdb prints a plain decimal: a pointer prints as
# "(BmcvSpan *) 0x20", and picking the number back out of that is how the size
# came through as 0 the first time.
layout=$("${ARM_GCC_DIR:-$HOME/arm-gcc-xpack}/bin/arm-none-eabi-gdb-py3" -batch \
  -ex 'print (int)sizeof(BmcvProfile)' \
  -ex 'print (int)&((BmcvProfile *)0)->tick' \
  -ex 'print (int)&((BmcvProfile *)0)->load' \
  -ex 'print (int)&((BmcvSpan *)0)->avg_us' \
  -ex 'print (int)&((BmcvProfile *)0)->dac' \
  -ex 'print (int)&((BmcvProfile *)0)->dac_cplt' \
  "$ELF" 2>/dev/null | grep -oP '^\$\d+ = \K-?\d+$' | tr '\n' ' ')
read -r SIZE OFF_TICK OFF_LOAD OFF_AVG OFF_DAC OFF_DAC_CPLT <<<"$layout"
if [[ -z "${OFF_AVG:-}" || "${SIZE:-0}" -lt 16 ]]; then
  echo "could not read the layout of BmcvProfile from $ELF (got '$layout')" >&2
  exit 1
fi

raw=$(cmd.exe /C "STM32_Programmer_CLI -c port=SWD mode=hotplug -r32 $addr $(printf '0x%x' $((SIZE)))" 2>&1 | tr -d '\r')
words=$(echo "$raw" | grep -oP '^0x[0-9A-Fa-f]+\s*:\s*\K.*' | tr ' ' '\n' | grep -E '^[0-9A-Fa-f]{8}$')

if [[ -z "${words:-}" ]]; then
  echo "read $SIZE bytes at $addr and got nothing back" >&2
  echo "$raw" | tail -8 >&2
  exit 1
fi

python3 - "$addr" "$OFF_TICK" "$OFF_LOAD" "$OFF_AVG" "$OFF_DAC" "$OFF_DAC_CPLT" <<PY
import struct, sys
w = [int(x, 16) for x in """$words""".split()]
raw = b"".join(struct.pack("<I", x) for x in w)
off_tick, off_load, off_avg, off_dac, off_dac_cplt = (int(a) for a in sys.argv[2:7])

def span(off):
    max_at = struct.unpack_from("<I", raw, off + 12)[0]
    last, avg, mn, mx = struct.unpack_from("<4f", raw, off + off_avg - 4)
    return dict(last=last, avg=avg, min=mn, max=mx, max_at=max_at)

engine = span(0)
tick   = span(off_tick)
dac    = span(off_dac)
dac_cplt = span(off_dac_cplt)
load, ticks, overruns, resyncs = struct.unpack_from("<f3I", raw, off_load)

print(f"bmcv_profile @ {sys.argv[1]}   {ticks} ticks since boot")
print()
print(f"{'':8s} {'avg':>9s} {'min':>9s} {'max':>9s}")
for name, s in (("engine", engine), ("tick", tick), ("dac", dac), ("dac_cplt", dac_cplt)):
    print(f"{name:8s} {s['avg']:8.1f}us {s['min']:8.1f}us {s['max']:8.1f}us   (max at tick {s['max_at']})")
print()
print(f"load     {load:.2f} of the tick period")
print(f"overruns {overruns}  ({100.0*overruns/max(ticks,1):.1f}% of ticks ran past their period)")
print(f"resyncs  {resyncs}  (ticks that began a whole period or more late)")

# The dac row is one call of the service, measured inside the timer interrupt
# that owns it - so against its own period it is a share of the whole CPU, not
# of the tick. This is the number that decides DAC_SUBSTEPS, and the one that
# read 55% when the interrupt starved the engine outright.
#
# No backticks or dollar signs in this block: the heredoc is unquoted so it can
# interpolate the words read above, which means the shell reads this too.
if ticks == 0:
    print()
    print("The engine has not ticked. If dac is anywhere near its period, this")
    print("interrupt is why - try 'just where'.")
PY
