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

# The struct is 2 BmcvSpans (9 words each) then load, ticks, overruns, resyncs.
words=$(cmd.exe /C "STM32_Programmer_CLI -c port=SWD mode=hotplug -r32 $addr 0x60" 2>&1 | tr -d '\r' |
  grep -oP '^0x[0-9A-Fa-f]+\s*:\s*\K.*' | tr ' ' '\n' | grep -E '^[0-9A-Fa-f]{8}$')

if [[ -z "${words:-}" ]]; then
  echo "could not read $addr - is it powered, and the ST-Link attached?" >&2
  exit 1
fi

python3 - "$addr" <<PY
import struct, sys
w = [int(x, 16) for x in """$words""".split()]
raw = b"".join(struct.pack("<I", x) for x in w)

def span(off):
    last_c, min_c, max_c, max_at = struct.unpack_from("<4I", raw, off)
    last, avg, mn, mx = struct.unpack_from("<4f", raw, off + 20)
    return dict(last=last, avg=avg, min=mn, max=mx, max_at=max_at)

engine = span(0)
tick   = span(36)
load, ticks, overruns, resyncs = struct.unpack_from("<f3I", raw, 72)

print(f"bmcv_profile @ {sys.argv[1]}   {ticks} ticks since boot")
print()
print(f"{'':8s} {'avg':>9s} {'min':>9s} {'max':>9s}")
for name, s in (("engine", engine), ("tick", tick)):
    print(f"{name:8s} {s['avg']:8.1f}us {s['min']:8.1f}us {s['max']:8.1f}us   (max at tick {s['max_at']})")
print()
print(f"load     {load:.2f} of the tick period")
print(f"overruns {overruns}  ({100.0*overruns/max(ticks,1):.1f}% of ticks ran past their period)")
print(f"resyncs  {resyncs}  (ticks dropped outright)")
PY
