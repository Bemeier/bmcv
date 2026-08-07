#!/usr/bin/env bash
# Where is the module right now? Halts the core over SWD, reads PC and LR,
# translates both back to source lines, and lets it run on.
#
# For the failure that leaves no other evidence: a firmware that has stopped
# responding but is still powered. A frozen panel looks the same whether the
# main loop is stuck in a spin, sitting in a fault handler, or off in code that
# no longer exists - and this is what tells the three apart.
#
# The ELF has to be the one that was flashed, or the addresses resolve to
# nonsense. Defaults to build-rel, which is what the .bin comes from.
#
#   just where                       # against build-rel
#   just where build/BMCVFirmware.elf
set -uo pipefail

ELF="${1:-build-rel/BMCVFirmware.elf}"
TOOLS=/home/jan/st/stm32cubeclt_1.18.0/GNU-tools-for-STM32/bin
ADDR2LINE="$TOOLS/arm-none-eabi-addr2line"
NM="$TOOLS/arm-none-eabi-nm"

if [[ ! -f "$ELF" ]]; then
  echo "no such ELF: $ELF" >&2
  exit 1
fi

out=$(cmd.exe /C "STM32_Programmer_CLI -c port=SWD mode=hotplug -halt -coreReg PC -coreReg LR -coreReg MSP -coreReg XPSR -run" 2>&1 | tr -d '\r')

read_reg() { echo "$out" | grep -oP "^$1\s+=\s+\K0x[0-9A-Fa-f]+" | head -1; }

pc=$(read_reg PC)
if [[ -z "${pc:-}" ]]; then
  echo "could not read the core - is it powered, and the ST-Link attached?" >&2
  echo "$out" | tail -5 >&2
  exit 1
fi
lr=$(read_reg LR)
msp=$(read_reg MSP)
xpsr=$(read_reg XPSR)

# Nearest preceding function symbol, plus the file:line from the debug info.
sym() {
  local addr=$(( $1 & 0xFFFFFFFE )) # drop the Thumb bit
  local name
  name=$("$NM" -C "$ELF" | awk -v a="$addr" \
    '$2 ~ /^[TtWw]$/ { s=strtonum("0x"$1); if (s<=a && (best=="" || a-s<best)) { best=a-s; n=$3 } } END { print (n==""?"?":n) }')
  printf '0x%08X  %-28s %s\n' "$addr" "$name" \
    "$("$ADDR2LINE" -e "$ELF" -s -C "$(printf '0x%X' "$addr")")"
}

# XPSR's low 9 bits are the exception number: 0 means thread mode (normal code),
# 3 means HardFault, and anything else is whichever handler is running.
mode=$(( xpsr & 0x1FF ))
case "$mode" in
  0) where="thread mode (normal code)" ;;
  3) where="HardFault" ;;
  *) where="exception $mode" ;;
esac

echo "mode  $where"
echo "MSP   $msp"
echo
echo "PC    $(sym $((pc)))"
# LR points after the call, so step back to land on the calling line.
echo "LR    $(sym $((lr - 2)))"
