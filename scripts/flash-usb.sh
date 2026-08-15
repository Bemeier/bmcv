#!/usr/bin/env bash
set -euo pipefail

# Flash over the module's own USB port, through the STM32's ROM DFU bootloader,
# instead of over SWD through an ST-Link.
#
# The point is that the module can stay in the rack. The debug probe needs the
# module out and a ribbon on its header; its USB port is on the panel and is
# already patched into the computer for the simulator's live link. So this is
# the recipe to reach for mid-session, and scripts/flash.sh the one to reach for
# when something needs stepping through.
#
# What it gives up against the ST-Link: no halt, no breakpoints, no
# scripts/where.sh. Printf-level debugging over the WebUSB link still works, and
# so does the diagnostics page - see docs/live-module.md.
#
# Same shape as flash.sh: the flashing tool is the Windows-side
# STM32_Programmer_CLI, because under WSL the USB device belongs to Windows.
# Handing it to WSL instead would take an elevated `usbipd bind`, and would then
# hide the device from the very tool that has to write to it.

BIN="${1:-build/BMCVFirmware.bin}"

# Where the ROM bootloader puts the application. A raw .bin carries no load
# address, unlike the .elf the SWD path writes, so it has to be said here.
FLASH_START="0x08000000"

# The module in normal firmware, and the ROM bootloader it resets into. Kept in
# step with web/probe/wire.js and web/update/dfuse.js, which are the two other
# places that have to know them.
BMCV_ID="0483:572b"
DFU_ID="0483:df11"

# How long to keep looking for the bootloader after asking for it, so putting
# the module into update mode can happen *after* the command is started rather
# than before. Two boots' worth: the reset itself, then enumeration.
WAIT_S="${FLASH_USB_WAIT:-30}"

# Somewhere both sides can see. The Windows tool cannot open a \\wsl.localhost
# path, so the image is copied out first - the same arrangement flash.sh makes
# for the .elf, and the same default, overridable for a machine that is not this
# one.
WIN_DIR="${FLASH_WIN_DIR:-/mnt/c/Users/janko/Desktop}"

if [[ ! -f "$BIN" ]]; then
  echo "Image not found: $BIN" >&2
  echo "Build one first: just build   (or: just build-rel)" >&2
  exit 1
fi

# cmd.exe refuses a UNC working directory and prints a warning about it before
# doing the right thing anyway. Running it from a drive letter keeps the output
# to what the tool actually said.
#
# Failure is swallowed here rather than left to `set -e`: both callers below are
# *asking a question*, and "the tool exited non-zero because it found nothing"
# is one of the answers, not a reason to stop.
win() { (cd /mnt/c && cmd.exe /C "$@" 2>&1 | tr -d '\r') || true; }

# usbipd is asked only to *list*; nothing is bound or attached. It is the one
# thing on this side that can see Windows' USB tree, which is what makes
# "plugged in but not in update mode" distinguishable from "not plugged in".
usb_present() { win "usbipd list" | grep -qi "$1"; }

# Two tests rather than one. Matching USB1 alone would be fooled by the tool
# printing usage text; excluding the empty-list message alone would be fooled by
# it failing to run at all. The device has to be both not-absent and named.
dfu_present() {
  local out
  out=$(win "STM32_Programmer_CLI -l usb")
  ! grep -qi "No STM32 device" <<<"$out" && grep -qi "USB1" <<<"$out"
}

if ! dfu_present; then
  echo "No STM32 in DFU mode on USB yet."
  echo

  if usb_present "$BMCV_ID"; then
    echo "The module is there and running its firmware. Put it into update mode:"
  else
    echo "No BMCV on USB either. Check the cable, then put it into update mode:"
  fi

  echo
  echo "  - hold CPY while powering the module up, or"
  echo "  - open http://localhost:8000/update/ (just docs-page) and press its"
  echo "    update-mode button - that is the same request over WebUSB, and it"
  echo "    needs no hands on the module."
  echo
  echo "Waiting up to ${WAIT_S}s..."

  for ((i = 0; i < WAIT_S; i++)); do
    sleep 1
    if dfu_present; then
      echo "Bootloader is up."
      break
    fi
  done

  if ! dfu_present; then
    echo >&2
    echo "Gave up waiting. Nothing was written." >&2
    exit 1
  fi
fi

mkdir -p "$WIN_DIR"
WIN_BIN="$WIN_DIR/$(basename "$BIN")"
cp "$BIN" "$WIN_BIN"
WIN_BIN_W=$(wslpath -w "$WIN_BIN")

echo "Flashing over USB DFU: $WIN_BIN_W -> $FLASH_START"

# -rst runs the new firmware rather than leaving the module sitting in the
# bootloader, which is what makes this a one-command edit-flash-listen loop.
win "STM32_Programmer_CLI -c port=USB1 -w \"$WIN_BIN_W\" $FLASH_START -v -rst"
