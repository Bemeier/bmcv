#!/usr/bin/env bash
set -euo pipefail

# --- config ---
SRC_ELF="${1:-build/BMCVFirmware.elf}"

# Where Windows tool expects the file
WIN_DIR="/mnt/c/Users/janko/Desktop"
WIN_ELF_PATH="$WIN_DIR/BMCVFirmware.elf"

# --- sanity checks ---
if [[ ! -f "$SRC_ELF" ]]; then
  echo "ELF not found: $SRC_ELF"
  exit 1
fi

mkdir -p "$WIN_DIR"

cp "$SRC_ELF" "$WIN_ELF_PATH"
WIN_ELF_PATH_WIN=$(wslpath -w "$WIN_ELF_PATH")

echo "Flashing: $WIN_ELF_PATH_WIN"

cmd.exe /C STM32_Programmer_CLI \
  -c port=SWD \
  -w "$WIN_ELF_PATH_WIN" \
  -v -rst

