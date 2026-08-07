#!/usr/bin/env bash
set -euo pipefail

# Everything `just build` needs that this repo cannot check in: the ARM
# compiler and ST's HAL. Run once via `just arm-sdk`.
#
# Neither needs ST's account-gated, multi-gigabyte STM32CubeCLT installer -
# CI does not use it either. The compiler is xPack's build of the same GNU
# Arm Embedded toolchain CI installs via apt, and the HAL is only the two
# STM32CubeG4 submodules the firmware actually compiles against, fetched the
# same sparse-checkout way CI does it.
#
# Idempotent: an existing TOOLCHAIN_DIR, CUBE_DIR or toolchain.cmake is left
# alone, so re-running this after a partial failure just finishes the rest.

ARM_GCC_VERSION="13.2.1-1.1"
CUBE_G4_TAG="v1.6.1"

TOOLCHAIN_DIR="${1:?usage: fetch-arm-sdk.sh TOOLCHAIN_DIR CUBE_DIR}"
CUBE_DIR="${2:?usage: fetch-arm-sdk.sh TOOLCHAIN_DIR CUBE_DIR}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$(uname -s)-$(uname -m)" in
  Linux-x86_64) platform=linux-x64 ;;
  Linux-aarch64 | Linux-arm64) platform=linux-arm64 ;;
  Linux-arm*) platform=linux-arm ;;
  Darwin-x86_64) platform=darwin-x64 ;;
  Darwin-arm64) platform=darwin-arm64 ;;
  *)
    echo "fetch-arm-sdk.sh: unsupported platform $(uname -s)-$(uname -m)." >&2
    echo "xPack publishes linux-x64/arm64/arm and darwin-x64/arm64 - on anything" >&2
    echo "else, install the GNU Arm Embedded toolchain yourself and write" >&2
    echo "toolchain.cmake by hand from toolchain.default.cmake." >&2
    exit 1
    ;;
esac

if [ -x "$TOOLCHAIN_DIR/bin/arm-none-eabi-gcc" ]; then
  echo "ARM toolchain already at $TOOLCHAIN_DIR - skipping."
else
  echo "Fetching xPack ARM GCC $ARM_GCC_VERSION ($platform)..."
  asset="xpack-arm-none-eabi-gcc-${ARM_GCC_VERSION}-${platform}.tar.gz"
  base_url="https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v${ARM_GCC_VERSION}"
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  curl -sL -o "$tmp/$asset" "${base_url}/${asset}"
  curl -sL -o "$tmp/$asset.sha" "${base_url}/${asset}.sha"
  (cd "$tmp" && sha256sum -c "$asset.sha")
  mkdir -p "$TOOLCHAIN_DIR"
  tar xzf "$tmp/$asset" -C "$TOOLCHAIN_DIR" --strip-components=1
  rm -rf "$tmp"
  trap - EXIT
  echo "Installed to $TOOLCHAIN_DIR."
fi

if [ -d "$CUBE_DIR/Drivers/STM32G4xx_HAL_Driver/Src" ]; then
  echo "STM32Cube FW already at $CUBE_DIR - skipping."
else
  echo "Fetching STM32Cube FW $CUBE_G4_TAG (HAL driver + CMSIS device only)..."
  rm -rf "$CUBE_DIR"
  git clone --filter=blob:none --no-checkout --depth 1 --branch "$CUBE_G4_TAG" \
    https://github.com/STMicroelectronics/STM32CubeG4.git "$CUBE_DIR"
  (
    cd "$CUBE_DIR"
    git sparse-checkout init --cone
    git sparse-checkout set \
      Drivers/STM32G4xx_HAL_Driver \
      Drivers/CMSIS/Device/ST/STM32G4xx \
      Drivers/CMSIS/Include \
      Middlewares/ST/STM32_USB_Device_Library
    git checkout "$CUBE_G4_TAG"
    # HAL driver and CMSIS device headers are separate repos, linked as submodules.
    git submodule update --init --depth 1 \
      Drivers/STM32G4xx_HAL_Driver \
      Drivers/CMSIS/Device/ST/STM32G4xx
  )
  echo "Fetched to $CUBE_DIR."
fi

if [ -e "$REPO_ROOT/toolchain.cmake" ]; then
  echo "toolchain.cmake already exists - leaving it alone."
else
  cat > "$REPO_ROOT/toolchain.cmake" <<EOF
set(STM32CUBE_FW_PATH "$CUBE_DIR" CACHE PATH "Path to STM32Cube FW")
set(ARM_TOOLCHAIN_DIR "$TOOLCHAIN_DIR" CACHE PATH "Path to ARM Toolchain")
EOF
  echo "Wrote $REPO_ROOT/toolchain.cmake."
fi

echo "Done. \`just configure && just build\` should work now."
