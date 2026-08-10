#!/usr/bin/env bash
set -euo pipefail

# Builds the VCV Rack plugin for one Rack platform. Called by `just vcv`,
# `just vcv-dist` and `just vcv-clean`, and by the CI matrix through those same
# recipes, so a released .vcvplugin and a local build go down one code path.
#
# Rack has exactly four platforms with a published SDK. lin-arm64 is
# deliberately not among them: vcvrack.com serves no such zip, so an arm64
# Linux machine compiles every object and then dies at `-lRack` against an
# x86-64 libRack. One clear error is worth more than that.
#
# Two of the four are cross builds. win-x64 comes from Linux with MinGW-w64,
# because Rack on Windows loads a plugin.dll and will not look at a .so, so a
# Linux checkout needs it to test anything Windows-side. mac-x64 comes from an
# arm64 mac, via CROSS_COMPILE below. Neither crosses an OS boundary: a mac
# platform still needs a mac, since only Apple's toolchain emits a Mach-O dylib.
#
# MAKE_TARGET is passed to vcv/Makefile, except for the pseudo-target `sdk`,
# which fetches the platform's SDK, prints where it is and stops - `just
# vcv-sdk`, for priming a machine or telling clangd where the headers are.
# `--host-arch` in place of an ARCH prints this machine's platform.
#
# Nothing here touches vcv/plugin.json's version. It is fixed at 2.0.0, because
# Rack will not load a plugin whose version does not start with its own major
# and the firmware keeps its own semver - see .cz.toml. Which firmware a built
# plugin came from is instead recorded in the release asset's filename, by
# .github/workflows/vcv.yml.
#
# Overridable from the environment: RACK_SDK_HOME (where SDKs are unpacked),
# RACK_SDK_VERSION, RACK_DIR (an SDK you unpacked yourself - skips the fetch),
# CC/CXX/STRIP, CROSS_COMPILE, and CODESIGN - the last two read by the SDK
# rather than by anything here.

RACK_SDK_VERSION="${RACK_SDK_VERSION:-2.6.6}"
RACK_SDK_HOME="${RACK_SDK_HOME:-$HOME}"

# This machine's platform in Rack's naming, or the bare uname pair if Rack has
# no name for it - which then fails the ARCH check below with it quoted back.
# The Justfile asks for this rather than mapping uname itself, so the platform
# list lives in one file.
host_arch() {
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64) echo lin-x64 ;;
    Linux-aarch64 | Linux-arm64) echo lin-arm64 ;;
    Darwin-x86_64) echo mac-x64 ;;
    Darwin-arm64) echo mac-arm64 ;;
    *) echo "$(uname -s)-$(uname -m)" ;;
  esac
}

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "${1:-}" = --host-arch ]; then
  host_arch
  exit 0
fi

ARCH="${1:?usage: vcv-build.sh ARCH [MAKE_TARGET] | --host-arch}"
TARGET="${2:-all}"

# Whether this host can build that platform at all, before anything is fetched.
case "$ARCH" in
  lin-x64 | win-x64) ;;
  mac-x64 | mac-arm64)
    # Only a mac can produce a Mach-O dylib without a cross toolchain this
    # script does not set up (osxcross, as VCV's own builder uses). Without this
    # the build would look like it was working: arch.mk would ask the host's cc
    # for the target, name the platform lin-x64, and fail somewhere down in the
    # linker against a libRack.dylib it was never going to match. An explicit
    # CROSS_COMPILE is taken as "I have such a toolchain" and left alone.
    if [ "$(uname -s)" != Darwin ] && [ -z "${CROSS_COMPILE:-}" ]; then
      echo "vcv-build.sh: $ARCH has to be built on a mac. Only Apple's toolchain" >&2
      echo "emits a Mach-O dylib, and this script sets up no cross toolchain for" >&2
      echo "it. CI builds both mac platforms - .github/workflows/vcv.yml - or set" >&2
      echo "CROSS_COMPILE and CC yourself if you have an osxcross-style setup." >&2
      exit 1
    fi
    ;;
  lin-arm64)
    echo "vcv-build.sh: Rack publishes no lin-arm64 SDK, so the plugin cannot be" >&2
    echo "linked for arm64 Linux at all - there is no libRack.so to link against." >&2
    echo "The firmware, the simulator and the web frontend all build here; only" >&2
    echo "the Rack plugin does not. Build it for another platform instead:" >&2
    echo "  just vcv-dist win-x64   # needs gcc-mingw-w64-x86-64" >&2
    echo "or let CI do it - .github/workflows/vcv.yml builds all four." >&2
    exit 1
    ;;
  *)
    echo "vcv-build.sh: unknown platform '$ARCH'." >&2
    echo "Rack's are lin-x64, win-x64, mac-x64, mac-arm64." >&2
    exit 1
    ;;
esac

# An SDK per platform rather than one Rack-SDK: a release builds four of them
# from the same checkout, and pointing a build at the wrong libRack is
# otherwise a mystery rather than an error.
#
# Progress goes to stderr throughout, so that `sdk` below can put the directory
# on stdout and be the one thing that knows where SDKs live.
if [ -n "${RACK_DIR:-}" ]; then
  echo "Using the Rack SDK at \$RACK_DIR ($RACK_DIR)." >&2
else
  RACK_DIR="$RACK_SDK_HOME/Rack-SDK-$ARCH"
  if [ -d "$RACK_DIR" ]; then
    echo "Rack SDK ($ARCH) already at $RACK_DIR - skipping." >&2
  else
    echo "Fetching Rack SDK $RACK_SDK_VERSION ($ARCH)..." >&2
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' EXIT
    curl -fL -o "$tmp/sdk.zip" \
      "https://vcvrack.com/downloads/Rack-SDK-$RACK_SDK_VERSION-$ARCH.zip"
    unzip -q "$tmp/sdk.zip" -d "$tmp"
    mkdir -p "$RACK_SDK_HOME"
    mv "$tmp/Rack-SDK" "$RACK_DIR"
    # Explicitly, not left to the trap: the exec at the bottom of this script
    # replaces the shell, and a trap does not survive that.
    rm -rf "$tmp"
    echo "Rack SDK ($ARCH) at $RACK_DIR" >&2
  fi
fi
export RACK_DIR

if [ "$TARGET" = sdk ]; then
  echo "$RACK_DIR"
  exit 0
fi

# No CC is set for the mac platforms: Apple's clang builds for either CPU, and
# CROSS_COMPILE below is the SDK's own way of saying which. Nor is CODESIGN -
# plugin.mk already defaults it to `codesign -f -s -` under ARCH_MAC, the ad-hoc
# signature an arm64 dylib must carry to load at all, so setting it here would
# only be a second copy of the SDK's answer. (Ad-hoc is not notarization: a
# download still arrives quarantined, which is what the README says to clear.)
case "$ARCH" in
  win-x64)
    export CC="${CC:-x86_64-w64-mingw32-gcc}"
    export CXX="${CXX:-x86_64-w64-mingw32-g++}"
    export STRIP="${STRIP:-x86_64-w64-mingw32-strip}"
    ;;
  mac-x64 | mac-arm64)
    # A mac builds for the CPU it is not, when asked. Left alone, arch.mk names
    # the platform from `$(CC) -dumpmachine`, and an Apple silicon machine can
    # only ever produce mac-arm64; CROSS_COMPILE overrides that name, and
    # compile.mk turns it into `--target=`, which reaches the compile and the
    # link alike. That is what keeps mac-x64 reachable after August 2027, when
    # Actions retires its last x86_64 macOS runner and the only mac left to
    # build on is an arm64 one.
    case "$(uname -m)-$ARCH" in
      arm64-mac-x64) export CROSS_COMPILE="${CROSS_COMPILE:-x86_64-apple-darwin}" ;;
      x86_64-mac-arm64) export CROSS_COMPILE="${CROSS_COMPILE:-arm64-apple-darwin}" ;;
    esac
    ;;
esac

exec make -C "$REPO_ROOT/vcv" "$TARGET" -j"$(getconf _NPROCESSORS_ONLN)"
