# Setup

What to install, once, to build each of the four things this repo builds -
see [architecture.md](architecture.md) for what they are. Every fetchable
dependency (the ARM toolchain, ST's HAL, emsdk, the Rack SDK) is one `just`
recipe away rather than a manual download, and none of them need an account
or an installer: `just arm-sdk` replaces ST's STM32CubeCLT with a couple
hundred megabytes fetched straight from GitHub and ST's own HAL repos.

## Common to everything

- **[`just`](https://just.systems/man/en/packages.html)** runs every recipe
  in this doc. `cargo install just`, `brew install just`, or see the link for
  your platform.
- **CMake 3.22+** and **Ninja**. `apt install cmake ninja-build`,
  `brew install cmake ninja`.
- **git**, **curl** - for the fetch recipes below.
- **Python 3** - the `just wavetable`/`just sr-table`/`just panel`
  generators, and `just web`'s dev server. Not needed for a build.

## Firmware (ARM)

```
just arm-sdk        # once: fetches the ARM toolchain + STM32Cube HAL
just configure       # once per fresh build/, or after editing CMakeLists.txt
just build            # -> build/BMCVFirmware.elf, build/BMCVFirmware.bin
just flash             # over an ST-Link, via OpenOCD
```

`arm-sdk` fetches two things, both version-pinned in
`scripts/fetch-arm-sdk.sh` for reproducibility:

- The [xPack build](https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack)
  of the GNU Arm Embedded toolchain - the same compiler CI installs via
  `apt`, just self-contained rather than a system package. sha256-verified
  against the release's own checksum.
- ST's HAL and CMSIS device headers, sparse-checked-out from
  [STM32CubeG4](https://github.com/STMicroelectronics/STM32CubeG4) at the tag
  named in the script - only the ~30MB the firmware actually compiles
  against, not the multi-gigabyte package with every ST eval board's BSP.

It writes `toolchain.cmake` pointing at both (gitignored - this is meant to
vary per machine) and leaves one alone if it already exists, so re-running
it after an interrupted fetch just finishes the rest. `ARM_GCC_DIR` and
`CUBE_G4_DIR` override where things land, same as `RACK_DIR`/`EMSDK_DIR`
below; both default under `$HOME`.

If you already have STM32CubeCLT (from STM32CubeIDE) and would rather point
at that instead, copy `toolchain.default.cmake` to `toolchain.cmake` and
fill in its two paths - `arm-sdk` is a replacement for that installer, not a
requirement on top of it.

## Native tests and tools

```
just configure-native
just test              # ctest over Core/Src/Lib, host-compiled
just test-san           # the same, under ASan/UBSan
```

Needs only cmake/ninja/a C compiler - no ARM toolchain, no STM32Cube FW. The
one requirement worth calling out: **GCC 13+**, or a comparably recent
Clang. `Core/Inc/Lib/hw_setup.h` uses `static_assert` as a bare C23 keyword,
which GCC did not support until 13; an older compiler fails several files
deep with a confusing `expected declaration specifiers` rather than
anything mentioning C23, which is why `tests/CMakeLists.txt` and
`sim/CMakeLists.txt` both check for this up front and say so plainly.

Ubuntu 24.04 and Debian 13 ship a new-enough `gcc` as the system compiler.
On an older system, install a newer one alongside it rather than replacing
the system compiler, and point `CC` at it:

```
# Ubuntu 22.04 and older need the toolchain PPA first; 24.04+ has gcc-13
# directly. Debian: gcc-13 is in bookworm-backports on 12, direct on 13.
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test && sudo apt update
sudo apt install gcc-13
just configure-native && CC=gcc-13 cmake -S tests -B build-native -G Ninja

# or, no root and no package manager needed, into a local prefix:
curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj bin/micromamba
MAMBA_ROOT_PREFIX=~/.micromamba ./bin/micromamba create -y -p ~/.gcc13 -c conda-forge gcc=13
CC=~/.gcc13/bin/gcc just configure-native
```

macOS's Clang has had this for years, so this is a Linux-only concern in
practice.

## Web simulator (wasm)

```
just wasm-sdk         # once: clones and builds emsdk (a real build - slow)
just configure-wasm
just wasm               # -> web/bmcv.js, web/bmcv.wasm
just web                 # serves web/ at http://localhost:8000
```

`wasm-sdk` clones [emsdk](https://github.com/emscripten-core/emsdk) to
`$EMSDK_DIR` (default `~/emsdk`) and installs its `latest` toolchain -
self-contained, needs nothing from the system beyond what emsdk's own
installer wants (Python 3, and on Linux, the usual build-essential
headers). `just wasm-check`/`web-check` additionally need **Node** to run
headless against the built module - any reasonably recent LTS.

## VCV Rack plugin

```
just vcv-install         # -> ~/.local/share/Rack2/plugins-lin-x64/BMCV
just vcv-dist            # -> vcv/dist/BMCV-<version>-<platform>.vcvplugin
```

Both fetch the Rack SDK they need on first use; `just vcv-sdk` does only
that, for priming a machine. Needs `make` and a C/C++ toolchain, both
usually already present (`build-essential` on Debian/Ubuntu, Xcode Command
Line Tools on macOS). The same GCC 13+ / modern-Clang requirement as the
native tests applies here too - `vcv/Makefile` compiles the same
`Core/Src/Lib` sources, and checks for it the same way before running
`make -j` turns one real error into a wall of them across every source in
parallel. Pass `CC=`/`CXX=` to build with a non-default compiler.

### Platforms

Every recipe takes a Rack platform name and defaults to this host's:

```
just vcv-dist lin-x64      just vcv-dist mac-x64
just vcv-dist win-x64      just vcv-dist mac-arm64
```

Those four are the entire list, because they are the four Rack publishes an
SDK for. There is **no `lin-arm64`**: on an arm64 Linux machine the plugin
compiles and then has no `libRack.so` to link against, so the recipes stop
with that rather than with `cannot find -lRack`. Everything else in this
repository - firmware, tests, simulator, web frontend - builds there fine.

`win-x64` is a cross build from Linux, which is what a checkout in WSL needs
since Rack on Windows loads a `plugin.dll` and will not look at a `.so`. It
wants the MinGW toolchain, GCC 13 or newer for the same C23 reason as above:

```
sudo apt install gcc-mingw-w64-x86-64 g++-mingw-w64-x86-64   # 24.04+: GCC 13
```

The two mac platforms need a mac - only Apple's toolchain emits a Mach-O
dylib - but either mac builds both: the recipes hand the SDK a `CROSS_COMPILE`
when the CPU you have is not the one you asked for, so `just vcv-dist mac-x64`
works on Apple silicon. If you have no mac at all,
[`vcv.yml`](../.github/workflows/vcv.yml) builds all four on every pull
request and attaches them to each release.

To build the two x86_64 platforms on a machine that is not x86_64, an emulated
container works. Register the emulator and build an image once:

```
docker run --privileged --rm tonistiigi/binfmt --install amd64
printf 'FROM ubuntu:24.04\nRUN apt-get update && apt-get install -y \\\n\
  build-essential jq zstd unzip curl ca-certificates \\\n\
  gcc-mingw-w64-x86-64 g++-mingw-w64-x86-64\n' \
  | docker build --platform linux/amd64 -t bmcv-vcv-builder -
```

then build in it as yourself, so nothing in the checkout ends up root-owned:

```
docker run --rm --platform linux/amd64 -v "$PWD:/repo" -w /repo \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e RACK_SDK_HOME=/tmp/sdks \
  bmcv-vcv-builder ./scripts/vcv-build.sh win-x64 dist
```

Slow under emulation, but it is how `lin-x64` and `win-x64` were first
verified from an arm64 machine. Nothing reaches macOS this way.

## Versioning (commitizen)

```
pipx install commitizen     # or: pip install --user commitizen
just bump-dry-run             # preview
just bump                      # bump VERSION + CHANGELOG.md, commit, tag
```

See the [Versioning](../README.md#versioning) section of the main README for
what commit messages this expects, and for why `vcv/plugin.json`'s version is
fixed at `2.0.0` rather than following `VERSION` like everything else.

## Formatting

```
# Ubuntu/Debian:
sudo apt install clang-format-18
# macOS:
brew install clang-format@18

just fmt-check                          # CI's check
CLANG_FORMAT=clang-format-18 just fmt    # if `clang-format` on PATH isn't 18
```

Pinned to 18 specifically because clang-format's output moves between major
versions - a different version reformatting everything is its own kind of
diff noise.

## Troubleshooting

**`expected declaration specifiers or '...' before numeric constant` in
`hw_setup.h`, or a wall of similar errors across many files at once.** Host
GCC is older than 13 - see [Native tests and tools](#native-tests-and-tools)
above.

**`arm-none-eabi-objcopy: not found` right at the end of an otherwise
successful ARM build.** `toolchain.cmake`'s `ARM_TOOLCHAIN_DIR` is set but
its `bin/` is not on `PATH`, and something predates the fix that makes this
unnecessary - pull the latest `cmake/gcc-arm-none-eabi.cmake`, which now
resolves `objcopy`/`size` the same way it already resolved the compiler.

**`STM32CUBE_FW_PATH = ''` in the CMake configure log, then hundreds of
missing-header errors.** `toolchain.cmake` does not exist yet - run
`just arm-sdk`, or see [Firmware (ARM)](#firmware-arm) above for pointing at
an existing STM32CubeCLT install instead.
