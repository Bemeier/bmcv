# Anything machine-specific goes in local.just, which is not checked in - the
# same arrangement CMakeLists.txt has with local.cmake. It can add variables and
# recipes, but not redefine a variable this file already sets: just rejects that
# outright with "variable has multiple definitions". To change one of those,
# either set it in the environment, since every one of them reads there first:
#
#   export RACK_WIN_DIR=/mnt/c/Users/someone/AppData/Local/Rack2
#
# or pass it for a single run, which needs no local.just at all:
#
#   just RACK_WIN_DIR=/mnt/c/Users/someone/AppData/Local/Rack2 vcv-win-install
import? 'local.just'

# Everything `just build` needs beyond cmake/ninja: the ARM compiler and ST's
# HAL, fetched once. Neither needs ST's account-gated, multi-gigabyte
# STM32CubeCLT installer - see scripts/fetch-arm-sdk.sh for what this gets
# instead and why. Writes toolchain.cmake pointing at both, unless one is
# already there, so this is the only thing to run before `just configure`
# on a fresh checkout.
ARM_GCC_DIR := env_var_or_default("ARM_GCC_DIR", env_var("HOME") / "arm-gcc-xpack")
CUBE_G4_DIR := env_var_or_default("CUBE_G4_DIR", env_var("HOME") / "STM32CubeG4")

arm-sdk:
	./scripts/fetch-arm-sdk.sh "{{ARM_GCC_DIR}}" "{{CUBE_G4_DIR}}"

configure:
  rm -Rf build && cmake -B build -G Ninja

build:
	cmake --build build

flash:
	./scripts/flash.sh

# Halt the running module over SWD and say where its PC is, in source terms.
# For when the panel has frozen and nothing else can tell you why. Needs the
# ST-Link attached and the ELF matching what is flashed.
where ELF="build-rel/BMCVFirmware.elf":
	./scripts/where.sh {{ELF}}

build-flash: build flash

# The same firmware at -O2. `just build` gives you a Debug build - CMakeLists.txt
# defaults CMAKE_BUILD_TYPE to Debug, and cmake/gcc-arm-none-eabi.cmake's
# `$<$<CONFIG:DEBUG>:-O0>` is appended after its global -O2, so -O0 wins and
# every TU is unoptimized. That is fine to step through and expensive to run:
# at -O0 the engine's hot path keeps fclamp, phase_mod, quantize_value and the
# rest as real calls with every local spilled, eight channels deep, and the main
# loop cannot hold its 4kHz tick - engine_fps and dac_fps read the same number
# because the tick is already late on arrival.
#
# RelWithDebInfo rather than Release: -O2 over -Os, and symbols kept, so this
# stays debuggable (stepping will jump around - use the Debug build for that).
# Its own directory, so `build/`, `just flash` and the compile_commands.json
# symlink clangd reads all stay pointing at the Debug build.
#
# Configure once, then `just build-flash-rel`.
configure-rel:
  rm -Rf build-rel && cmake -B build-rel -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo

build-rel:
	cmake --build build-rel

flash-rel:
	./scripts/flash.sh build-rel/BMCVFirmware.elf

build-flash-rel: build-rel flash-rel

# Native host-compiled test/tool build. Kept in its own directory so `build/`
# stays exactly what the ARM recipes, scripts/flash.sh and clangd expect.
# Re-run after editing tests/CMakeLists.txt or adding a test file.
configure-native:
	rm -Rf build-native && cmake -S tests -B build-native -G Ninja

test:
	cmake --build build-native
	ctest --test-dir build-native --output-on-failure

render *ARGS:
	cmake --build build-native --target render_channel
	./build-native/render_channel {{ARGS}}

# Native build of the simulator: the firmware's core behind a flat C API,
# plus a headless CLI that drives it from a scripted input timeline.
configure-sim:
	rm -Rf build-sim && cmake -S sim -B build-sim -G Ninja

sim:
	cmake --build build-sim

# Run an input script through the module. See sim/flows/ for examples and
# sim/src/bmcv_sim_cli.c for the script format.
#   just sim-run --script=sim/flows/demo_amp.txt --emit=all
sim-run *ARGS:
	cmake --build build-sim
	./build-sim/bmcv_sim_cli {{ARGS}}

# Replay every flow in sim/flows/ and diff against its committed golden file.
# Regenerate with `just flows-bless` after reviewing the change.
flows:
	cmake --build build-sim
	./sim/flows/run.sh check

flows-bless:
	cmake --build build-sim
	./sim/flows/run.sh bless

# WebAssembly build of the same simulator -> web/bmcv.js + web/bmcv.wasm.
# Needs emsdk; install once with `just wasm-sdk`, then source it in your shell
# (or let these recipes do it, which is why they run under bash).
EMSDK := env_var_or_default("EMSDK_DIR", env_var("HOME") / "emsdk")

wasm-sdk:
	bash -c '[ -d "{{EMSDK}}" ] || git clone --depth 1 https://github.com/emscripten-core/emsdk.git "{{EMSDK}}"'
	bash -c 'cd "{{EMSDK}}" && ./emsdk install latest && ./emsdk activate latest'

configure-wasm:
	rm -Rf build-wasm
	bash -c 'source "{{EMSDK}}/emsdk_env.sh" >/dev/null 2>&1 && emcmake cmake -S sim -B build-wasm -G Ninja'

wasm:
	bash -c 'source "{{EMSDK}}/emsdk_env.sh" >/dev/null 2>&1 && cmake --build build-wasm'

# Serve web/ so the browser will load the wasm module (file:// will not).
web PORT="8000": wasm
	@echo "http://localhost:{{PORT}}/"
	python3 -m http.server {{PORT}} --directory web

# Headless check that the wasm module loads and its exports behave, so a broken
# build is caught without opening a browser.
wasm-check: wasm
	node web/smoke.mjs

# The same idea one layer up: stands up enough of a DOM to import the whole
# frontend for real, against the real wasm and panel spec, and drives a few
# frames. Catches a missing export, a bad element id or a broken import graph
# without opening a browser.
web-check: wasm
	node web/frontend-check.mjs

# Serve the firmware updater at http://localhost:PORT/update/. No wasm
# dependency, deliberately: the updater is a separate page precisely so that a
# broken simulator build cannot take the thing that flashes the module with it.
update-page PORT="8000":
	@echo "http://localhost:{{PORT}}/update/"
	python3 -m http.server {{PORT}} --directory web

# Checks the DfuSe client against a fake device: command opcodes, data block
# numbering, the short final block, and what counts as a firmware image at all.
# The one piece of this project whose failure mode is a half-written module.
dfu-check:
	node web/update/dfu-check.mjs

# VCV Rack plugin. Same core as the firmware, compiled straight out of
# Core/Src/Lib - see vcv/Makefile.
#
# Every recipe here takes a Rack platform name and defaults it to this host's.
# The four with a published SDK are the whole list - lin-x64, win-x64, mac-x64,
# mac-arm64 - and scripts/vcv-build.sh, which all of them go through, explains
# why lin-arm64 is not one of them and which are cross builds.
#
#   just vcv                 build plugin.so/.dll for this host
#   just vcv-dist win-x64    build the distributable .vcvplugin for a platform
#   just vcv-install         unpack a host build into Rack's plugin directory
#
# CI's matrix calls vcv-dist for each of the four - see
# .github/workflows/vcv.yml - so nothing about a released artifact is
# reachable only from a workflow file.
#
# An empty ARCH means the host's, and is resolved inside the script rather than
# by a variable here. A `just` variable holding a backtick is evaluated on any
# invocation that reads it, and one holding this script's output would be read
# by `just build` and `just test` too - making every recipe in this file depend
# on the vcv build script being present and runnable.

# Fetch a Rack SDK without building anything, and say where it landed. Not a
# dependency of the recipes below - each fetches what it needs - so this is
# only for priming a machine, or for asking where an SDK is.
vcv-sdk ARCH="":
	./scripts/vcv-build.sh "{{ARCH}}" sdk

vcv ARCH="":
	./scripts/vcv-build.sh "{{ARCH}}"

# The zstd-compressed .vcvplugin Rack distributes, into vcv/dist/. This is what
# a release attaches.
vcv-dist ARCH="":
	./scripts/vcv-build.sh "{{ARCH}}" dist

vcv-clean ARCH="":
	./scripts/vcv-build.sh "{{ARCH}}" clean

# clangd needs the Rack SDK's include paths, which live outside the repo and
# differ per machine. Asking make for them keeps them out of a checked-in file
# - vcv/compile_commands.json is generated and ignored, like the CMake ones.
vcv-compdb ARCH="":
	RACK_DIR="$(./scripts/vcv-build.sh "{{ARCH}}" sdk)" \
	  python3 tools/gen_compdb.py --dir vcv --out vcv/compile_commands.json

# Through the SDK's own install target, which puts the .vcvplugin in the
# plugins-<os>-<cpu> directory Rack actually scans - a different place, and a
# different library name, on each of the four. Rack unpacks the package on the
# next start. Restart Rack afterwards; it only looks at startup.
vcv-install ARCH="":
	./scripts/vcv-build.sh "{{ARCH}}" install

# Straight into the Windows-side Rack over /mnt/c, which is the point of the
# win-x64 cross build: Rack runs on Windows, the checkout lives in WSL. The
# default guesses the one Windows user that has Rack installed; export
# RACK_WIN_DIR if that guess is wrong. Restart Rack afterwards - it only scans
# for plugins at startup.
RACK_WIN_DIR := env_var_or_default("RACK_WIN_DIR", "")

vcv-win-install: (vcv "win-x64")
	bash -c 'set -e; \
	  r="{{RACK_WIN_DIR}}"; \
	  [ -n "$r" ] || r=$(echo /mnt/c/Users/*/AppData/Local/Rack2 | cut -d" " -f1); \
	  [ -d "$r" ] || { echo "no Windows Rack2 directory found; export RACK_WIN_DIR" >&2; exit 1; }; \
	  d="$r/plugins-win-x64/BMCV"; mkdir -p "$d"; \
	  cp vcv/plugin.dll "$d/" 2>/dev/null || { \
	    echo "could not replace $d/plugin.dll." >&2; \
	    echo "Windows holds a loaded DLL open - close VCV Rack and run this again." >&2; \
	    exit 1; }; \
	  cp vcv/plugin.json "$d/" && cp -r vcv/res "$d/" && \
	  echo "installed to $d"'

# The screenshots in docs/. Needs a Chrome or Chromium; under WSL that is the
# Windows one, so export its path:
#   export CHROME="/mnt/c/Program Files/Google/Chrome/Application/chrome.exe"
CHROME := env_var_or_default("CHROME", "chromium")

docs-shots PORT="8123": wasm
	bash -c 'set -e; \
	  python3 -m http.server {{PORT}} --directory web >/dev/null 2>&1 & \
	  srv=$!; trap "kill $srv" EXIT; sleep 2; \
	  mkdir -p docs/images; \
	  tmp=$(mktemp -d); \
	  "{{CHROME}}" --headless --disable-gpu --hide-scrollbars \
	    --window-size=1500,1000 --virtual-time-budget=6000 \
	    --screenshot="$(command -v wslpath >/dev/null && wslpath -w "$tmp/shot.png" || echo "$tmp/shot.png")" \
	    "http://localhost:{{PORT}}/" >/dev/null 2>&1; \
	  cp "$tmp/shot.png" docs/images/web-overview.png; \
	  rm -rf "$tmp"'
	ffmpeg -loglevel error -i docs/images/web-overview.png -vf "crop=364:560:22:76" -y docs/images/web-panel.png
	@echo "wrote docs/images/web-overview.png, docs/images/web-panel.png"

# Everything that can be checked without hardware or a browser.
check: test flows wasm-check web-check dfu-check

# The same tests under AddressSanitizer and UBSan. Kept out of `check` because
# it is a second full build, but it is what catches the class of fault that
# reads or writes outside an array - the wavetable index off a NaN phase, for
# one - and the suite has been clean under it from the day it was added.
test-san:
	cmake -S tests -B build-san -G Ninja -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -g"
	cmake --build build-san
	UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 ctest --test-dir build-san --output-on-failure

# The firmware, with the toolchain and the STM32Cube FW package located by
# environment rather than by a checked-out toolchain.cmake. This is the form CI
# builds; locally `just build` and toolchain.cmake are the shorter path.
ARM_TOOLCHAIN_DIR := env_var_or_default("ARM_TOOLCHAIN_DIR", "/usr")
STM32CUBE_FW_PATH := env_var_or_default("STM32CUBE_FW_PATH", "")

build-ci:
	cmake -S . -B build-ci -G Ninja -DCMAKE_BUILD_TYPE=Release \
	  -DARM_TOOLCHAIN_DIR="{{ARM_TOOLCHAIN_DIR}}" \
	  -DSTM32CUBE_FW_PATH="{{STM32CUBE_FW_PATH}}"
	cmake --build build-ci
	@arm-none-eabi-size build-ci/BMCVFirmware.elf

# Formatting. The set is the hand-written C: everything else under Core/,
# USB_Device/ and the STM32Cube package is regenerated from the .ioc and would
# fail the moment anyone reopened the project, and the two big headers in
# Core/Inc/Lib come out of tools/. vcv/src is C++ in Rack's tab style and is
# left to it.
#
# clang-format's output moves between major versions; CI pins 18.
CLANG_FORMAT := env_var_or_default("CLANG_FORMAT", "clang-format")
FMT_PATHS := "'Core/Src/Lib/*.c' 'Core/Inc/Lib/*.h' 'sim/src/*.c' 'sim/include/*.h' 'tests/*.c' 'tests/*.h' 'tests/fixtures/*' 'tools/*.c' 'tools/*.h'"
# Generated files. They are formatted by whatever emits them, and a formatter
# rewriting one only guarantees the next `just wavetable` undoes it.
FMT_EXCLUDE := "wavetables\\.[ch]|stepped_random_table\\.h"

# --cached --others --exclude-standard, not a bare ls-files: a plain listing
# covers tracked files only, so a file you have just written is silently not
# checked - which is exactly when it has never been formatted. Locally that
# reads as a pass; CI sees it the moment it is committed and disagrees.
_fmt-files:
	@git ls-files --cached --others --exclude-standard {{FMT_PATHS}} | grep -vE '{{FMT_EXCLUDE}}'

fmt:
	just _fmt-files | xargs {{CLANG_FORMAT}} -i

fmt-check:
	just _fmt-files | xargs {{CLANG_FORMAT}} --dry-run --Werror

# Versioning. VERSION and CHANGELOG.md are driven by commits since the last
# tag, per .cz.toml - see version.h.in for why VERSION is the one file a bump
# has to touch. The Rack plugin's own version is not one of them and does not
# move; .cz.toml says why. Needs `pipx install commitizen` (or `pip install
# --user commitizen`) once; CI enforces the commit format this expects.
#
#   just commit       interactive wizard for a correctly formatted message
#   just bump-dry-run  preview what `just bump` would change, without changing it
#   just bump          bump VERSION + CHANGELOG.md, commit, tag - then push both:
#                       git push && git push --tags
commit:
	cz commit

bump-dry-run:
	cz bump --changelog --dry-run

bump:
	cz bump --changelog

# The unattended form CI's release job runs after a merge to main: --yes skips
# the "is this the first tag?" prompt bump asks interactively otherwise.
bump-ci:
	cz bump --changelog --yes

# Regenerate the SHAPE_LFO wavetable: Core/Inc/Lib/wavetables.h, its data in
# Core/Src/Lib/wavetables.c, and the plots in docs/images/. All four come out of
# one run, so the pictures cannot show a shape the firmware does not have.
# Output is checked in, so this only needs running after editing the shape
# family in the generator - review the diff, then `just test`. `--report`
# prints the table as sparklines without writing anything.
wavetable *ARGS:
	python3 tools/gen_wavetable.py {{ARGS}}

# Regenerate the stepped-random tables into Core/Inc/Lib/stepped_random_table.h.
# Output is checked in, so this only needs running after editing the generator -
# review the diff, then `just test` to confirm the pattern properties still hold.
sr-table:
	cc -O2 -o build-native/gen_sr_table tools/gen_sr_table.c -lm
	./build-native/gen_sr_table > Core/Inc/Lib/stepped_random_table.h

# Regenerate the panel layout from the PCB project's KiCad output plus the
# firmware's own HwSetup tables. Outputs are checked in, so this only needs
# running after editing hw_setup.c or when the board changes - review the diff.
# Point HW_REPO at the PCB project directory if it is not pcb/.
panel HW_REPO="pcb":
	cmake --build build-native --target dump_hw_setup
	python3 tools/gen_panel_spec.py --hw-repo {{HW_REPO}}
