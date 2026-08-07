# VCV Rack plugin: the BMCV firmware's own core, in a Rack module.
#
# Nothing here is a copy of the firmware. The core sources are compiled
# straight out of Core/Src/Lib, from the same list the ARM build, tests/ and
# sim/ use, so the four cannot drift apart.
#
#   make -C vcv            (or `just vcv`)
#
# RACK_DIR points at an unpacked Rack SDK - `just vcv-sdk` fetches one.
# BMCV_ROOT is the indirection that lets this directory become its own repo
# later: the VCV Library requires a plugin at the root of its repository, so
# the eventual move is a submodule plus a changed default here.

RACK_DIR  ?= $(HOME)/Rack-SDK
BMCV_ROOT ?= ..

# The single source of truth for which Lib sources are hardware-free, parsed
# rather than retyped. A new core file is picked up here the moment it is added
# to the CMake list; one that is missing fails at link, not at runtime.
CORE_NAMES := $(shell sed -n '/^set(BMCV_CORE_SOURCES/,/^)/p' \
                        $(BMCV_ROOT)/cmake/core_sources.cmake | grep -o '[a-z_]*\.c')

# Sources living outside this directory cannot go in SOURCES: compile.mk maps a
# source to build/<path>.o, and a leading ../ would put the object outside
# build/ where `make clean` will not find it. They get their own rule below,
# and OBJECTS is appended to by compile.mk rather than overwritten, so setting
# it here is enough.
CORE_SOURCES := $(addprefix $(BMCV_ROOT)/Core/Src/Lib/,$(CORE_NAMES)) \
                $(BMCV_ROOT)/sim/src/sim_rt.c \
                $(BMCV_ROOT)/sim/src/slot_store.c

CORE_OBJECTS := $(patsubst %, build/ext/%.o, $(notdir $(CORE_SOURCES)))
OBJECTS := $(CORE_OBJECTS)

SOURCES := $(wildcard src/*.cpp)

DISTRIBUTABLES += res

FLAGS += -I$(BMCV_ROOT)/Core/Inc/Lib -I$(BMCV_ROOT)/sim/include -I$(BMCV_ROOT)/panel

# The core is C23 under CMake. GCC only spells that -std=gnu23 from 14 on;
# gnu2x is the same dialect under the older name and is what GCC 13 accepts.
EXTRA_CFLAGS += -std=gnu2x

include $(RACK_DIR)/plugin.mk

# Checked directly against the construct that actually needs C23 - bare
# static_assert, no <assert.h> - rather than against a compiler version
# number, which would have to special-case Clang defining __GNUC__ too (see
# the gnu2x note above for why this needs checking at all). Left unchecked,
# `make -j` runs every core source in parallel and buries the one real
# problem under a wall of "expected declaration specifiers" from each.
ifneq ($(shell echo 'static_assert(1, "x");' | $(CC) -std=gnu2x -x c -c -o /dev/null - 2>&1),)
$(error $(CC) cannot compile a bare C23 static_assert - too old. Core/Inc/Lib/hw_setup.h \
  uses it as a keyword, not the <assert.h> macro. GCC needs 13+; Ubuntu 24.04 and Debian 13 \
  ship that by default. Install a newer compiler alongside the system one and pass CC=<path> \
  - see docs/setup.md)
endif

# compile.mk hardcodes build/, so a Linux build and a cross-compiled Windows
# build would drop their objects on top of each other and fail at link with
# something unhelpful about file formats. Wipe the tree when the target
# changes; ARCH_NAME is only known after the include above.
ARCH_STAMP := build/.arch-$(ARCH_NAME)
$(shell [ -e $(ARCH_STAMP) ] || rm -rf build)
$(shell mkdir -p build && touch $(ARCH_STAMP))

ifdef ARCH_WIN
# plugin.mk links libstdc++ statically but not libgcc, which leaves the DLL
# importing libgcc_s_seh-1.dll - a MinGW runtime file that exists on the
# machine that built it and nowhere else. Rack loads the plugin, Windows
# cannot resolve the import, and the module simply never appears.
LDFLAGS += -static-libgcc
endif

# compile.mk turns on -funsafe-math-optimizations for everything. That is fine
# for widget code and not fine for the engine: the firmware and its tests are
# built without it, and a Rack module that rounds differently from the module
# it is meant to be a reference for is worse than a slower one.
CORE_CFLAGS := $(filter-out -funsafe-math-optimizations,$(CFLAGS)) -fno-unsafe-math-optimizations

vpath %.c $(sort $(dir $(CORE_SOURCES)))

build/ext/%.c.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CORE_CFLAGS) -c -o $@ $<

-include $(CORE_OBJECTS:.o=.d)
