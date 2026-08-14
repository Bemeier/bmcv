# Standalone native (host-compiled) project for the BMCVFirmware DSP/logic
# layer. Deliberately independent of the top-level CMakeLists.txt (which is
# ARM-toolchain-specific via cmake/gcc-arm-none-eabi.cmake) so this can be
# configured with the plain system compiler:
#
#   cmake -S tests -B build/Native -G Ninja
#   cmake --build build/Native
#   ctest --test-dir build/Native
#
# or via `just test` / `just render` (see ../Justfile).
cmake_minimum_required(VERSION 3.22)
project(bmcv_native C)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/require_modern_c.cmake)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # so clangd can index tests/ and tools/

set(BMCV_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(BMCV_LIB_SRC ${BMCV_ROOT}/Core/Src/Lib)
set(BMCV_LIB_INC ${BMCV_ROOT}/Core/Inc/Lib)

# The real DSP/logic sources, compiled as-is - no copies, no #ifdefs.
# Same list the firmware builds, so the two cannot drift apart.
include(${BMCV_ROOT}/cmake/core_sources.cmake)
list(TRANSFORM BMCV_CORE_SOURCES PREPEND ${BMCV_LIB_SRC}/)

add_library(bmcv_core STATIC
  ${BMCV_CORE_SOURCES}
)
# panel/ for panel_layout.h - see the note in the top-level CMakeLists.txt.
target_include_directories(bmcv_core PUBLIC ${BMCV_LIB_INC} ${BMCV_ROOT}/panel)
target_link_libraries(bmcv_core PUBLIC m)
# -Wmissing-prototypes catches a non-static function with no declaration, which
# is how three dead or internal functions here ended up externally visible.
target_compile_options(bmcv_core PRIVATE -Wall -Wextra -Wmissing-prototypes)

add_library(bmcv_fixture STATIC fixtures/fixture.c)
target_include_directories(bmcv_fixture PUBLIC fixtures)
target_link_libraries(bmcv_fixture PUBLIC bmcv_core)

enable_testing()

function(bmcv_add_test name)
  add_executable(${name} ${name}.c)
  target_link_libraries(${name} PRIVATE bmcv_fixture)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME ${name} COMMAND ${name})
endfunction()

bmcv_add_test(test_helpers)
bmcv_add_test(test_stepped_random)
bmcv_add_test(test_wavetable)
target_sources(test_wavetable PRIVATE pll_metrics.c)
bmcv_add_test(test_channel)
bmcv_add_test(test_clock_sync)
bmcv_add_test(test_scene)
bmcv_add_test(test_quantizer)
bmcv_add_test(test_assign)
bmcv_add_test(test_led_render)
bmcv_add_test(test_config_migrate)
bmcv_add_test(test_config_validate)
bmcv_add_test(test_ui_input)
bmcv_add_test(test_ui_gestures)
bmcv_add_test(test_ui_select)
bmcv_add_test(test_ui_scene)
bmcv_add_test(test_ui_channel)
bmcv_add_test(test_ui_feedback)
bmcv_add_test(test_ui_mode)
bmcv_add_test(test_input_fold)
bmcv_add_test(test_mcp_decode)
bmcv_add_test(test_sysex)
bmcv_add_test(test_midi_realtime)
bmcv_add_test(test_midi_out)

# Characterises the phase-lock loop: settling, overshoot, ringing, frequency
# slew and long-run alignment. pll_metrics.c is the measurement side, kept out
# of the fixture library because nothing else needs it.
bmcv_add_test(test_pll)
target_sources(test_pll PRIVATE pll_metrics.c)
bmcv_add_test(test_instance)

# Host-side runtime glue from sim/. Built here so `just test` covers it too -
# it is shared by every host, so a regression would be everywhere at once.
bmcv_add_test(test_sim_rt)
target_sources(test_sim_rt PRIVATE ${BMCV_ROOT}/sim/src/sim_rt.c)

bmcv_add_test(test_slot_store)
target_sources(test_slot_store PRIVATE ${BMCV_ROOT}/sim/src/slot_store.c)
target_include_directories(test_slot_store PRIVATE ${BMCV_ROOT}/sim/include)
target_include_directories(test_sim_rt PRIVATE ${BMCV_ROOT}/sim/include)

# The flat C API itself, exercised where a debugger can reach it. Covers the
# snapshot path a debug probe uses - see docs/plans/probe-bridge.md - which is
# the one place a BmcvInstance arrives as bytes rather than being built in
# place, and so the one place its internal pointers can be left dangling.
bmcv_add_test(test_sim_import)
target_sources(test_sim_import PRIVATE
  ${BMCV_ROOT}/sim/src/bmcv_sim.c
  ${BMCV_ROOT}/sim/src/sim_rt.c
  ${BMCV_ROOT}/sim/src/slot_store.c
)
target_include_directories(test_sim_import PRIVATE ${BMCV_ROOT}/sim/include)

# Checks the generated panel layout against the firmware's index tables.
# Needs panel/panel_layout.h, which `just panel` produces.
bmcv_add_test(test_panel_spec)
target_include_directories(test_panel_spec PRIVATE ${BMCV_ROOT}/panel)
target_link_libraries(test_panel_spec PRIVATE m)


add_executable(render_channel ${BMCV_ROOT}/tools/render_channel.c)
target_link_libraries(render_channel PRIVATE bmcv_fixture)
target_include_directories(render_channel PRIVATE ${BMCV_ROOT}/tools)

# Feeds gen_panel_spec.py the firmware's own index tables, so the generated
# panel spec cannot drift from hw_setup.c. See `just panel`.
add_executable(dump_hw_setup ${BMCV_ROOT}/tools/dump_hw_setup.c)
target_link_libraries(dump_hw_setup PRIVATE bmcv_core)
