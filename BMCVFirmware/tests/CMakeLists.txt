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
  fakes/fake_drivers.c
)
target_include_directories(bmcv_core PUBLIC ${BMCV_LIB_INC})
target_link_libraries(bmcv_core PUBLIC m)
target_compile_options(bmcv_core PRIVATE -Wall -Wextra)

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

bmcv_add_test(test_stepped_random)
bmcv_add_test(test_channel)
bmcv_add_test(test_clock_sync)
bmcv_add_test(test_scene)
bmcv_add_test(test_quantizer)
bmcv_add_test(test_assign)
bmcv_add_test(test_led_render)
bmcv_add_test(test_config_validate)
bmcv_add_test(test_ui_input)
bmcv_add_test(test_ui_gestures)
bmcv_add_test(test_ui_select)
bmcv_add_test(test_ui_feedback)
bmcv_add_test(test_ui_mode)


add_executable(render_channel ${BMCV_ROOT}/tools/render_channel.c)
target_link_libraries(render_channel PRIVATE bmcv_fixture)
target_include_directories(render_channel PRIVATE ${BMCV_ROOT}/tools)
