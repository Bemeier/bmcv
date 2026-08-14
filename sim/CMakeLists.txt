# Standalone project for the BMCV simulator: the firmware's core logic wrapped
# in a flat C API, plus a headless CLI that drives it from a scripted input
# timeline.
#
# Independent of the top-level CMakeLists.txt (ARM-only) and of tests/, so it
# can be configured with the system compiler or with emcmake for wasm:
#
#   cmake -S sim -B build-sim -G Ninja        # native, via `just sim`
#   emcmake cmake -S sim -B build-wasm ...    # wasm, via `just wasm`
cmake_minimum_required(VERSION 3.22)
project(bmcv_sim C)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/require_modern_c.cmake)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BMCV_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(BMCV_LIB_SRC ${BMCV_ROOT}/Core/Src/Lib)
set(BMCV_LIB_INC ${BMCV_ROOT}/Core/Inc/Lib)

# The same list the firmware and the test build use, so the three cannot drift.
include(${BMCV_ROOT}/cmake/core_sources.cmake)
list(TRANSFORM BMCV_CORE_SOURCES PREPEND ${BMCV_LIB_SRC}/)

add_library(bmcv_sim STATIC
  ${BMCV_CORE_SOURCES}
  src/bmcv_sim.c
  src/sim_rt.c
  src/slot_store.c
)
target_include_directories(bmcv_sim PUBLIC ${BMCV_LIB_INC} ${BMCV_ROOT}/panel ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(bmcv_sim PUBLIC m)
target_compile_options(bmcv_sim PRIVATE -Wall -Wextra -Wmissing-prototypes)

if(EMSCRIPTEN)
  # web/bmcv.js + web/bmcv.wasm, loaded as an ES module by web/main.js.
  #
  # The function list is explicit rather than relying on EMSCRIPTEN_KEEPALIVE:
  # bmcv_sim is a static library, so anything not named here is dead-stripped
  # before it can be exported. Keep it in step with sim/include/bmcv_sim.h.
  set(BMCV_WASM_EXPORTS
      _bmcv_sim_create
      _bmcv_sim_destroy
      _bmcv_sim_reset
      _bmcv_sim_set_button
      _bmcv_sim_add_encoder
      _bmcv_sim_set_slider01
      _bmcv_sim_set_cv
      _bmcv_sim_fire_gate
      _bmcv_sim_remote_button
      _bmcv_sim_remote_encoder
      _bmcv_sim_remote_slider01
      _bmcv_sim_remote_clear
      _bmcv_sim_remote_offset
      _bmcv_sim_remote_size
      _bmcv_sim_remote_blob
      _bmcv_sim_sysex7_encoded_len
      _bmcv_sim_sysex7_decoded_len
      _bmcv_sim_sysex7_encode
      _bmcv_sim_sysex7_decode
      _bmcv_sim_run
      _bmcv_sim_now_us
      _bmcv_sim_outputs_v
      _bmcv_sim_leds_rgb
      _bmcv_sim_scope
      _bmcv_sim_input_scope
      _bmcv_sim_effective
      _bmcv_sim_scope_head
      _bmcv_sim_shift_state
      _bmcv_sim_mode_name
      _bmcv_sim_mode_count
      _bmcv_sim_engine_fps
      _bmcv_sim_dac_fps
      _bmcv_sim_led_fps
      _bmcv_sim_selected_param
      _bmcv_sim_active_scene
      _bmcv_sim_bpm
      _bmcv_sim_active_bpm
      _bmcv_sim_have_beat
      _bmcv_sim_scene_contribution
      _bmcv_sim_channel_muted
      _bmcv_sim_error_flags
      _bmcv_sim_channel_param
      _bmcv_sim_encoder_pos
      _bmcv_sim_slider01
      _bmcv_sim_channel_shape_mode
      _bmcv_sim_shape_mode_name
      _bmcv_sim_midi_drain
      _bmcv_sim_instance_size
      _bmcv_sim_export
      _bmcv_sim_import
      _bmcv_sim_storage_size
      _bmcv_sim_storage_get
      _bmcv_sim_storage_set
      _malloc
      _free)
  list(JOIN BMCV_WASM_EXPORTS "," BMCV_WASM_EXPORTS_CSV)

  add_executable(bmcv_wasm src/wasm_stub.c)
  target_link_libraries(bmcv_wasm PRIVATE bmcv_sim)
  set_target_properties(bmcv_wasm PROPERTIES
    OUTPUT_NAME bmcv
    SUFFIX ".js"
    RUNTIME_OUTPUT_DIRECTORY ${BMCV_ROOT}/web)
  target_link_options(bmcv_wasm PRIVATE
    --no-entry
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createBmcv
    -sENVIRONMENT=web
    -sALLOW_MEMORY_GROWTH=1
    -sEXPORTED_FUNCTIONS=${BMCV_WASM_EXPORTS_CSV}
    -sEXPORTED_RUNTIME_METHODS=HEAPU8,HEAPF32,HEAPU32,UTF8ToString
    -O3)
else()
  add_executable(bmcv_sim_cli src/bmcv_sim_cli.c)
  target_link_libraries(bmcv_sim_cli PRIVATE bmcv_sim)
  target_compile_options(bmcv_sim_cli PRIVATE -Wall -Wextra)
endif()
