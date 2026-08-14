cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_PROJECT_NAME BMCVFirmware)
project(${CMAKE_PROJECT_NAME} C ASM)


if(EXISTS "${CMAKE_SOURCE_DIR}/toolchain.cmake")
  message(STATUS "Including user toolchain.cmake")
  include("${CMAKE_SOURCE_DIR}/toolchain.cmake")
endif()

include("cmake/gcc-arm-none-eabi.cmake")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message("Build type: " ${CMAKE_BUILD_TYPE})

# Executable target
add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

# version.h is generated from VERSION rather than checked in, so a `cz bump`
# (see .cz.toml) can never drift from what midi.c reports over SysEx - there
# is exactly one file for it to touch.
#
# The dependency is not implicit. file(STRINGS) reads VERSION now, at configure
# time, and tells CMake nothing about having done so - so a bump left every
# later `just build-rel` re-emitting the version that was current when the build
# directory was first configured. The module went on reporting 0.9.0 over SysEx
# after the 0.10.0 bump, and would have gone on doing it until someone
# reconfigured for an unrelated reason. Naming VERSION here is what makes a bump
# re-run this block.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/VERSION")
file(STRINGS "${CMAKE_SOURCE_DIR}/VERSION" BMCV_VERSION LIMIT_COUNT 1)
if(NOT BMCV_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)")
  message(FATAL_ERROR "VERSION must contain MAJOR.MINOR.PATCH, got: '${BMCV_VERSION}'")
endif()
set(FW_VERSION_MAJOR ${CMAKE_MATCH_1})
set(FW_VERSION_MINOR ${CMAKE_MATCH_2})
set(FW_VERSION_PATCH ${CMAKE_MATCH_3})
configure_file(
  Core/Inc/Lib/version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/version.h
  @ONLY
)

# Source lists are shared with the native test build so a new Lib file only
# has to be declared once - see cmake/core_sources.cmake
include(cmake/core_sources.cmake)
list(TRANSFORM BMCV_CORE_SOURCES PREPEND Core/Src/Lib/)
list(TRANSFORM BMCV_DRIVER_SOURCES PREPEND Core/Src/Lib/)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.c
  ${BMCV_CORE_SOURCES}
  ${BMCV_DRIVER_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Inc
  Core/Inc/Lib
  # ui_sparkle.c samples the LEDs' real positions on the board, which `just
  # panel` generates from the same source as the panel artwork.
  panel
  ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    stm32cubemx
    STM32_Drivers
    USB_Device_Library
)

add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin"
)

include(${CMAKE_SOURCE_DIR}/local.cmake OPTIONAL)

