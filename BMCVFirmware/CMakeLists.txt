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

