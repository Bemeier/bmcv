cmake_minimum_required(VERSION 3.22)

# Compiler settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=bugprone-*,performance-*,portability-*")

if(EXISTS "${CMAKE_SOURCE_DIR}/toolchain.cmake")
  message(STATUS "Including user toolchain.cmake")
  include("${CMAKE_SOURCE_DIR}/toolchain.cmake")
endif()

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME BMCVFirmware)

# Toolchain
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(COMPILE_COMMANDS_SRC "${CMAKE_BINARY_DIR}/compile_commands.json")
set(COMPILE_COMMANDS_DST "${CMAKE_SOURCE_DIR}/compile_commands.json")

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

# Executable target
add_executable(${CMAKE_PROJECT_NAME})

# Include STM32CubeMX subproject — adds drivers, libs, includes, defines
add_subdirectory(cmake/stm32cubemx)

# Optional: Your custom sources
file(GLOB MY_DRIVER_SOURCES
    Core/Src/Drivers/*.c
    Core/Src/Lib/*.c
)

file(GLOB USB_MIDI_SOURCES
    Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/*.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MY_DRIVER_SOURCES}
    ${USB_MIDI_SOURCES}
)

# Optional: Your custom includes
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Inc
    Core/Inc/Drivers
    Core/Inc/Lib
)

# Optional: Add your own defines
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user-defined macros here
)

# Optional: Add your own linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${COMPILE_COMMANDS_SRC}" "${COMPILE_COMMANDS_DST}"
    COMMENT "Copy compile_commands.json to project root for clangd"
    VERBATIM
)

add_dependencies(copy_compile_commands ${CMAKE_PROJECT_NAME})