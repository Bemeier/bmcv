set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PREFIX arm-none-eabi)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)

set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_SIZE    ${TOOLCHAIN_PREFIX}-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

add_compile_options(
    ${MCU_FLAGS}
    -Wall
    -Wextra
    -Wpedantic
    -ffunction-sections
    -fdata-sections
    -O2
)

add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
)

# -g rather than -g0 in Release, because the ELF is a release artifact and a
# debugger attached to a released module is worth more than a smaller file that
# nothing loads. It costs the firmware nothing: -g only adds .debug_* sections
# to the ELF, and objcopy leaves them behind when it extracts the .bin - the
# image is byte-identical either way, which is what makes this safe to carry in
# the build CI releases from.
add_compile_options(
    $<$<CONFIG:DEBUG>:-O0>
    $<$<CONFIG:DEBUG>:-g3>
    $<$<CONFIG:RELEASE>:-Os>
    $<$<CONFIG:RELEASE>:-g>
)

add_link_options(
    ${MCU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/STM32G474XX_FLASH.ld
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--print-memory-usage
)

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb")
set(FPU_FLAGS "-mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_COMPILER ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
# The compiler was re-pointed at ARM_TOOLCHAIN_DIR above; these were not, so
# linking worked (arm-none-eabi-gcc is an absolute path) and the objcopy step
# right after it silently needed ARM_TOOLCHAIN_DIR/bin on PATH too - true by
# accident for anyone whose IDE already put a toolchain there, not otherwise.
set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-objcopy)
set(CMAKE_SIZE ${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-size)
set(CMAKE_SYSROOT ${ARM_TOOLCHAIN_DIR}/arm-none-eabi)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

