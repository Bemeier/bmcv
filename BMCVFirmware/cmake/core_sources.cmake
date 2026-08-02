# Single source of truth for which Lib sources are hardware-free.
#
# BMCV_CORE_SOURCES compile with a plain host compiler and are built by both
# the ARM firmware and the native test/tool build (tests/CMakeLists.txt).
# BMCV_DRIVER_SOURCES touch the STM32 HAL or a peripheral and are ARM-only.
#
# Filenames only - each consumer prepends its own path.

set(BMCV_CORE_SOURCES
    assign.c
    channel.c
    clock_sync.c
    config_validate.c
    ctrl_button.c
    engine.c
    error.c
    hw_setup.c
    input_fold.c
    instance.c
    led_fb.c
    quantizer.c
    scene.c
    stepped_random.c
    ui_channel.c
    ui_feedback.c
    ui_input.c
    ui_mode.c
    ui_render.c
    ui_scene.c
    ui_select.c
    ux_setup.c
    ux_state.c
    wave_fn.c
)

set(BMCV_DRIVER_SOURCES
    bmcv.c
    dac_adc.c
    fram.c
    mcp.c
    midi.c
    presets.c
    ws2811.c
)
