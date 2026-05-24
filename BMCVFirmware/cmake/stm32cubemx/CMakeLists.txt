cmake_minimum_required(VERSION 3.22)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32G474xx
    $<$<CONFIG:Debug>:DEBUG>
)

message(STATUS "STM32CUBE_FW_PATH = '${STM32CUBE_FW_PATH}'")

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_SOURCE_DIR}/USB_Device/App
    ${CMAKE_SOURCE_DIR}/USB_Device/Target
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Inc
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${STM32CUBE_FW_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${STM32CUBE_FW_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc
    ${STM32CUBE_FW_PATH}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${STM32CUBE_FW_PATH}/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_SOURCE_DIR}/USB_Device/App/usb_device.c
    ${CMAKE_SOURCE_DIR}/USB_Device/App/usbd_desc.c
    ${CMAKE_SOURCE_DIR}/USB_Device/Target/usbd_conf.c
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32g4xx_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32g4xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/startup_stm32g474xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32g4xx.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ${STM32CUBE_FW_PATH}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
)

# Drivers Midllewares


set(USB_Device_Library_Src
    ${STM32CUBE_FW_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${STM32CUBE_FW_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${STM32CUBE_FW_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${STM32CUBE_FW_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c
)


# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx SYSTEM INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

add_library(STM32_Drivers OBJECT ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

add_library(USB_Device_Library OBJECT ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

