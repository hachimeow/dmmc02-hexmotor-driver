set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(ROOT_DIR "${CMAKE_SOURCE_DIR}")
set(FW_DIR   "${ROOT_DIR}/firmware")

set(MCU_FLAGS
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard)

set(STM32H7_DIR "${ROOT_DIR}/third_party/stm32h7")
set(HAL_DIR     "${STM32H7_DIR}/STM32H7xx_HAL_Driver")
set(CMSIS_DEV   "${STM32H7_DIR}/CMSIS_Device/ST/STM32H7xx")
set(CMSIS_INC   "${STM32H7_DIR}/CMSIS_Include")

set(HAL_SOURCES
    ${HAL_DIR}/Src/stm32h7xx_hal.c
    ${HAL_DIR}/Src/stm32h7xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_dma.c
    ${HAL_DIR}/Src/stm32h7xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_exti.c
    ${HAL_DIR}/Src/stm32h7xx_hal_fdcan.c
    ${HAL_DIR}/Src/stm32h7xx_hal_flash.c
    ${HAL_DIR}/Src/stm32h7xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_spi.c
    ${HAL_DIR}/Src/stm32h7xx_hal_spi_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart_ex.c)

set(BOARD_SOURCES
    ${FW_DIR}/platform/startup_stm32h723vgtx.s
    ${FW_DIR}/platform/system_stm32h7xx.c
    ${FW_DIR}/platform/stm32h7xx_it.c
    ${FW_DIR}/platform/syscalls.c
    ${FW_DIR}/platform/sysmem.c
    ${FW_DIR}/bsp/board.c
    ${FW_DIR}/bsp/board_clock.c
    ${FW_DIR}/bsp/board_uart.c
    ${FW_DIR}/bsp/board_fdcan.c
    ${FW_DIR}/bsp/board_led.c)

set(DRIVER_SOURCES
    ${FW_DIR}/drivers/canfd/canfd.c
    ${FW_DIR}/drivers/canopen/canopen.c
    ${FW_DIR}/drivers/meow/meow_types.c
    ${FW_DIR}/drivers/meow/meow_codec.c
    ${FW_DIR}/drivers/meow/meow_motor.c
    ${FW_DIR}/drivers/meow/meow_bus.c)

set(APP_SOURCES
    ${FW_DIR}/app/main.c
    ${FW_DIR}/app/demo.c)

set(ALL_SOURCES
    ${BOARD_SOURCES}
    ${DRIVER_SOURCES}
    ${APP_SOURCES}
    ${HAL_SOURCES})

add_executable(dmmc02_hexmotor.elf ${ALL_SOURCES})

target_include_directories(dmmc02_hexmotor.elf PRIVATE
    ${FW_DIR}/platform
    ${FW_DIR}/bsp
    ${FW_DIR}/drivers/canfd
    ${FW_DIR}/drivers/canopen
    ${FW_DIR}/drivers/meow
    ${FW_DIR}/app
    ${CMSIS_DEV}/Include
    ${CMSIS_INC}
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy)

option(MEOW_DEMO_MIT "Enable the zero-torque MIT mode self-test" OFF)
option(MEOW_DEMO_PVPP "Enable the one-shot PV/PP motion self-test" OFF)
option(MEOW_DEBUG_OD "Probe the motor object dictionary once" OFF)
option(MEOW_DEMO_CIA_PP "Test Profile Position via CiA402 objects" OFF)
set(MEOW_TEST_NODE "2" CACHE STRING "Motor node used by one-shot tests")
set_property(CACHE MEOW_TEST_NODE PROPERTY STRINGS 1 2)

# Optional: extracted Debian/Ubuntu toolchain packages keep newlib outside the
# compiler's default search path. The official ARM tarball does not need this.
if(ARM_TOOLCHAIN_NEWLIB_DIR)
    target_compile_options(dmmc02_hexmotor.elf PRIVATE
        -B${ARM_TOOLCHAIN_NEWLIB_DIR})
    target_link_options(dmmc02_hexmotor.elf PRIVATE
        -B${ARM_TOOLCHAIN_NEWLIB_DIR})
endif()
if(ARM_TOOLCHAIN_NEWLIB_INCLUDE)
    target_compile_options(dmmc02_hexmotor.elf PRIVATE
        -isystem ${ARM_TOOLCHAIN_NEWLIB_INCLUDE})
endif()

target_compile_definitions(dmmc02_hexmotor.elf PRIVATE
    USE_HAL_DRIVER
    STM32H723xx
    HSE_VALUE=24000000U
    MEOW_HOST_NODE_ID=0x10U
    MEOW_DEMO_MIT=$<BOOL:${MEOW_DEMO_MIT}>
    MEOW_DEMO_PVPP=$<BOOL:${MEOW_DEMO_PVPP}>
    MEOW_DEBUG_OD=$<BOOL:${MEOW_DEBUG_OD}>
    MEOW_DEMO_CIA_PP=$<BOOL:${MEOW_DEMO_CIA_PP}>
    MEOW_TEST_NODE=${MEOW_TEST_NODE})

target_compile_options(dmmc02_hexmotor.elf PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-Os -g>)

set(LINKER_SCRIPT "${FW_DIR}/platform/STM32H723VGTX_FLASH.ld")

set_target_properties(dmmc02_hexmotor.elf PROPERTIES
    LINK_DEPENDS "${LINKER_SCRIPT}")

target_link_options(dmmc02_hexmotor.elf PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_BINARY_DIR}/dmmc02_hexmotor.map
    --specs=nano.specs
    --specs=nosys.specs
    -u _printf_float)

set(HEX_FILE "${CMAKE_BINARY_DIR}/dmmc02_hexmotor.hex")
set(BIN_FILE "${CMAKE_BINARY_DIR}/dmmc02_hexmotor.bin")

add_custom_command(TARGET dmmc02_hexmotor.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:dmmc02_hexmotor.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:dmmc02_hexmotor.elf> ${BIN_FILE}
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:dmmc02_hexmotor.elf>
    COMMENT "Generating ${HEX_FILE} and ${BIN_FILE}")
