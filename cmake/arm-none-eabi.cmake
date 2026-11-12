# ARM GNU Toolchain for STM32H723 (Cortex-M7).
#
# Usage:
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#
# The toolchain is looked up on PATH first. If it is not installed system-wide,
# set ARM_TOOLCHAIN_PATH to the directory that contains bin/arm-none-eabi-gcc:
#   -DARM_TOOLCHAIN_PATH=$HOME/.local/arm-gnu-toolchain/bin

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(DEFINED ENV{ARM_TOOLCHAIN_PATH} AND NOT DEFINED ARM_TOOLCHAIN_PATH)
    set(ARM_TOOLCHAIN_PATH "$ENV{ARM_TOOLCHAIN_PATH}")
endif()

if(ARM_TOOLCHAIN_PATH)
    set(ARM_NONE_EABI_GCC "${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc")
else()
    find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc REQUIRED)
endif()

if(NOT EXISTS "${ARM_NONE_EABI_GCC}")
    message(FATAL_ERROR "arm-none-eabi-gcc not found at ${ARM_NONE_EABI_GCC}")
endif()

get_filename_component(ARM_TOOLCHAIN_BIN "${ARM_NONE_EABI_GCC}" DIRECTORY)
set(ARM_TOOLCHAIN_PREFIX "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-")

set(CMAKE_C_COMPILER   "${ARM_TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_PREFIX}gcc")
set(CMAKE_AR           "${ARM_TOOLCHAIN_PREFIX}ar")
set(CMAKE_OBJCOPY      "${ARM_TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP      "${ARM_TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE         "${ARM_TOOLCHAIN_PREFIX}size")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
