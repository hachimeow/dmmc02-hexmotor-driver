cmake_minimum_required(VERSION 3.20)

project(dmmc02_hexmotor LANGUAGES C ASM)

# Host-side unit tests are only built for native builds.
if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(tests/host)
    return()
endif()

add_subdirectory(firmware)
