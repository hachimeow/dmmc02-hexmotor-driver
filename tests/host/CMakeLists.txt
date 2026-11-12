cmake_minimum_required(VERSION 3.20)

add_executable(test_meow_codec
    test_meow_codec.c
    ${CMAKE_SOURCE_DIR}/firmware/drivers/meow/meow_codec.c
    ${CMAKE_SOURCE_DIR}/firmware/drivers/meow/meow_types.c)
target_include_directories(test_meow_codec PRIVATE
    ${CMAKE_SOURCE_DIR}/firmware/drivers/meow)
target_link_libraries(test_meow_codec PRIVATE m)

add_test(NAME meow_codec COMMAND test_meow_codec)
