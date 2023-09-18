cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040-sd-emu C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040-sd-emu main.c)

# Pull in basic dependencies
target_link_libraries(rp2040-sd-emu pico_stdlib hardware_spi)

pico_enable_stdio_usb(rp2040-sd-emu 1)
pico_enable_stdio_uart(rp2040-sd-emu 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(rp2040-sd-emu)