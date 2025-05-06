# Copyright (c) 2025 Hirokuni Yano
#
# Released under the MIT license.
# see https://opensource.org/licenses/MIT
#
cmake_minimum_required(VERSION 3.13)

#set(PICO_PLATFORM rp2040)
#set(PICO_BOARD pico)

include(pico_sdk_import.cmake)

project(rp3802 C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp3802
  rp3802.cpp
)

#pico_set_binary_type(rp3802 copy_to_ram)

pico_enable_stdio_usb(rp3802 1)
pico_enable_stdio_uart(rp3802 0)

pico_generate_pio_header(rp3802 ${CMAKE_CURRENT_SOURCE_DIR}/rp3802.pio)
#pico_generate_pio_header(rp3802 ${CMAKE_CURRENT_SOURCE_DIR}/busmon.pio)

pico_add_extra_outputs(rp3802)

target_link_libraries(rp3802 pico_stdlib pico_multicore pico_bootrom hardware_pio hardware_dma hardware_flash hardware_sync hardware_watchdog hardware_uart)

#pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
#target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
#pico_set_boot_stage2(rp3802 slower_boot2)
