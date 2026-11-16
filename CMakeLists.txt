# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(n91_hdd_emulator C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(n91_hdd_emulator
    main.cpp
)

# Автоматическая компиляция .pio в заголовочные файлы .pio.h
pico_generate_pio_header(n91_hdd_emulator ${CMAKE_CURRENT_LIST_DIR}/sdio_cmd.pio)
pico_generate_pio_header(n91_hdd_emulator ${CMAKE_CURRENT_LIST_DIR}/sdio_dat.pio)
pico_generate_pio_header(n91_hdd_emulator ${CMAKE_CURRENT_LIST_DIR}/sdio_pulse.pio)

target_link_libraries(n91_hdd_emulator
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
)

# Вывод отладки через USB (или UART)
pico_enable_stdio_usb(n91_hdd_emulator 1)
pico_enable_stdio_uart(n91_hdd_emulator 0)

# Генерация итогового файла прошивки .uf2
pico_add_extra_outputs(n91_hdd_emulator)