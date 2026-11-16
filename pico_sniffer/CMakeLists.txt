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
cmake_minimum_required(VERSION 3.13)

# === Блок для расширения VS Code (НЕ ТРОГАТЬ) ===
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
# =================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Настройка платы
set(PICO_BOARD pico CACHE STRING "Board type")

# Подключение SDK
include(pico_sdk_import.cmake)

# Имя проекта (должно совпадать с папкой, но лучше назвать reengineering)
project(pico_sniffer C CXX ASM)

pico_sdk_init()

# Исполняемый файл
add_executable(pico_sniffer
    main.cpp
)

pico_set_program_name(pico_sniffer "SDIO Sniffer")
pico_set_program_version(pico_sniffer "1.0")

# Генерация PIO заголовка (убедись, что файл sdio_cmd.pio лежит рядом)
pico_generate_pio_header(pico_sniffer ${CMAKE_CURRENT_LIST_DIR}/sdio_sniffer.pio)

# Настройки вывода
pico_enable_stdio_uart(pico_sniffer 0)
pico_enable_stdio_usb(pico_sniffer 1)

# Библиотеки
target_link_libraries(pico_sniffer
    pico_stdlib
    pico_multicore
    hardware_pio
)

# Генерация .uf2
pico_add_extra_outputs(pico_sniffer)