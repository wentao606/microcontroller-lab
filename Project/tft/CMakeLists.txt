# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(tft C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(tft)

# pico_set_program_name(tft "tft")
# pico_set_program_version(tft "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_usb(tft 1)
pico_enable_stdio_uart(tft 1)


pico_generate_pio_header(tft ${CMAKE_CURRENT_LIST_DIR}/SPIPIO.pio)

target_sources(tft PRIVATE tft.c TFTMaster.c glcdfont.c)
# Add the standard library to the build
target_link_libraries(tft PRIVATE
        pico_stdlib hardware_gpio pico_time hardware_pio hardware_dma hardware_spi
        hardware_pwm hardware_irq pico_bootsel_via_double_reset pico_multicore
        )


pico_add_extra_outputs(tft)

