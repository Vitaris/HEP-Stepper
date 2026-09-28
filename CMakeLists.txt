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
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(hep_stepper C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(hep_stepper
        hep_stepper.c
        tmc_interface.c
        stepper_controller.c
        servo_control.c
        core1_main.c
        )

pico_generate_pio_header(hep_stepper ${CMAKE_CURRENT_LIST_DIR}/step_generator.pio)
pico_generate_pio_header(hep_stepper ${CMAKE_CURRENT_LIST_DIR}/stepdir_counter.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(hep_stepper 1)
pico_enable_stdio_uart(hep_stepper 0)

# Pull in basic dependencies
target_link_libraries(hep_stepper pico_stdlib 
        hardware_spi
        hardware_pio
        pico_multicore)

# create map/bin/hex file etc.
pico_add_extra_outputs(hep_stepper)

# add url via pico_set_program_url
