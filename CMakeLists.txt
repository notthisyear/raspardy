cmake_minimum_required(VERSION 3.11)

# Ensure that SDK path is set
if (NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Environment variable PICO_SDK_PATH not set")
endif()

set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

# Set common properties 
set(PICO_PLATFORM "rp2040")
set(PICO_COMPILER "pico_arm_gcc")
set(PICO_BOARD "pico")
set(CMAKE_BUILD_TYPE "Debug")

# Pull in the PICO SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Ensure that SDK version is sufficient
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.5.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.5.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Specify the project
project(raspardy 
    LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(main
    	main.c
    )

# Get common dependencies
target_link_libraries(main pico_stdlib pico_multicore)

# Enable USB output, disable UART output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# Set the program name in the binary
pico_set_program_name(main "raspardy")

# Creates extra files, such as the uf2 file, which we need
pico_add_extra_outputs(main)