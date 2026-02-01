cmake_minimum_required(VERSION 3.12)

# Default to Pico 2 unless overridden by the caller.
set(PICO_BOARD "pico2")

# Shared dependency roots (LIBS_ROOT, PICO_SDK_PATH, FREERTOS_KERNEL_PATH, etc.)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

# All Third_Party deps live under projects/libs
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "Disable host -arch flag when cross-compiling for bare metal" FORCE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

if (WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" CACHE STRING "Prefer static libs for cross compile on Windows")
endif()

set(PICOTOOL_FETCH_FROM_GIT_PATH "${LIBS_ROOT}/Third_Party/picotool/fetch_from_git.cmake")

project(bareman_tracker C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.2.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.2.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Fail fast if an MSVC generator is selected: Visual Studio's generator uses MSVC
# and won't respect ARM GCC cross-toolchain settings.
if (CMAKE_GENERATOR MATCHES "Visual Studio")
    message(FATAL_ERROR "Visual Studio generator selected; this project targets ARM GCC. Use -G 'Ninja' and set CMAKE_TOOLCHAIN_FILE to the ARM toolchain.")
endif()

# FreeRTOSConfig.h location for the RP2350 FreeRTOS port
set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include)

# Initialize the Pico SDK now so its helper functions and toolchain flags take effect.
pico_sdk_init()

# Enable single-precision FPU instructions on Cortex-M33 (FreeRTOS port uses FP regs).
add_compile_options($<$<COMPILE_LANGUAGE:C,CXX,ASM>:-mfpu=fpv5-sp-d16>)

# FreeRTOS port selection
set(FREERTOS_THIRD_PARTY_DIR ${FREERTOS_KERNEL_PATH}/portable/ThirdParty)
if (PICO_PLATFORM MATCHES "rp2350")
    if (EXISTS "${FREERTOS_THIRD_PARTY_DIR}/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake")
        include(${FREERTOS_THIRD_PARTY_DIR}/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)
    else()
        message(FATAL_ERROR "FreeRTOS port for RP2350 not found under ${FREERTOS_THIRD_PARTY_DIR}. Ensure a port exists for your platform.")
    endif()
else()
    include(${FREERTOS_THIRD_PARTY_DIR}/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
endif()

# LoRa driver library
add_subdirectory(${AVIONICS_ROOT}/projects/libs/lora ${CMAKE_CURRENT_BINARY_DIR}/lora)

add_executable(bareman_tracker
    src/main.cpp
)

target_include_directories(bareman_tracker PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(bareman_tracker
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    lora
)

target_compile_definitions(bareman_tracker PRIVATE
    USE_FREERTOS=1
    PICO_STDIO_STACK_BUFFER_SIZE=64
)

pico_enable_stdio_usb(bareman_tracker 1)
pico_enable_stdio_uart(bareman_tracker 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(bareman_tracker)

add_compile_options(-Wall
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -fpermissive
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
