cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s)
set(PICO_BOARD pico2)

# Locate the avionics root (2 levels up from this file)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

# Pull in Pico SDK (must be before project())
set(PICO_SDK_ROOT ${PICO_SDK_PATH})
if (NOT EXISTS "${PICO_SDK_ROOT}/pico_sdk_init.cmake")
    message(FATAL_ERROR "Pico SDK not found at ${PICO_SDK_ROOT}. "
            "Initialize the submodule or set PICO_SDK_PATH.")
endif()
include(${PICO_SDK_ROOT}/pico_sdk_init.cmake)

# Pull in FreeRTOS RP2350 port
set(FREERTOS_KERNEL_ROOT ${FREERTOS_KERNEL_PATH})
if (NOT EXISTS "${FREERTOS_KERNEL_ROOT}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake")
    message(FATAL_ERROR "FreeRTOS Kernel not found at ${FREERTOS_KERNEL_ROOT}. "
            "Initialize the submodule or set FREERTOS_KERNEL_PATH.")
endif()
include(${FREERTOS_KERNEL_ROOT}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "Disable host -arch when cross-compiling" FORCE)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(bareman_tracker C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# LoRa library — provides the lr11xx target (Semtech driver + Pico SPI HAL).
if (NOT EXISTS "${LORA_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "LoRa drivers not found at ${LORA_ROOT}. Set LORA_ROOT.")
endif()
add_subdirectory(${LORA_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/lora)

# GPS NMEA parser library — provides the 'gps' target.
if (NOT EXISTS "${GPS_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "GPS library not found at ${GPS_ROOT}. Set GPS_ROOT.")
endif()
add_subdirectory(${GPS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/gps)

# pico_logger — flash ring-buffer logger.
if (NOT EXISTS "${PICO_LOGGER_PATH}/CMakeLists.txt")
    message(FATAL_ERROR "pico_logger not found at ${PICO_LOGGER_PATH}. "
            "Initialize the submodule or set PICO_LOGGER_PATH.")
endif()
add_subdirectory(${PICO_LOGGER_PATH} ${CMAKE_CURRENT_BINARY_DIR}/pico_logger)

add_subdirectory(src)

add_compile_options(
    -Wall
    -Wno-format
    -Wno-unused-function
)
