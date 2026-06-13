cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s)
set(PICO_BOARD bareman_pcb_v2)

# Bareman custom PCB has a 16 MB flash chip (vs 4 MB on stock Pico 2).
# Must be set before pico_sdk_init() so the SDK linker script sizes the flash
# region correctly and PICO_FLASH_SIZE_BYTES resolves to 16 MB everywhere.
set(PICO_FLASH_SIZE_BYTES 16777216)

# Locate the avionics root (2 levels up from this file)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

set(PICO_BOARD_HEADER_DIRS ${AVIONICS_ROOT}/boards)

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
set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)

pico_sdk_init()

# GPS NMEA parser library — provides the 'gps' target.
if (NOT EXISTS "${GPS_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "GPS library not found at ${GPS_ROOT}. Set GPS_ROOT.")
endif()
add_subdirectory(${GPS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/gps)

add_subdirectory(${LIBS_ROOT}/mesh ${CMAKE_CURRENT_BINARY_DIR}/mesh)

# pico_logger — flash ring-buffer logger.
if (NOT EXISTS "${PICO_LOGGER_PATH}/CMakeLists.txt")
    message(FATAL_ERROR "pico_logger not found at ${PICO_LOGGER_PATH}. "
            "Initialize the submodule or set PICO_LOGGER_PATH.")
endif()
add_subdirectory(${PICO_LOGGER_PATH} ${CMAKE_CURRENT_BINARY_DIR}/pico_logger)

# ICM-40609-D IMU driver
if (NOT EXISTS "${ICM40609D_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "icm40609d not found at ${ICM40609D_ROOT}. "
            "Check ICM40609D_ROOT in deps.cmake.")
endif()
add_subdirectory(${ICM40609D_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/icm40609d)

# MS5607 barometer driver.
if (NOT EXISTS "${MS5607_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "MS5607 driver not found at ${MS5607_ROOT}. Set MS5607_ROOT.")
endif()
add_subdirectory(${MS5607_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/ms5607)
# Compile ms5607 with FreeRTOS support; FreeRTOSConfig.h lives in this project's include/.
target_compile_definitions(ms5607 PRIVATE USE_FREERTOS=1)
target_include_directories(ms5607 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(ms5607 PUBLIC FreeRTOS-Kernel)

add_subdirectory(src)

add_compile_options(
    -Wall
    -Wno-format
    -Wno-unused-function
)
