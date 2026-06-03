# -- RadioLib (local submodule, set by deps.cmake) -----------------------------
if (NOT EXISTS "${RADIOLIB_PATH}/CMakeLists.txt")
    message(FATAL_ERROR "RadioLib not found at ${RADIOLIB_PATH}. "
            "Check RADIOLIB_PATH in deps.cmake or set -DRADIOLIB_PATH=...")
endif()
add_subdirectory(${RADIOLIB_PATH} ${CMAKE_CURRENT_BINARY_DIR}/radiolib)

add_executable(bareman_tracker
    main.cpp
    new_delete.cpp
    Tasks/GPS/gps_task.cpp
    Tasks/LoRa/lora_task.cpp
    Tasks/USB/usb_task.cpp
    Tasks/Baro/baro_task.cpp
    Tasks/IMU/imu_task.cpp
    Tasks/I2C/i2c_task.cpp
    Tasks/Logger/logger_task.cpp
)

target_include_directories(bareman_tracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include   # FreeRTOSConfig.h, PicoHal.h
    ${CMAKE_CURRENT_SOURCE_DIR}              # shared.hpp, task headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Tasks/IMU  # imu_task.hpp
    ${LIBS_ROOT}/packets                     # SIGMA.hpp
    ${RADIOLIB_PATH}/src                     # RadioLib.h
)

target_compile_definitions(bareman_tracker PRIVATE
    USE_FREERTOS=1
    PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1  # we supply our own in new_delete.cpp
)

target_link_libraries(bareman_tracker
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_spi
    hardware_i2c
    hardware_dma
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    RadioLib                                 # SX1276 driver
    gps                                      # NMEA parser (gps::GPSParser)
    pico_logger                              # flash ring-buffer logger
    icm40609d                                # TDK ICM-40609-D IMU driver
    ms5607                                   # TE MS5607 barometer driver
)

pico_enable_stdio_usb(bareman_tracker 1)
pico_enable_stdio_uart(bareman_tracker 0)

pico_set_program_name(bareman_tracker "bareman_tracker")
pico_set_program_version(bareman_tracker "0.1")

pico_add_extra_outputs(bareman_tracker)
