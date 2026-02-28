add_executable(bareman_tracker
    main.cpp
    Tasks/GPS/gps_task.cpp
    Tasks/LoRa/lora_task.cpp
    Tasks/USB/usb_task.cpp
    Tasks/Baro/ms5607.cpp
    Tasks/Baro/baro_task.cpp
    Tasks/Logger/logger_task.cpp
)

target_include_directories(bareman_tracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include   # FreeRTOSConfig.h
    ${CMAKE_CURRENT_SOURCE_DIR}              # shared.hpp, task headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Tasks/Baro  # ms5607.hpp, altitude-pa.h
    ${LIBS_ROOT}/packets                     # SIGMA.hpp
)

target_compile_definitions(bareman_tracker PRIVATE
    USE_FREERTOS=1
)

target_link_libraries(bareman_tracker
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_spi
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    lr11xx                                   # Semtech LR11XX driver + Pico SPI HAL
    gps                                      # NMEA parser (gps::GPSParser)
    pico_logger                              # flash ring-buffer logger
)

# lr11xx_hal.c uses FreeRTOS — give it our FreeRTOSConfig.h so it compiles.
# (FreeRTOS-Kernel itself does not bundle FreeRTOSConfig.h; it is project-specific.)
target_include_directories(lr11xx PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

pico_enable_stdio_usb(bareman_tracker 1)
pico_enable_stdio_uart(bareman_tracker 0)

pico_set_program_name(bareman_tracker "bareman_tracker")
pico_set_program_version(bareman_tracker "0.1")

pico_add_extra_outputs(bareman_tracker)
