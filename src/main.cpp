// Bareman Tracker — LR1121 LoRa GPS transmitter
// FreeRTOS / RP2350
//
// Task layout:
//   gps       (pri 3) – UART0 NMEA parse; overwrites g_gps_queue
//   fusion    (pri 2) – baro/GPS nav snapshot; overwrites g_fusion_queue
//   lora      (pri 4) – mesh/radio owner; transmits SIGMA2 frames
//   baro      (pri 1-3) – MS5607 sample + reader, overwrites g_baro_queue
//   imu       (pri 2) – optional ICM-40609-D 100 Hz, overwrites g_imu_queue
//   log_flash (pri 1) – drains g_logger_queue; commits SigmaStorageFullRecords to flash
//   usb       (pri 1) – drains g_log_queue; sole caller of printf()
//
// All FreeRTOS objects are statically allocated.

#include "shared.hpp"

#include "Tasks/GPS/gps_task.hpp"
#include "Tasks/LoRa/lora_task.hpp"
#include "Tasks/USB/usb_task.hpp"
#include "Tasks/Baro/baro_task.hpp"
#include "Tasks/Logger/logger_task.hpp"
#include "Tasks/IMU/imu_task.hpp"
#include "Tasks/I2C/i2c_task.hpp"
#include "Tasks/Fusion/fusion_task.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

// -- Shared FreeRTOS handles ---------------------------------------------------
QueueHandle_t    g_gps_queue    = nullptr;
QueueHandle_t    g_log_queue    = nullptr;
QueueHandle_t    g_baro_queue   = nullptr;
QueueHandle_t    g_fusion_queue = nullptr;
QueueHandle_t    g_logger_queue = nullptr;
QueueHandle_t    g_imu_queue    = nullptr;

volatile FlightState g_flight_state = FlightState::GROUND_IDLE;

static StaticQueue_t s_gps_queue_buf;
static uint8_t       s_gps_queue_storage[ GPS_QUEUE_DEPTH * sizeof( GpsData ) ];

static StaticQueue_t s_log_queue_buf;
static uint8_t       s_log_queue_storage[ LOG_QUEUE_DEPTH * sizeof( LogMessage ) ];

static StaticQueue_t s_baro_queue_buf;
static uint8_t       s_baro_queue_storage[ BARO_QUEUE_DEPTH * sizeof( BaroData ) ];

static StaticQueue_t s_fusion_queue_buf;
static uint8_t       s_fusion_queue_storage[ FUSION_QUEUE_DEPTH * sizeof( SIGMA2::NavSnapshot ) ];

static StaticQueue_t s_logger_queue_buf;
static uint8_t       s_logger_queue_storage[ LOGGER_QUEUE_DEPTH * sizeof( SigmaStorageFullRecord ) ];

static StaticQueue_t s_imu_queue_buf;
static uint8_t       s_imu_queue_storage[ IMU_QUEUE_DEPTH * sizeof( ImuData ) ];

// -- FreeRTOS static-allocation callbacks --------------------------------------
extern "C" {

void vApplicationGetIdleTaskMemory( StaticTask_t**  ppxIdleTaskTCBBuffer,
                                     StackType_t**   ppxIdleTaskStackBuffer,
                                     uint32_t*       pulIdleTaskStackSize )
{
    static StaticTask_t idle_tcb;
    static StackType_t  idle_stack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer   = &idle_tcb;
    *ppxIdleTaskStackBuffer =  idle_stack;
    *pulIdleTaskStackSize   =  configMINIMAL_STACK_SIZE;
}

// RP2350 has 2 cores — one passive idle task per additional core.
void vApplicationGetPassiveIdleTaskMemory( StaticTask_t**  ppxIdleTaskTCBBuffer,
                                            StackType_t**   ppxIdleTaskStackBuffer,
                                            uint32_t*       pulIdleTaskStackSize,
                                            BaseType_t      xPassiveIdleTaskIndex )
{
    static StaticTask_t passive_tcb  [ configNUMBER_OF_CORES - 1 ];
    static StackType_t  passive_stack[ configNUMBER_OF_CORES - 1 ]
                                     [ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer   = &passive_tcb  [ xPassiveIdleTaskIndex ];
    *ppxIdleTaskStackBuffer =  passive_stack [ xPassiveIdleTaskIndex ];
    *pulIdleTaskStackSize   =  configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t**  ppxTimerTaskTCBBuffer,
                                      StackType_t**   ppxTimerTaskStackBuffer,
                                      uint32_t*       pulTimerTaskStackSize )
{
    static StaticTask_t timer_tcb;
    static StackType_t  timer_stack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer   = &timer_tcb;
    *ppxTimerTaskStackBuffer =  timer_stack;
    *pulTimerTaskStackSize   =  configTIMER_TASK_STACK_DEPTH;
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char* pcTaskName )
{
    ( void ) xTask;
    ( void ) pcTaskName;
    // Halt on stack overflow — attach a debugger to see pcTaskName
    for ( ;; ) {}
}

void vApplicationMallocFailedHook( void )
{
    for ( ;; ) {}
}

} // extern "C"

// -- Heartbeat task ------------------------------------------------------------
static StaticTask_t s_hb_tcb;
static StackType_t  s_hb_stack[ 256 ];

static void heartbeat_task( void* )
{
    gpio_init( Pins::STATUS );
    gpio_set_dir( Pins::STATUS, GPIO_OUT );
    for ( ;; ) {
        gpio_put( Pins::STATUS, 1 );
        vTaskDelay( pdMS_TO_TICKS( 500 ) );
        gpio_put( Pins::STATUS, 0 );
        vTaskDelay( pdMS_TO_TICKS( 500 ) );
    }
}

// -- Entry point ---------------------------------------------------------------
int main( void )
{
    stdio_init_all();

    g_gps_queue = xQueueCreateStatic( GPS_QUEUE_DEPTH,
                                       sizeof( GpsData ),
                                       s_gps_queue_storage,
                                       &s_gps_queue_buf );

    g_log_queue = xQueueCreateStatic( LOG_QUEUE_DEPTH,
                                       sizeof( LogMessage ),
                                       s_log_queue_storage,
                                       &s_log_queue_buf );

    g_baro_queue = xQueueCreateStatic( BARO_QUEUE_DEPTH,
                                        sizeof( BaroData ),
                                        s_baro_queue_storage,
                                        &s_baro_queue_buf );

    g_fusion_queue = xQueueCreateStatic( FUSION_QUEUE_DEPTH,
                                          sizeof( SIGMA2::NavSnapshot ),
                                          s_fusion_queue_storage,
                                          &s_fusion_queue_buf );

    g_logger_queue = xQueueCreateStatic( LOGGER_QUEUE_DEPTH,
                                          sizeof( SigmaStorageFullRecord ),
                                          s_logger_queue_storage,
                                          &s_logger_queue_buf );

    g_imu_queue = xQueueCreateStatic( IMU_QUEUE_DEPTH,
                                       sizeof( ImuData ),
                                       s_imu_queue_storage,
                                       &s_imu_queue_buf );

    TaskHandle_t h = xTaskCreateStatic( heartbeat_task, "hb", 256,
                                            NULL, tskIDLE_PRIORITY + 1,
                                            s_hb_stack, &s_hb_tcb );
    configASSERT( h );

    usb_task_init();
    i2c_task_init();     // initialises i2c0 hardware + queue (must be first)
    baro_task_init();    // calls s_baro.initialize() pre-scheduler via direct fallback
#if HAS_IMU
    imu_task_init();
#else
    log_print( "[imu] disabled by board profile; running baro-only fusion\n" );
#endif
    fusion_task_init();
    logger_task_init();

    gps_task_init();
    lora_task_init();

    vTaskStartScheduler();

    for ( ;; ) {}
}
