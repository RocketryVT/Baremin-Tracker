// Bareman Tracker — LR1121 LoRa GPS transmitter
// FreeRTOS / RP2350
//
// Task layout:
//   gps  (pri 3) – UART0 NMEA parse; overwrites g_gps_queue
//   lora (pri 2) – reads g_gps_queue; transmits JSON via LR1121 at 915 MHz
//   usb  (pri 1) – drains g_log_queue; sole caller of printf()
//
// All FreeRTOS objects are statically allocated.

#include "shared.hpp"

#include "Tasks/GPS/gps_task.hpp"
#include "Tasks/LoRa/lora_task.hpp"
#include "Tasks/USB/usb_task.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

// ── Shared FreeRTOS handles ───────────────────────────────────────────────────
QueueHandle_t g_gps_queue = nullptr;
QueueHandle_t g_log_queue = nullptr;

static StaticQueue_t s_gps_queue_buf;
static uint8_t       s_gps_queue_storage[ GPS_QUEUE_DEPTH * sizeof( GpsData ) ];

static StaticQueue_t s_log_queue_buf;
static uint8_t       s_log_queue_storage[ LOG_QUEUE_DEPTH * sizeof( LogMessage ) ];

// ── FreeRTOS static-allocation callbacks ──────────────────────────────────────
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

// ── Heartbeat task ────────────────────────────────────────────────────────────
static StaticTask_t s_hb_tcb;
static StackType_t  s_hb_stack[ 256 ];

static void heartbeat_task( void* )
{
    gpio_init( Pins::STATUS );
    gpio_set_dir( Pins::STATUS, GPIO_OUT );
    for ( ;; ) {
        gpio_put( Pins::STATUS, 1 );
        // printf( "[heartbeat] led on\n" );
        vTaskDelay( pdMS_TO_TICKS( 500 ) );
        gpio_put( Pins::STATUS, 0 );
        // printf( "[heartbeat] led off\n" );
        vTaskDelay( pdMS_TO_TICKS( 500 ) );
    }
}

// ── Test task ─────────────────────────────────────────────────────────────────
static StaticTask_t s_test_tcb;
static StackType_t  s_test_stack[ 1024 ];

volatile uint32_t g_test_ticks = 0;   // incremented by test_task; no queue needed

static void test_task( void* )
{
    for ( ;; ) {
        g_test_ticks++;
        struct LogMessage msg = { .buf = "Hello from test_task!\n" };
        xQueueSend( g_log_queue, &msg, portMAX_DELAY );
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────
int main( void )
{
    stdio_init_all();

    // sleep_ms(2500);

    // printf( "=== Bareman Tracker — LR1121 GPS LoRa TX ===\n" );
    // printf( "    Board  : RP2350 custom PCB\n" );
    // printf( "    LoRa   : LR1121  SPI0  NSS=%u BUSY=%u RST=%u\n",
    //         Pins::LR_NSS, Pins::LR_BUSY, Pins::LR_NRESET );
    // printf( "    GPS    : UART0   GPIO RX=%u TX=%u\n",
    //         Pins::GPS_UART_RX, Pins::GPS_UART_TX );
    // printf( "    Freq   : %lu Hz  SF%u  BW%u  sync=0x%02X\n\n",
    //         ( unsigned long ) LoRaCfg::FREQ_HZ,
    //         LoRaCfg::SF, LoRaCfg::BW, LoRaCfg::SYNC_WORD );

    // // flush stdout before FreeRTOS takes over
    // fflush( stdout );

    g_gps_queue = xQueueCreateStatic( GPS_QUEUE_DEPTH,
                                       sizeof( GpsData ),
                                       s_gps_queue_storage,
                                       &s_gps_queue_buf );

    g_log_queue = xQueueCreateStatic( LOG_QUEUE_DEPTH,
                                       sizeof( LogMessage ),
                                       s_log_queue_storage,
                                       &s_log_queue_buf );

    usb_task_init();

    TaskHandle_t h = xTaskCreateStatic( heartbeat_task, "hb", 256,
                                            NULL, tskIDLE_PRIORITY + 1,
                                            s_hb_stack, &s_hb_tcb );
    configASSERT( h );
    vTaskCoreAffinitySet( h, ( 1u << 0 ) );

    TaskHandle_t test = xTaskCreateStatic( test_task, "test", 1024,
                                      NULL, tskIDLE_PRIORITY + 2,
                                      s_test_stack, &s_test_tcb );
    configASSERT( test );
    // vTaskCoreAffinitySet( test, 0x01);

    // gps_task_init();
    // lora_task_init();

    vTaskStartScheduler();

    for ( ;; ) {}
}
