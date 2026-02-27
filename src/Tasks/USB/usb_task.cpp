#include "usb_task.hpp"
#include "shared.hpp"

extern volatile uint32_t g_test_ticks;

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define STATUS_PERIOD_MS  10000   // print heap / queue stats every 10 s

// ── log_print ─────────────────────────────────────────────────────────────────
// Defined here so this translation unit is the sole caller of printf().
// All other tasks call log_print() to avoid stdio contention under FreeRTOS SMP.
// Non-blocking: drops the message silently if the queue is full.
void log_print( const char* fmt, ... )
{
    LogMessage msg;
    va_list args;
    va_start( args, fmt );
    vsnprintf( msg.buf, sizeof( msg.buf ), fmt, args );
    va_end( args );

    xQueueSend( g_log_queue, &msg, 0 );
}

// ── USB console task ──────────────────────────────────────────────────────────
static void usb_task( void* param )
{
    ( void ) param;

    LogMessage msg;
    TickType_t last_status = 0;
    uint32_t   msgs_received = 0;

    for ( ;; ) {
        // Block up to 100 ms for a log message, then drain any extras
        if ( xQueueReceive( g_log_queue, &msg, pdMS_TO_TICKS( 100 ) ) == pdTRUE ) {
            msgs_received++;
            printf( "%s", msg.buf );

            while ( xQueueReceive( g_log_queue, &msg, 0 ) == pdTRUE ) {
                msgs_received++;
                printf( "%s", msg.buf );
            }

            fflush( stdout );
        }

        // // Periodic status line
        // TickType_t now = xTaskGetTickCount();
        // if ( ( now - last_status ) >= pdMS_TO_TICKS( STATUS_PERIOD_MS ) ) {
        //     last_status = now;
        //     printf( "[status] heap=%u B  log_q=%u/%u  rx=%lu  test_ticks=%lu\n",
        //             ( unsigned ) xPortGetFreeHeapSize(),
        //             ( unsigned ) uxQueueMessagesWaiting( g_log_queue ),  ( unsigned ) LOG_QUEUE_DEPTH,
        //             ( unsigned long ) msgs_received,
        //             ( unsigned long ) g_test_ticks );
        // }
    }
}

static StaticTask_t s_usb_tcb;
static StackType_t  s_usb_stack[ 2048 ];

void usb_task_init()
{
    // Lowest priority — printing should never pre-empt real work.
    // Pinned to core 0: TinyUSB is not SMP-safe and the USB IRQ always fires
    // on core 0, so the task that calls tud_task() / printf must live there too.
    TaskHandle_t h = xTaskCreateStatic( usb_task, "usb", 2048,
                                         NULL, tskIDLE_PRIORITY + 1,
                                         s_usb_stack, &s_usb_tcb );
    configASSERT( h );
    vTaskCoreAffinitySet( h, 0x01 );
}
