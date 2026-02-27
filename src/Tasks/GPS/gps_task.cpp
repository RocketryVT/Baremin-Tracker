#include "gps_task.hpp"
#include "shared.hpp"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "gps/GPSParser.h"


#define GPS_UART      uart0
#define GPS_BAUD      9600

static void gps_task( void* param )
{
    ( void ) param;

    uart_init( GPS_UART, GPS_BAUD );
    gpio_set_function( Pins::GPS_UART_TX, GPIO_FUNC_UART );
    gpio_set_function( Pins::GPS_UART_RX, GPIO_FUNC_UART );
    uart_set_hw_flow( GPS_UART, false, false );
    uart_set_format( GPS_UART, 8, 1, UART_PARITY_NONE );
    uart_set_fifo_enabled( GPS_UART, true );

    log_print( "[gps] UART0 ready at %d baud (RX=GPIO%u TX=GPIO%u)\n",
            GPS_BAUD, Pins::GPS_UART_RX, Pins::GPS_UART_TX );

    gps::GPSParser parser;
    bool           had_fix = false;

    for ( ;; ) {
        // Drain UART FIFO — at 9600 baud, 10 ms ≈ 9.6 bytes; FIFO is 32 deep
        while ( uart_is_readable( GPS_UART ) ) {
            char c = ( char ) uart_getc( GPS_UART );
            parser.parse( c );
        }

        if ( parser.hasFix() ) {
            const gps::Coordinate& c = parser.getCoordinate();

            if ( !had_fix ) {
                log_print( "[gps] fix acquired: %.6f, %.6f  alt %.1f m  sats %d\n",
                        c.latitude, c.longitude, c.altitude, c.satellites );
                had_fix = true;
            }

            GpsData data {
                c.latitude,
                c.longitude,
                c.altitude,
                static_cast<uint8_t>( c.satellites )
            };

            // Overwrite so the LoRa task always sees the latest fix
            xQueueOverwrite( g_gps_queue, &data );

        } else if ( had_fix ) {
            log_print( "[gps] fix lost\n" );
            had_fix = false;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );   // 100 Hz poll
    }
}

static StaticTask_t s_gps_tcb;
static StackType_t  s_gps_stack[ 1024 ];

void gps_task_init()
{
    configASSERT( xTaskCreateStatic( gps_task, "gps", 1024,
                                      NULL, tskIDLE_PRIORITY + 3,
                                      s_gps_stack, &s_gps_tcb ) );
}
