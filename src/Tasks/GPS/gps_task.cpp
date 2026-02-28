#include "gps_task.hpp"
#include "shared.hpp"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "gps/GPSParser.h"


#define GPS_UART      uart0
#define GPS_BAUD      38400

static void gps_task( void* param )
{
    ( void ) param;

    gpio_set_function(Pins::GPS_UART_TX, UART_FUNCSEL_NUM(GPS_UART, Pins::GPS_UART_TX));
    gpio_set_function(Pins::GPS_UART_RX, UART_FUNCSEL_NUM(GPS_UART, Pins::GPS_UART_RX));
    uart_init( GPS_UART, GPS_BAUD );

    uart_set_hw_flow( GPS_UART, false, false );
    uart_set_format( GPS_UART, 8, 1, UART_PARITY_NONE );
    uart_set_fifo_enabled( GPS_UART, true );

    log_print( "[gps] UART0 ready at %d baud (RX=GPIO%u TX=GPIO%u)\n",
            GPS_BAUD, Pins::GPS_UART_RX, Pins::GPS_UART_TX );

    gps::GPSParser parser;
    bool           had_fix = false;

    char nmea_buf[ 128 ];
    int  nmea_len = 0;

    for ( ;; ) {
        while ( uart_is_readable( GPS_UART ) ) {
            char c = ( char ) uart_getc( GPS_UART );
            parser.parse( c );

            // Echo raw NMEA to USB console (when "nmea on")
            if ( c == 0 ) {
                continue;
            } else if ( (int)c == 10 ) {
                nmea_buf[ nmea_len ] = '\0';
                if ( nmea_len > 0 && g_nmea_raw_enabled )
                    log_print( "%s\n", nmea_buf );
                nmea_len = 0;
            } else if ( c != '\r' && nmea_len < ( int ) sizeof( nmea_buf ) - 1 ) {
                nmea_buf[ nmea_len++ ] = c;
            } else if ( nmea_len >= ( int ) sizeof( nmea_buf ) - 1 ) {
                nmea_len = 0;
            }
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

            xQueueOverwrite( g_gps_queue, &data );

        } else if ( had_fix ) {
            log_print( "[gps] fix lost\n" );
            had_fix = false;
        }

        vTaskDelay( pdMS_TO_TICKS( 1 ) );
    }
}

static StaticTask_t s_gps_tcb;
static StackType_t  s_gps_stack[ 1024 ];

void gps_task_init()
{
    TaskHandle_t h = xTaskCreateStatic( gps_task, "gps", 1024,
                                      NULL, tskIDLE_PRIORITY + 3,
                                      s_gps_stack, &s_gps_tcb );
    configASSERT( h );
    vTaskCoreAffinitySet( h, ( 1u << 0 ) );
}
