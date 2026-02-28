#include "usb_task.hpp"
#include "shared.hpp"
#include "Tasks/Logger/logger_task.hpp"   // console API: read, erase, force record on/off

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>   // abs
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/time.h"

// ── log_print ─────────────────────────────────────────────────────────────────
// Sole caller of printf() in the whole project.  All other tasks call
// log_print() to keep stdio off the FreeRTOS SMP shared stack.
// Non-blocking: message silently dropped if the queue is full.
void log_print( const char* fmt, ... )
{
    LogMessage msg;
    va_list args;
    va_start( args, fmt );
    vsnprintf( msg.buf, sizeof( msg.buf ), fmt, args );
    va_end( args );
    xQueueSend( g_log_queue, &msg, 0 );
}

// ── Console output flags ──────────────────────────────────────────────────────
// All default false → absolutely silent at boot, matching the user requirement.
static bool s_log_enabled  = false;   // print task log messages from g_log_queue
static bool s_stream_baro  = false;   // print fresh baro data at ~1 Hz
static bool s_stream_gps   = false;   // print fresh GPS data at ~1 Hz
static bool s_stream_state = false;   // print flight state at ~1 Hz

volatile bool g_nmea_raw_enabled = false;   // stream raw NMEA sentences ($GNRMC…)
volatile bool g_ubx_hex_enabled  = false;   // hex-dump every raw UART byte ([x] lines)

// ── Helpers ───────────────────────────────────────────────────────────────────

static const char* flight_state_str( FlightState s )
{
    switch ( s ) {
        case FlightState::GROUND_IDLE:    return "GROUND_IDLE";
        case FlightState::ARMED:          return "ARMED";
        case FlightState::POWERED_ASCENT: return "POWERED_ASCENT";
        case FlightState::COAST_ASCENT:   return "COAST_ASCENT";
        case FlightState::APOGEE:         return "APOGEE";
        case FlightState::DESCENT_DROGUE: return "DESCENT_DROGUE";
        case FlightState::DESCENT_MAIN:   return "DESCENT_MAIN";
        case FlightState::LANDED:         return "LANDED";
        case FlightState::FAULT:          return "FAULT";
        default:                          return "UNKNOWN";
    }
}

// Parse an optional "on" / "off" argument after a command keyword.
// cmd_len = strlen of the keyword (e.g. 4 for "baro").
// With no argument the flag is toggled.
static void apply_on_off( const char* line, size_t cmd_len, bool& flag )
{
    const char* arg = line + cmd_len;
    while ( *arg == ' ' ) arg++;
    if ( strncmp( arg, "on",  2 ) == 0 &&
         ( arg[2] == '\0' || isspace( (unsigned char) arg[2] ) ) )
        flag = true;
    else if ( strncmp( arg, "off", 3 ) == 0 &&
              ( arg[3] == '\0' || isspace( (unsigned char) arg[3] ) ) )
        flag = false;
    else
        flag = !flag;   // bare command with no argument → toggle
}

// ── Commands ──────────────────────────────────────────────────────────────────

static void cmd_help()
{
    printf(
        "Commands:\n"
        "  help           show this list\n"
        "  status         print heap, queue depths, state, stream flags\n"
        "  baro  [on|off] toggle/set barometer streaming (~1 Hz)\n"
        "  gps   [on|off] toggle/set GPS streaming (~1 Hz)\n"
        "  nmea  [on|off] toggle/set raw NMEA sentence output ($GNRMC…)\n"
        "  hex   [on|off] toggle/set raw UART hex dump (16 bytes/line)\n"
        "  state [on|off] toggle/set flight state streaming (~1 Hz)\n"
        "  log   [on|off] toggle/set task log message output (default: off)\n"
        "  record on      force-start recording to flash now\n"
        "  record off     force-stop recording\n"
        "  data           decode and print all flash log records\n"
        "  csv            dump all flash records as CSV (spreadsheet-ready)\n"
        "  raw            hex-dump all raw flash log bytes\n"
        "  erase          erase flash log (DESTRUCTIVE — asks for 'yes')\n"
        "  clear          clear terminal screen\n"
    );
}

static void cmd_status()
{
    printf( "heap free    : %u B\n",
            ( unsigned ) xPortGetFreeHeapSize() );
    printf( "flight state : %s\n",
            flight_state_str( g_flight_state ) );
    printf( "log_q        : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_log_queue ),
            ( unsigned ) LOG_QUEUE_DEPTH );
    printf( "baro_q       : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_baro_queue ),
            ( unsigned ) BARO_QUEUE_DEPTH );
    printf( "logger_q     : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_logger_queue ),
            ( unsigned ) LOGGER_QUEUE_DEPTH );
    printf( "streaming    : baro=%-3s  gps=%-3s  nmea=%-3s  hex=%-3s  state=%-3s  log=%-3s\n",
            s_stream_baro        ? "on" : "off",
            s_stream_gps         ? "on" : "off",
            g_nmea_raw_enabled   ? "on" : "off",
            g_ubx_hex_enabled    ? "on" : "off",
            s_stream_state       ? "on" : "off",
            s_log_enabled        ? "on" : "off" );
}

// ── Command dispatch ──────────────────────────────────────────────────────────

static void dispatch( const char* line, size_t len )
{
    if ( len == 0 ) return;

    // ── Utility ───────────────────────────────────────────────────────────
    if ( strncmp( line, "help",   4 ) == 0 ) { cmd_help();   return; }
    if ( strncmp( line, "status", 6 ) == 0 ) { cmd_status(); return; }
    if ( strncmp( line, "clear",  5 ) == 0 ) {
        printf( "\x1b[2J\x1b[H" );
        stdio_flush();
        return;
    }

    // ── Streaming toggles ─────────────────────────────────────────────────
    if ( strncmp( line, "baro",  4 ) == 0 ) {
        apply_on_off( line, 4, s_stream_baro );
        printf( "baro streaming: %s\n", s_stream_baro  ? "on" : "off" );
        return;
    }
    if ( strncmp( line, "gps",   3 ) == 0 ) {
        apply_on_off( line, 3, s_stream_gps );
        printf( "gps streaming: %s\n",  s_stream_gps   ? "on" : "off" );
        return;
    }
    if ( strncmp( line, "nmea",  4 ) == 0 ) {
        bool f = g_nmea_raw_enabled;
        apply_on_off( line, 4, f );
        g_nmea_raw_enabled = f;
        printf( "nmea raw: %s\n", g_nmea_raw_enabled ? "on" : "off" );
        return;
    }
    if ( strncmp( line, "hex",   3 ) == 0 ) {
        bool f = g_ubx_hex_enabled;
        apply_on_off( line, 3, f );
        g_ubx_hex_enabled = f;
        printf( "hex dump: %s\n", g_ubx_hex_enabled ? "on" : "off" );
        return;
    }
    if ( strncmp( line, "state", 5 ) == 0 ) {
        apply_on_off( line, 5, s_stream_state );
        printf( "state streaming: %s\n", s_stream_state ? "on" : "off" );
        return;
    }
    if ( strncmp( line, "log",   3 ) == 0 ) {
        apply_on_off( line, 3, s_log_enabled );
        printf( "log output: %s\n", s_log_enabled ? "on" : "off" );
        return;
    }

    // ── Recording control ─────────────────────────────────────────────────
    if ( strncmp( line, "record", 6 ) == 0 ) {
        const char* arg = line + 6;
        while ( *arg == ' ' ) arg++;
        if ( strncmp( arg, "on", 2 ) == 0 ) {
            logger_task_force_record_on();
            printf( "recording: force on\n" );
        } else if ( strncmp( arg, "off", 3 ) == 0 ) {
            logger_task_force_record_off();
            printf( "recording: force off\n" );
        } else {
            printf( "usage: record on  |  record off\n" );
        }
        return;
    }

    // ── Flash inspection ──────────────────────────────────────────────────
    // The logger task is suspended for the duration to prevent concurrent
    // flash writes while we are reading or erasing.
    if ( strncmp( line, "data", 4 ) == 0 ) {
        TaskHandle_t h = logger_task_get_handle();
        if ( h ) vTaskSuspend( h );
        logger_task_read_decoded();
        if ( h ) vTaskResume( h );
        return;
    }

    if ( strncmp( line, "csv", 3 ) == 0 ) {
        TaskHandle_t h = logger_task_get_handle();
        if ( h ) vTaskSuspend( h );
        logger_task_read_csv();
        if ( h ) vTaskResume( h );
        return;
    }

    if ( strncmp( line, "raw", 3 ) == 0 ) {
        TaskHandle_t h = logger_task_get_handle();
        if ( h ) vTaskSuspend( h );
        logger_task_read_raw();
        if ( h ) vTaskResume( h );
        return;
    }

    if ( strncmp( line, "erase", 5 ) == 0 ) {
        printf( "Type 'yes' to confirm erase: " );
        stdio_flush();

        // Read confirmation with a 10-second timeout, echoing each character.
        char   confirm[ 8 ] = { 0 };
        size_t ci            = 0;
        TickType_t deadline  = xTaskGetTickCount() + pdMS_TO_TICKS( 10000 );

        while ( ci < sizeof( confirm ) - 1 && xTaskGetTickCount() < deadline ) {
            int c = getchar_timeout_us( 0 );
            if ( c == PICO_ERROR_TIMEOUT || c >= 256 ) {
                vTaskDelay( pdMS_TO_TICKS( 10 ) );
                continue;
            }
            if ( c == '\r' || c == '\n' ) { printf( "\r\n" ); break; }
            stdio_putchar( c );
            confirm[ ci++ ] = ( char ) c;
        }
        confirm[ ci ] = '\0';

        if ( strcmp( confirm, "yes" ) == 0 ) {
            TaskHandle_t h = logger_task_get_handle();
            if ( h ) vTaskSuspend( h );
            printf( "Erasing flash..." );
            stdio_flush();
            logger_task_erase();
            if ( h ) vTaskResume( h );
            printf( " done.\n" );
        } else {
            printf( "Cancelled.\n" );
        }
        return;
    }

    printf( "unknown command: '%s'  (type 'help')\n", line );
}

// ── USB / serial console task ─────────────────────────────────────────────────
// Pinned to core 0: TinyUSB IRQ fires on core 0; printf() must live there too.
//
// Loop:
//   1. Drain g_log_queue — print only when s_log_enabled.
//   2. Sensor streaming — one line per enabled sensor every ~1 s.
//   3. Non-blocking char read — echo, backspace, dispatch on CR/LF.
//      Prints "# " as a prompt after every dispatched line.
static void usb_task( void* )
{
    char       line[ 128 ] = { 0 };
    size_t     line_len    = 0;
    TickType_t last_stream = 0;

    for ( ;; ) {
        // ── 1. Drain log queue ────────────────────────────────────────────
        {
            LogMessage msg;
            if ( xQueueReceive( g_log_queue, &msg, pdMS_TO_TICKS( 10 ) ) == pdTRUE ) {
                auto should_print = [&]( const LogMessage& m ) -> bool {
                    if ( s_log_enabled ) return true;
                    if ( g_nmea_raw_enabled && m.buf[0] == '$' ) return true;
                    if ( g_ubx_hex_enabled  && m.buf[0] == '[' &&
                         m.buf[1] == 'x'   && m.buf[2] == ']' ) return true;
                    return false;
                };
                bool flushed = false;
                if ( should_print( msg ) ) { printf( "%s", msg.buf ); flushed = true; }
                while ( xQueueReceive( g_log_queue, &msg, 0 ) == pdTRUE )
                    if ( should_print( msg ) ) { printf( "%s", msg.buf ); flushed = true; }
                if ( flushed ) stdio_flush();
            }
        }

        // ── 2. Sensor streaming ───────────────────────────────────────────
        {
            TickType_t now = xTaskGetTickCount();
            if ( ( now - last_stream ) >= pdMS_TO_TICKS( 1000 ) ) {
                last_stream = now;

                if ( s_stream_baro ) {
                    BaroData b;
                    if ( xQueuePeek( g_baro_queue, &b, 0 ) == pdTRUE )
                        printf( "[baro] P=%ldPa  T=%ld.%02ld°C  Alt=%ld.%01ldm\n",
                                ( long ) b.pressure_pa,
                                ( long ) ( b.temperature_cdeg / 100 ),
                                ( long ) abs( b.temperature_cdeg % 100 ),
                                ( long ) ( b.altitude_dm / 10 ),
                                ( long ) abs( b.altitude_dm % 10 ) );
                    else
                        printf( "[baro] no data yet\n" );
                }

                if ( s_stream_gps ) {
                    GpsData g;
                    if ( xQueuePeek( g_gps_queue, &g, 0 ) == pdTRUE ) {
                        printf( "[gps]  %04u-%02u-%02u %02u:%02u:%02u UTC"
                                "  fix=%u  sats=%u/%u"
                                "  hDOP=%.2f  vDOP=%.2f  cno=%udBHz\n"
                                "       lat=%11.7f  lon=%12.7f  alt=%7.1fm\n"
                                "       spd=%6.2fm/s  crs=%6.1f\xc2\xb0"
                                "  vN=%6ld  vE=%6ld  vD=%6ld mm/s\n",
                                ( unsigned ) g.utc_year,
                                ( unsigned ) g.utc_month,
                                ( unsigned ) g.utc_day,
                                ( unsigned )( g.utc_ms / 3600000u ),
                                ( unsigned )( g.utc_ms % 3600000u / 60000u ),
                                ( unsigned )( g.utc_ms % 60000u   / 1000u ),
                                ( unsigned ) g.fix_type,
                                ( unsigned ) g.num_sv_used,
                                ( unsigned ) g.satellites,
                                ( double ) g.hdop, ( double ) g.vdop,
                                ( unsigned ) g.best_cno,
                                g.lat, g.lon, ( double ) g.alt_m,
                                ( double ) g.speed_mps, ( double ) g.course_deg,
                                ( long ) g.vel_north_mms,
                                ( long ) g.vel_east_mms,
                                ( long ) g.vel_down_mms );
                    } else {
                        printf( "[gps]  no fix\n" );
                    }
                }

                if ( s_stream_state )
                    printf( "[state] %s\n",
                            flight_state_str( g_flight_state ) );

                if ( s_stream_baro || s_stream_gps || s_stream_state )
                    stdio_flush();
            }
        }

        // ── 3. Read characters from USB CDC ───────────────────────────────
        {
            int c;
            while ( ( c = getchar_timeout_us( 0 ) ) != PICO_ERROR_TIMEOUT && c < 256 ) {
                if ( c == '\r' || c == '\n' ) {
                    printf( "\r\n" );
                    line[ line_len ] = '\0';
                    dispatch( line, line_len );
                    line_len  = 0;
                    line[ 0 ] = '\0';
                    printf( "# " );
                    stdio_flush();

                } else if ( ( c == '\b' || c == 127 ) && line_len > 0 ) {
                    // Backspace: erase one character on the terminal.
                    line[ --line_len ] = '\0';
                    printf( "\b \b" );
                    stdio_flush();

                } else if ( c >= 0x20 && line_len < sizeof( line ) - 1 ) {
                    line[ line_len++ ] = ( char ) c;
                    stdio_putchar( c );   // echo
                    stdio_flush();
                }
            }
        }
    }
}

static StaticTask_t s_usb_tcb;
static StackType_t  s_usb_stack[ 2048 ];

void usb_task_init()
{
    TaskHandle_t h = xTaskCreateStatic( usb_task, "usb", 2048,
                                         nullptr, tskIDLE_PRIORITY + 1,
                                         s_usb_stack, &s_usb_tcb );
    configASSERT( h );
    // TinyUSB IRQ fires on core 0; printf must live on the same core.
    vTaskCoreAffinitySet( h, 0x01 );
}
