#include "usb_task.hpp"
#include "shared.hpp"
#include "Tasks/Logger/logger_task.hpp"   // console API: read, erase, force record on/off
// #include "Tasks/Baro/baro_task.hpp"       // disabled with i2c scan
// #include "Tasks/I2C/i2c_task.hpp"         // disabled with i2c scan

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>   // abs, atoi
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// -- log_print -----------------------------------------------------------------
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

// -- Console output flags ------------------------------------------------------
// All default false -> absolutely silent at boot, matching the user requirement.
static bool s_log_enabled  = false;   // print task log messages from g_log_queue
static bool s_stream_baro  = false;   // print fresh baro data
static bool s_stream_gps   = false;   // print fresh GPS data
static bool s_stream_imu   = false;   // print fresh IMU data
static bool s_stream_state = false;   // print flight state at ~1 Hz

// Per-sensor streaming intervals (ms).  Clamped to [100, 60000].
static uint32_t s_baro_interval_ms  = 1000;
static uint32_t s_gps_interval_ms   = 1000;
static uint32_t s_imu_interval_ms   = 1000;

volatile bool g_nmea_raw_enabled = false;   // stream raw NMEA sentences ($GNRMC…)
volatile bool g_ubx_hex_enabled  = false;   // hex-dump every raw UART byte ([x] lines)

// -- Helpers -------------------------------------------------------------------

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

// Simple on/off toggle — no Hz parsing.
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
        flag = !flag;
}

// Parse an optional "on" / "off" / Hz argument after a command keyword.
// cmd_len = strlen of the keyword (e.g. 4 for "baro").
// Behaviour:
//   "baro on"    -> flag = true,  interval unchanged
//   "baro off"   -> flag = false, interval unchanged
//   "baro 5"     -> flag = true,  interval = 1000/5 = 200 ms  (capped 100..60000)
//   "baro 0.5"   -> flag = true,  interval = 2000 ms
//   "baro"       -> toggle flag,  interval unchanged
static void apply_on_off_hz( const char* line, size_t cmd_len,
                              bool& flag, uint32_t& interval_ms )
{
    const char* arg = line + cmd_len;
    while ( *arg == ' ' ) arg++;

    if ( strncmp( arg, "on",  2 ) == 0 &&
         ( arg[2] == '\0' || isspace( (unsigned char) arg[2] ) ) ) {
        flag = true;
    } else if ( strncmp( arg, "off", 3 ) == 0 &&
                ( arg[3] == '\0' || isspace( (unsigned char) arg[3] ) ) ) {
        flag = false;
    } else if ( *arg >= '0' && *arg <= '9' ) {
        // Numeric: treat as Hz (float ok, e.g. "0.5")
        double hz = atof( arg );
        if ( hz > 0.0 ) {
            uint32_t ms = ( uint32_t )( 1000.0 / hz );
            if ( ms < 100  ) ms = 100;    // cap at 10 Hz
            if ( ms > 60000 ) ms = 60000; // floor at ~0.017 Hz
            interval_ms = ms;
        }
        flag = true;
    } else {
        flag = !flag;   // bare command with no argument -> toggle
    }
}

// -- I2C scan — uses official Pico SDK pattern --------------------------------
// Identical to the pico-examples bus_scan: i2c_init + i2c_read_blocking per
// address.  Works because we call i2c_deinit between pin pairs *after*
// suspending any tasks that own that bus, so the peripheral is idle.
//
// Addresses of the form 000 0xxx or 111 1xxx are I2C-reserved; skip them.
static bool i2c_reserved_addr( uint8_t addr )
{
    return ( addr & 0x78u ) == 0u || ( addr & 0x78u ) == 0x78u;
}

// -- Commands ------------------------------------------------------------------

static void cmd_help()
{
    printf(
        "Commands:\n"
        "  help           show this list\n"
        "  status         print heap, queue depths, state, stream flags\n"
        "  top            task CPU%%, stack usage, and heap stats (like top)\n"
        "  top live       refresh top every 2 s until any key pressed\n"
        "  baro  [on|off|Hz] toggle/set barometer streaming (e.g. 'baro 5' = 5 Hz)\n"
        "  gps   [on|off|Hz] toggle/set GPS streaming      (e.g. 'gps 0.5' = 0.5 Hz)\n"
        "  imu   [on|off|Hz] toggle/set IMU streaming      (e.g. 'imu 10' = 10 Hz)\n"
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

// -- top -----------------------------------------------------------------------
// Renders one snapshot of per-task stats + heap, using ANSI escape codes so
// repeated calls overwrite the same screen region (like 'top').
//
// Layout (one line per task, sorted by CPU ticks descending):
//
//   TASK            STATE  PRI  CORE  CPU%   STACK_FREE/TOTAL  STACK%
//   gps             R       3    0    42.1%   512 /1024 B  50%
//   ...
//   ── Heap ──  free=68432  min_free=65104  total=73728  used=5296 (7%)

// State letter matching eTaskState enum.
static char task_state_char( eTaskState s )
{
    switch ( s ) {
        case eRunning:   return 'R';
        case eReady:     return 'r';
        case eBlocked:   return 'B';
        case eSuspended: return 'S';
        case eDeleted:   return 'D';
        default:         return '?';
    }
}

// Print one top snapshot.  Caller is responsible for cursor positioning.
// tasks[] is static so the ~960-byte array doesn't eat the USB task stack.
static void cmd_top()
{
    static constexpr UBaseType_t MAX_TASKS = 24;

    static TaskStatus_t tasks[ MAX_TASKS ];
    uint32_t     total_run_time = 0;

    UBaseType_t n = uxTaskGetSystemState( tasks, MAX_TASKS, &total_run_time );

    // Sort by runtime descending (insertion sort — n is small).
    for ( UBaseType_t i = 1; i < n; ++i ) {
        TaskStatus_t tmp = tasks[i];
        UBaseType_t  j   = i;
        while ( j > 0 && tasks[j-1].ulRunTimeCounter < tmp.ulRunTimeCounter ) {
            tasks[j] = tasks[j-1];
            --j;
        }
        tasks[j] = tmp;
    }

    // Header line.
    printf( "\x1b[1m%-16s %c %3s %4s  %6s   %22s  %5s\x1b[0m\n",
            "TASK", 'S', "PRI", "CORE", "CPU%%", "STACK used/total", "USE%%" );

    for ( UBaseType_t i = 0; i < n; ++i ) {
        const TaskStatus_t& t = tasks[i];

        // CPU% — runtime counter is in ms (time_us_64/1000).
        float cpu_pct = ( total_run_time > 0 )
            ? ( float ) t.ulRunTimeCounter * 100.0f / ( float ) total_run_time
            : 0.0f;

        // Stack total: pxEndOfStack is highest address (configRECORD_STACK_HIGH_ADDRESS=1).
        // ARM stack grows downward: total = (end - base + 1) words × 4 bytes/word.
        uint32_t total_bytes = 0;
        uint32_t free_bytes  = t.usStackHighWaterMark;  // min free bytes ever seen
        uint32_t used_bytes  = 0;
        int      use_pct     = 0;
#if configRECORD_STACK_HIGH_ADDRESS
        if ( t.pxEndOfStack && t.pxStackBase ) {
            total_bytes = static_cast<uint32_t>(
                ( t.pxEndOfStack - t.pxStackBase + 1 ) * sizeof( StackType_t ) );
            used_bytes  = ( free_bytes < total_bytes ) ? total_bytes - free_bytes : 0;
            use_pct     = ( total_bytes > 0 )
                          ? static_cast<int>( used_bytes * 100u / total_bytes ) : 0;
        }
#endif

        // Core affinity bitmask → display string.
#if configUSE_CORE_AFFINITY
        UBaseType_t aff = vTaskCoreAffinityGet( t.xHandle );
        char core_str[3];
        if      ( aff == 0x1u ) { core_str[0] = ' '; core_str[1] = '0'; core_str[2] = '\0'; }
        else if ( aff == 0x2u ) { core_str[0] = ' '; core_str[1] = '1'; core_str[2] = '\0'; }
        else                    { core_str[0] = ' '; core_str[1] = '*'; core_str[2] = '\0'; }
#else
        const char* core_str = " -";
#endif

        // Colour: red if stack use > 80%, yellow > 60%, normal otherwise.
        const char* color = ( use_pct >= 80 ) ? "\x1b[31m"
                          : ( use_pct >= 60 ) ? "\x1b[33m"
                          : "";
        const char* reset = ( use_pct >= 60 ) ? "\x1b[0m" : "";

        printf( "%-16s %c  %3u   %s  %5.1f%%   %s%5lu / %5lu B  %3d%%%s\n",
                t.pcTaskName,
                task_state_char( t.eCurrentState ),
                ( unsigned ) t.uxCurrentPriority,
                core_str,
                ( double ) cpu_pct,
                color,
                ( unsigned long ) used_bytes,
                ( unsigned long ) total_bytes,
                use_pct,
                reset );
    }

    // Separator.
    printf( "────────────────────────────────────────────────────────────\n" );

    // Heap stats — use xPortGetFreeHeapSize / xPortGetMinimumEverFreeHeapSize
    // which are always available with Heap4, unlike vPortGetHeapStats which
    // requires a newer kernel version.
    const size_t heap_free     = xPortGetFreeHeapSize();
    const size_t heap_min_free = xPortGetMinimumEverFreeHeapSize();
    const size_t heap_total    = configTOTAL_HEAP_SIZE;
    const size_t heap_used     = ( heap_free < heap_total ) ? heap_total - heap_free : 0;
    const int    heap_pct      = ( int )( heap_used * 100u / heap_total );

    const char* hcolor = ( heap_pct >= 80 ) ? "\x1b[31m"
                       : ( heap_pct >= 60 ) ? "\x1b[33m"
                       : "";
    const char* hreset = ( heap_pct >= 60 ) ? "\x1b[0m" : "";

    printf( "\x1b[1mHeap\x1b[0m"
            "  used=%s%lu\x1b[0m / %lu B  (%s%d%%%s)"
            "  free=%lu  min_free=%lu\n",
            hcolor, ( unsigned long ) heap_used, ( unsigned long ) heap_total,
            hcolor, heap_pct, hreset,
            ( unsigned long ) heap_free,
            ( unsigned long ) heap_min_free );

    printf( "\x1b[2mruntime counter: %lu ms  tasks: %u\x1b[0m\n",
            ( unsigned long ) total_run_time, ( unsigned ) n );

    stdio_flush();
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
    printf( "gps_q        : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_gps_queue ),
            ( unsigned ) GPS_QUEUE_DEPTH );
    printf( "baro_q       : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_baro_queue ),
            ( unsigned ) BARO_QUEUE_DEPTH );
    printf( "tx_q         : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_tx_queue ),
            ( unsigned ) TX_QUEUE_DEPTH );
    printf( "logger_q     : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_logger_queue ),
            ( unsigned ) LOGGER_QUEUE_DEPTH );
    printf( "imu_q        : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_imu_queue ),
            ( unsigned ) IMU_QUEUE_DEPTH );
    printf( "streaming    : baro=%-3s(%.2fHz)  gps=%-3s(%.2fHz)  imu=%-3s(%.2fHz)"
            "  hex=%-3s  state=%-3s  log=%-3s\n",
            s_stream_baro     ? "on"  : "off",
            s_stream_baro     ? 1000.0 / s_baro_interval_ms : 0.0,
            s_stream_gps      ? "on"  : "off",
            s_stream_gps      ? 1000.0 / s_gps_interval_ms  : 0.0,
            s_stream_imu      ? "on"  : "off",
            s_stream_imu      ? 1000.0 / s_imu_interval_ms  : 0.0,
            g_ubx_hex_enabled ? "on"  : "off",
            s_stream_state    ? "on"  : "off",
            s_log_enabled     ? "on"  : "off" );
}

// -- Command dispatch ----------------------------------------------------------

static void dispatch( const char* line, size_t len )
{
    if ( len == 0 ) return;

    // -- Utility -----------------------------------------------------------
    if ( strncmp( line, "help",   4 ) == 0 ) { cmd_help();   return; }
    if ( strncmp( line, "status", 6 ) == 0 ) { cmd_status(); return; }
    if ( strncmp( line, "top", 3 ) == 0 ) {
        const char* arg = line + 3;
        while ( *arg == ' ' ) arg++;
        const bool live = ( strncmp( arg, "live", 4 ) == 0 );
        if ( live ) {
            // Live mode: clear screen once, then overwrite in-place every 2 s.
            // ESC or any keypress exits.
            printf( "\x1b[2J\x1b[H" );  // clear + home
            stdio_flush();
            for ( ;; ) {
                printf( "\x1b[H" );     // cursor home — overwrite previous frame
                cmd_top();
                printf( "\x1b[K\n\x1b[2m  [top live — press any key to exit]\x1b[0m\x1b[K\n" );
                // Erase any lines below the current output that might be stale.
                printf( "\x1b[J" );
                stdio_flush();

                // Poll for keypress every 50 ms for up to 2 s.
                TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS( 2000 );
                bool quit = false;
                while ( xTaskGetTickCount() < deadline ) {
                    int c = getchar_timeout_us( 0 );
                    if ( c != PICO_ERROR_TIMEOUT && c < 256 ) { quit = true; break; }
                    vTaskDelay( pdMS_TO_TICKS( 50 ) );
                }
                if ( quit ) break;
            }
            printf( "\x1b[2J\x1b[H# " );  // restore prompt
            stdio_flush();
        } else {
            // One-shot: home+clear then render.
            printf( "\x1b[H\x1b[J" );
            cmd_top();
        }
        return;
    }
    // i2c scan disabled — corrupts running i2c task (baro/imu) due to repeated i2c_init
    // if ( strncmp( line, "i2c",    3 ) == 0 ) {
    //     cmd_i2c_scan();
    //     return;
    // }
    if ( strncmp( line, "clear",  5 ) == 0 ) {
        printf( "\x1b[2J\x1b[H" );
        stdio_flush();
        return;
    }

    // -- Streaming toggles -------------------------------------------------
    if ( strncmp( line, "baro",  4 ) == 0 ) {
        apply_on_off_hz( line, 4, s_stream_baro, s_baro_interval_ms );
        printf( "baro streaming: %s  @ %.2f Hz  (%lu ms)\n",
                s_stream_baro ? "on" : "off",
                s_stream_baro ? 1000.0 / s_baro_interval_ms : 0.0,
                ( unsigned long ) s_baro_interval_ms );
        return;
    }
    if ( strncmp( line, "gps",   3 ) == 0 ) {
        apply_on_off_hz( line, 3, s_stream_gps, s_gps_interval_ms );
        printf( "gps streaming: %s  @ %.2f Hz  (%lu ms)\n",
                s_stream_gps ? "on" : "off",
                s_stream_gps ? 1000.0 / s_gps_interval_ms : 0.0,
                ( unsigned long ) s_gps_interval_ms );
        return;
    }
    if ( strncmp( line, "imu",   3 ) == 0 ) {
        apply_on_off_hz( line, 3, s_stream_imu, s_imu_interval_ms );
        printf( "imu streaming: %s  @ %.2f Hz  (%lu ms)\n",
                s_stream_imu ? "on" : "off",
                s_stream_imu ? 1000.0 / s_imu_interval_ms : 0.0,
                ( unsigned long ) s_imu_interval_ms );
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

    // -- Recording control -------------------------------------------------
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

    // -- Flash inspection --------------------------------------------------
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

// -- USB / serial console task -------------------------------------------------
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
    TickType_t last_baro   = 0;
    TickType_t last_gps    = 0;
    TickType_t last_imu    = 0;
    TickType_t last_state  = 0;

    for ( ;; ) {
        // -- 1. Drain log queue --------------------------------------------
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

        // -- 2. Sensor streaming -------------------------------------------
        {
            TickType_t now     = xTaskGetTickCount();
            bool       flushed = false;

            if ( s_stream_baro &&
                 ( now - last_baro ) >= pdMS_TO_TICKS( s_baro_interval_ms ) ) {
                last_baro = now;
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
                flushed = true;
            }

            if ( s_stream_gps &&
                 ( now - last_gps ) >= pdMS_TO_TICKS( s_gps_interval_ms ) ) {
                last_gps = now;
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
                    printf( "[gps]  acquiring fix...\n" );
                }
                flushed = true;
            }

            if ( s_stream_imu &&
                 ( now - last_imu ) >= pdMS_TO_TICKS( s_imu_interval_ms ) ) {
                last_imu = now;
                ImuData m;
                if ( xQueuePeek( g_imu_queue, &m, 0 ) == pdTRUE )
                    printf( "[imu]  ax=%7.3f ay=%7.3f az=%7.3f m/s²"
                            "  gx=%8.3f gy=%8.3f gz=%8.3f °/s"
                            "  T=%.1f°C\n",
                            ( double ) m.accel_x_mss, ( double ) m.accel_y_mss,
                            ( double ) m.accel_z_mss,
                            ( double ) m.gyro_x_dps,  ( double ) m.gyro_y_dps,
                            ( double ) m.gyro_z_dps,  ( double ) m.temp_c );
                else
                    printf( "[imu]  no data yet\n" );
                flushed = true;
            }

            if ( s_stream_state &&
                 ( now - last_state ) >= pdMS_TO_TICKS( 1000 ) ) {
                last_state = now;
                printf( "[state] %s\n", flight_state_str( g_flight_state ) );
                flushed = true;
            }

            if ( flushed ) stdio_flush();
        }

        // -- 3. Read characters from USB CDC -------------------------------
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
static StackType_t  s_usb_stack[ 3072 ];

void usb_task_init()
{
    TaskHandle_t h = xTaskCreateStatic( usb_task, "usb", 3072,
                                         nullptr, tskIDLE_PRIORITY + 1,
                                         s_usb_stack, &s_usb_tcb );
    configASSERT( h );
    // TinyUSB IRQ fires on core 0; printf must live on the same core.
    vTaskCoreAffinitySet( h, 0x01 );
}
