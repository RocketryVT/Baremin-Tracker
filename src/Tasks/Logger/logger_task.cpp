#include "logger_task.hpp"
#include "shared.hpp"
#include "pico_logger.hpp"

#include <stdio.h>
#include <stdlib.h>   // abs

// Flash log starts at 1 MB offset — well past any RP2350 code image.
// Leaves ~3 MB for storage on a 4 MB Pico 2 board.
// At 50 Hz × 128 bytes/record -> ~8 minutes of flight data.
#define BAREMAN_LOG_FLASH_ADDR  0x00100000u

// Pre-launch circular buffer: 10 s × 50 Hz × 128 bytes = 64 000 bytes.
// Allocated from heap before the scheduler starts; freed after flush at launch.
#define BAREMAN_CIRC_RECORDS    500u

static Logger s_logger( sizeof( SigmaStorageFullRecord ),
                         BAREMAN_LOG_FLASH_ADDR,
                         nullptr );

// ── File-scope recording state ────────────────────────────────────────────────
// Moved from task-local to file-scope so the console API functions can inspect
// and modify them safely (logger task checks them on each iteration).
static volatile bool      s_use_circular      = true;
static volatile bool      s_recording_stopped = false;
static volatile bool      s_force_record_on   = false;
static          TaskHandle_t s_logger_handle  = nullptr;

// ── Console API ───────────────────────────────────────────────────────────────

TaskHandle_t logger_task_get_handle() { return s_logger_handle; }

void logger_task_force_record_on()
{
    if ( s_use_circular ) {
        // Set flag; the logger task (core 1) will flush the circular buffer and
        // switch to direct writes on its next iteration — keeping all flash
        // operations on core 1 where flash_safe_execute is set up.
        s_force_record_on = true;
    }
    // Un-stop recording in case it was previously stopped.
    s_recording_stopped = false;
}

void logger_task_force_record_off()
{
    s_recording_stopped = true;
}

// ── Flight state name helper ──────────────────────────────────────────────────
static const char* state_name( uint8_t s )
{
    switch ( static_cast<FlightState>( s ) ) {
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

// ── Read decoded ─────────────────────────────────────────────────────────────
// Walk XIP flash directly (read-only, no flash_safe_execute needed).
// Print each 128-byte SigmaStorageFullRecord in human-readable form.
// Stops at the first fully-erased (0xFF) record.
void logger_task_read_decoded()
{
    const uint8_t* base = reinterpret_cast<const uint8_t*>( XIP_BASE + BAREMAN_LOG_FLASH_ADDR );
    const uint8_t* end  = reinterpret_cast<const uint8_t*>( XIP_BASE + PICO_FLASH_SIZE_BYTES );

    int  count = 0;
    bool stop  = false;

    printf( "--- flash records from 0x%08X ---\n", BAREMAN_LOG_FLASH_ADDR );
    stdio_flush();

    for ( const uint8_t* page = base; page < end && !stop; page += FLASH_PAGE_SIZE ) {
        // Stop at the first fully-erased page (all 0xFF).
        bool page_empty = true;
        for ( size_t i = 0; i < FLASH_PAGE_SIZE && page_empty; i++ )
            if ( page[i] != 0xFF ) page_empty = false;
        if ( page_empty ) break;

        // Each 256-byte page holds 2 × 128-byte records.
        for ( size_t off = 0; off < FLASH_PAGE_SIZE && !stop; off += sizeof( SigmaStorageFullRecord ) ) {
            const auto* r    = reinterpret_cast<const SigmaStorageFullRecord*>( page + off );
            const auto* rb   = reinterpret_cast<const uint8_t*>( r );

            // Stop at the first fully-erased record.
            bool rec_empty = true;
            for ( size_t i = 0; i < sizeof( SigmaStorageFullRecord ) && rec_empty; i++ )
                if ( rb[i] != 0xFF ) rec_empty = false;
            if ( rec_empty ) { stop = true; break; }

            printf( "[%5d] t=%8lums  utc=%llu  %-16s  flags=0x%02X  sats=%u\n"
                    "        baro=%7.1fm  P=%8.0fPa  T=%5.2fC  mach=%.4f\n"
                    "        GPS: lat=%11.7f  lon=%12.7f  alt=%7.1fm\n"
                    "        quat: w=%8.5f  x=%8.5f  y=%8.5f  z=%8.5f\n"
                    "        RPY:  r=%7.2f  p=%7.2f  y=%7.2f deg\n"
                    "        accel: %7.4f  %7.4f  %7.4f g\n"
                    "        gyro:  %7.2f  %7.2f  %7.2f dps\n"
                    "        mag:   %7.2f  %7.2f  %7.2f uT\n"
                    "        vel:   %7.2f  %7.2f  %7.2f m/s (NED)\n",
                    count,
                    ( unsigned long ) r->boot_ms,
                    ( unsigned long long ) r->utc_unix_ms,
                    state_name( r->state ),
                    ( unsigned ) r->flags,
                    ( unsigned ) r->satellites,
                    ( double ) r->alt_baro_m,
                    ( double ) r->pressure_pa,
                    ( double ) r->temp_c,
                    ( double ) r->mach,
                    r->lat,
                    r->lon,
                    ( double ) r->alt_gps_m,
                    ( double ) r->q[0],
                    ( double ) r->q[1],
                    ( double ) r->q[2],
                    ( double ) r->q[3],
                    ( double ) r->rpy_deg[0],
                    ( double ) r->rpy_deg[1],
                    ( double ) r->rpy_deg[2],
                    ( double ) r->accel_g[0],
                    ( double ) r->accel_g[1],
                    ( double ) r->accel_g[2],
                    ( double ) r->gyro_dps[0],
                    ( double ) r->gyro_dps[1],
                    ( double ) r->gyro_dps[2],
                    ( double ) r->mag_uT[0],
                    ( double ) r->mag_uT[1],
                    ( double ) r->mag_uT[2],
                    ( double ) r->vel_ned_ms[0],
                    ( double ) r->vel_ned_ms[1],
                    ( double ) r->vel_ned_ms[2] );
            stdio_flush();
            count++;
        }
    }

    printf( "--- total: %d record%s ---\n", count, count == 1 ? "" : "s" );
    stdio_flush();
}

// ── Read CSV ─────────────────────────────────────────────────────────────────
// Same flash walk as read_decoded() but prints one CSV row per record so the
// output can be copy-pasted into a spreadsheet or piped to a file.
// Calls printf() directly (no 256-byte log_print limit).
void logger_task_read_csv()
{
    const uint8_t* base = reinterpret_cast<const uint8_t*>( XIP_BASE + BAREMAN_LOG_FLASH_ADDR );
    const uint8_t* end  = reinterpret_cast<const uint8_t*>( XIP_BASE + PICO_FLASH_SIZE_BYTES );

    // Header — columns match SigmaStorageFullRecord field order.
    printf( "boot_ms,utc_unix_ms,state,satellites,flags,"
            "lat,lon,alt_gps_m,alt_baro_m,pressure_pa,temp_c,mach,"
            "q_w,q_x,q_y,q_z,"
            "roll_deg,pitch_deg,yaw_deg,"
            "ax_g,ay_g,az_g,"
            "gx_dps,gy_dps,gz_dps,"
            "mx_uT,my_uT,mz_uT,"
            "vn_ms,ve_ms,vd_ms\n" );
    stdio_flush();

    int  count = 0;
    bool stop  = false;

    for ( const uint8_t* page = base; page < end && !stop; page += FLASH_PAGE_SIZE ) {
        bool page_empty = true;
        for ( size_t i = 0; i < FLASH_PAGE_SIZE && page_empty; i++ )
            if ( page[i] != 0xFF ) page_empty = false;
        if ( page_empty ) break;

        for ( size_t off = 0; off < FLASH_PAGE_SIZE && !stop;
              off += sizeof( SigmaStorageFullRecord ) )
        {
            const auto* r  = reinterpret_cast<const SigmaStorageFullRecord*>( page + off );
            const auto* rb = reinterpret_cast<const uint8_t*>( r );

            bool rec_empty = true;
            for ( size_t i = 0; i < sizeof( SigmaStorageFullRecord ) && rec_empty; i++ )
                if ( rb[i] != 0xFF ) rec_empty = false;
            if ( rec_empty ) { stop = true; break; }

            // Split across two printf calls for readability.
            // First half: timestamp + position + atmosphere + quaternion
            printf( "%lu,%llu,%s,%u,%u,"
                    "%.7f,%.7f,%.1f,%.1f,%.0f,%.2f,%.4f,"
                    "%.5f,%.5f,%.5f,%.5f,",
                    ( unsigned long ) r->boot_ms,
                    ( unsigned long long ) r->utc_unix_ms,
                    state_name( r->state ),
                    ( unsigned ) r->satellites,
                    ( unsigned ) r->flags,
                    r->lat,
                    r->lon,
                    ( double ) r->alt_gps_m,
                    ( double ) r->alt_baro_m,
                    ( double ) r->pressure_pa,
                    ( double ) r->temp_c,
                    ( double ) r->mach,
                    ( double ) r->q[0],
                    ( double ) r->q[1],
                    ( double ) r->q[2],
                    ( double ) r->q[3] );

            // Second half: attitude + IMU + navigation
            printf( "%.2f,%.2f,%.2f,"
                    "%.4f,%.4f,%.4f,"
                    "%.2f,%.2f,%.2f,"
                    "%.2f,%.2f,%.2f,"
                    "%.2f,%.2f,%.2f\n",
                    ( double ) r->rpy_deg[0],
                    ( double ) r->rpy_deg[1],
                    ( double ) r->rpy_deg[2],
                    ( double ) r->accel_g[0],
                    ( double ) r->accel_g[1],
                    ( double ) r->accel_g[2],
                    ( double ) r->gyro_dps[0],
                    ( double ) r->gyro_dps[1],
                    ( double ) r->gyro_dps[2],
                    ( double ) r->mag_uT[0],
                    ( double ) r->mag_uT[1],
                    ( double ) r->mag_uT[2],
                    ( double ) r->vel_ned_ms[0],
                    ( double ) r->vel_ned_ms[1],
                    ( double ) r->vel_ned_ms[2] );

            stdio_flush();
            count++;
        }
    }

    printf( "--- total: %d record%s ---\n", count, count == 1 ? "" : "s" );
    stdio_flush();
}

// ── Read raw ─────────────────────────────────────────────────────────────────
// Logger::read_memory() walks from log_base_addr, printing raw bytes when
// print_func is nullptr (our case).
void logger_task_read_raw()
{
    s_logger.read_memory();
}

// ── Erase ────────────────────────────────────────────────────────────────────
// Erase flash and re-initialize the logger so it is ready for a new flight.
// initialize(false) rescans without calling flash_safe_execute_core_init()
// again (that was already done by logger_task_init()).
void logger_task_erase()
{
    s_logger.erase_memory();
    s_logger.initialize( false );

    // Re-allocate circular buffer (freed by the previous flush, if any).
    s_logger.initialize_circular_buffer(
        BAREMAN_CIRC_RECORDS * sizeof( SigmaStorageFullRecord ) );

    // Reset recording flags so the device is ready for the next flight.
    s_use_circular      = true;
    s_recording_stopped = false;
    s_force_record_on   = false;
}

// ── Logger task ───────────────────────────────────────────────────────────────
static void logger_task( void* )
{
    log_print( "[logger] flash logger ready\n" );

    SigmaStorageFullRecord rec;
    for ( ;; ) {
        // Handle force-record-on from the serial console.  All flash writes
        // stay on core 1 (this task) so flash_safe_execute sees a consistent
        // core setup.
        if ( s_force_record_on && s_use_circular ) {
            log_print( "[logger] force record on — flushing circular buffer\n" );
            s_logger.flush_circular_buffer( true );
            s_use_circular    = false;
            s_force_record_on = false;
        }

        if ( xQueueReceive( g_logger_queue, &rec, portMAX_DELAY ) != pdTRUE ) {
            continue;
        }

        if ( s_recording_stopped ) {
            continue;   // console said stop — drop record
        }

        FlightState state = g_flight_state;

        if ( s_use_circular ) {
            s_logger.write_circular_buffer(
                reinterpret_cast<const uint8_t*>( &rec ) );

            if ( state != FlightState::GROUND_IDLE ) {
                // Rocket is off the pad — flush pre-launch buffer and switch
                // to direct paged writes for the rest of the flight.
                log_print( "[logger] launch — flushing pre-launch buffer\n" );
                s_logger.flush_circular_buffer( true );
                s_use_circular = false;
            }
        } else if ( state != FlightState::LANDED ) {
            // In flight: write_memory buffers internally, flushes to flash
            // every 2 records (2 × 128 bytes = one 256-byte flash page).
            s_logger.write_memory(
                reinterpret_cast<const uint8_t*>( &rec ), false );
        }
        // LANDED: silently drop — recording done, preserve flash.
    }
}

static StaticTask_t s_logger_tcb;
static StackType_t  s_logger_stack[ 1024 ];

void logger_task_init()
{
    // initialize() scans flash for the resume point and calls
    // flash_safe_execute_core_init() for multicore safety.  Called here,
    // before vTaskStartScheduler(), to match the pattern in active_drag_system
    // where logger.initialize() runs before the scheduler starts.
    s_logger.initialize( true );

    // Allocate pre-launch circular buffer (10 s × 50 Hz = 500 records = 64 KB).
    // Must be called before the scheduler starts (malloc from heap at init time).
    s_logger.initialize_circular_buffer(
        BAREMAN_CIRC_RECORDS * sizeof( SigmaStorageFullRecord ) );

    s_logger_handle = xTaskCreateStatic(
        logger_task, "log_flash", 1024,
        nullptr, tskIDLE_PRIORITY + 1,
        s_logger_stack, &s_logger_tcb );
    configASSERT( s_logger_handle );

    // Pin to core 1 — keeps flash writes off core 0 where the LoRa task runs.
    // Mirrors active_drag_system: vTaskCoreAffinitySet(logging_handle, 0x02).
    vTaskCoreAffinitySet( s_logger_handle, 0x02 );
}
