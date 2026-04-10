#include "baro_task.hpp"
#include "shared.hpp"
#include "ms5607.hpp"

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <cstdlib>
#include <string.h>

// ── Flight detection thresholds ───────────────────────────────────────────────
static constexpr float LAUNCH_AGL_M        = 20.0f;  // m AGL to declare launch
static constexpr float APOGEE_DELTA_M      =  5.0f;  // m below peak to declare apogee
static constexpr float LANDED_AGL_M        =  5.0f;  // m AGL threshold for landing
static constexpr int   LANDED_STABLE_COUNT = 500;    // ~10 s at 50 Hz
static constexpr int   CALIB_SAMPLES       = 50;     // 5 s at 10 Hz

// ── MS5607 instance ───────────────────────────────────────────────────────────
static MS5607 s_baro( i2c0 );

// ── Task storage ──────────────────────────────────────────────────────────────
static StaticTask_t s_sample_handler_tcb;
static StackType_t  s_sample_handler_stack[ 512 ];

static StaticTask_t s_update_tcb;
static StackType_t  s_update_stack[ 512 ];

static StaticTask_t s_reader_tcb;
static StackType_t  s_reader_stack[ 512 ];

// ── Reader task — flight state machine ────────────────────────────────────────
// Phases:
//   1. Ground calibration: average CALIB_SAMPLES at 10 Hz to find ground_alt_m.
//   2. Main loop: state transitions + dual-rate queue push (10 Hz pad / 50 Hz flight).
//
// State transitions (baro only — no IMU):
//   GROUND_IDLE     -> POWERED_ASCENT  when AGL ≥ LAUNCH_AGL_M
//   POWERED_ASCENT  -> APOGEE          when alt drops ≥ APOGEE_DELTA_M below peak
//   APOGEE          -> DESCENT_DROGUE  immediately (single-sample transition)
//   DESCENT_DROGUE  -> LANDED          when AGL ≤ LANDED_AGL_M for LANDED_STABLE_COUNT samples
static void baro_reader_task( void* )
{
    // ── Phase 1: ground calibration ───────────────────────────────────────────
    float calib_sum   = 0.0f;
    int   calib_count = 0;
    while ( calib_count < CALIB_SAMPLES )
    {
        calib_sum += static_cast<float>( s_baro.get_altitude() ) * 0.1f;
        calib_count++;
        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }
    const float ground_alt_m = calib_sum / static_cast<float>( CALIB_SAMPLES );
    log_print( "[baro] ground alt %.1f m (avg %d samples)\n",
               ground_alt_m, CALIB_SAMPLES );

    float peak_alt_m   = ground_alt_m;
    int   landed_count = 0;

    // ── Phase 2: main loop ────────────────────────────────────────────────────
    for ( ;; )
    {
        FlightState state = g_flight_state;   // local snapshot

        float alt_m = static_cast<float>( s_baro.get_altitude() ) * 0.1f;
        float agl_m = alt_m - ground_alt_m;

        // ── State transitions ──────────────────────────────────────────────
        switch ( state )
        {
            case FlightState::GROUND_IDLE:
                if ( agl_m >= LAUNCH_AGL_M ) {
                    g_flight_state = FlightState::POWERED_ASCENT;
                    log_print( "[baro] LAUNCH detected — AGL %.1f m\n", agl_m );
                }
                break;

            case FlightState::POWERED_ASCENT:
                if ( alt_m > peak_alt_m ) {
                    peak_alt_m = alt_m;
                }
                if ( ( peak_alt_m - alt_m ) >= APOGEE_DELTA_M ) {
                    g_flight_state = FlightState::APOGEE;
                    log_print( "[baro] APOGEE detected — peak %.1f m  now %.1f m\n",
                               peak_alt_m, alt_m );
                }
                break;

            case FlightState::APOGEE:
                g_flight_state = FlightState::DESCENT_DROGUE;
                break;

            case FlightState::DESCENT_DROGUE:
                if ( agl_m <= LANDED_AGL_M ) {
                    landed_count++;
                    if ( landed_count >= LANDED_STABLE_COUNT ) {
                        g_flight_state = FlightState::LANDED;
                        log_print( "[baro] LANDED — AGL %.1f m\n", agl_m );
                    }
                } else {
                    landed_count = 0;
                }
                break;

            default:
                // LANDED / FAULT — keep running so LoRa task gets fresh baro data.
                break;
        }

        // ── Refresh baro queue for LoRa task ──────────────────────────────
        BaroData fresh = {
            .pressure_pa      = s_baro.get_pressure(),
            .temperature_cdeg = s_baro.get_temperature(),
            .altitude_dm      = s_baro.get_altitude(),
        };
        xQueueOverwrite( g_baro_queue, &fresh );

        // ── Build flash record and push to logger queue ───────────────────
        SigmaStorageFullRecord rec;
        memset( &rec, 0, sizeof( rec ) );
        rec.boot_ms     = to_ms_since_boot( get_absolute_time() );
        rec.alt_baro_m  = alt_m;
        rec.pressure_pa = static_cast<float>( fresh.pressure_pa );
        rec.temp_c      = static_cast<float>( fresh.temperature_cdeg ) * 0.01f;
        rec.state       = static_cast<uint8_t>( g_flight_state );
        rec.flags       = SIGMA_FLAG_BARO_VALID;
        rec.q[0]        = 1.0f;   // identity quaternion (no IMU)

        GpsData gps;
        if ( xQueuePeek( g_gps_queue, &gps, 0 ) == pdTRUE ) {
            rec.lat        = gps.lat;
            rec.lon        = gps.lon;
            rec.alt_gps_m  = static_cast<float>( gps.alt_m );
            rec.satellites = gps.satellites;
            rec.flags     |= SIGMA_FLAG_GPS_VALID;
        }

        // Non-blocking send — drop record if logger task has fallen behind.
        xQueueSend( g_logger_queue, &rec, 0 );

        // ── Rate control: 10 Hz on pad, 50 Hz in flight ───────────────────
        if ( g_flight_state == FlightState::GROUND_IDLE ) {
            vTaskDelay( pdMS_TO_TICKS( 100 ) );
        } else {
            vTaskDelay( pdMS_TO_TICKS( 20 ) );
        }
    }
}

// ── Init ──────────────────────────────────────────────────────────────────────
void baro_task_init()
{
    // I2C0 peripheral — 400 kHz fast mode
    i2c_init( i2c0, 400'000 );
    gpio_set_function( Pins::BARO_SDA, GPIO_FUNC_I2C );
    gpio_set_function( Pins::BARO_SCL, GPIO_FUNC_I2C );
    gpio_pull_up( Pins::BARO_SDA );
    gpio_pull_up( Pins::BARO_SCL );

    // Read PROM calibration coefficients (blocks ~700 ms in initialize()).
    s_baro.initialize();

    // Sample-handler task: woken by ISR alarm callback to run each conversion
    // step in task context (safe for i2c_read_blocking / i2c_write_blocking).
    s_baro.sample_handler_task_handle = xTaskCreateStatic(
        MS5607::ms5607_sample_handler, "baro_irq", 512,
        &s_baro, tskIDLE_PRIORITY + 3,
        s_sample_handler_stack, &s_sample_handler_tcb );
    configASSERT( s_baro.sample_handler_task_handle );

    // Update task: triggers a new sample at MS5607_SAMPLE_RATE_HZ.
    s_baro.update_task_handle = xTaskCreateStatic(
        MS5607::update_ms5607_task, "baro_upd", 512,
        &s_baro, tskIDLE_PRIORITY + 2,
        s_update_stack, &s_update_tcb );
    configASSERT( s_baro.update_task_handle );

    // Reader task: runs calibration, then state machine + queue push.
    TaskHandle_t h = xTaskCreateStatic(
        baro_reader_task, "baro_rd", 512,
        NULL, tskIDLE_PRIORITY + 1,
        s_reader_stack, &s_reader_tcb );
    configASSERT( h );
}

