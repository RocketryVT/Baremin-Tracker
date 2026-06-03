#include "lora_task.hpp"
#include "shared.hpp"

#include <RadioLib.h>
#include "PicoHal.h"
#include "pico/time.h"
#include <string.h>

// -- SX1276 radio -------------------------------------------------------------
static PicoHal s_hal( spi1, Pins::LR_SCK, Pins::LR_MOSI, Pins::LR_MISO );
static SX1276  s_radio = new Module( &s_hal, Pins::LR_NSS, Pins::LR_DIO0,
                                     Pins::LR_NRESET, RADIOLIB_NC );

// -- Radio init ----------------------------------------------------------------
static bool radio_init()
{
    ConfigLoRa_t config;
    config.frequency       = static_cast<float>( LoRaCfg::FREQ_HZ ) / 1e6f;
    config.bandwidth       = static_cast<float>( LoRaCfg::BW );
    config.spreadingFactor = LoRaCfg::SF;
    config.codingRate      = LoRaCfg::CR;
    config.syncWord        = LoRaCfg::SYNC_WORD;
    config.power           = LoRaCfg::TX_DBM;
    config.preambleLength  = LoRaCfg::PREAMBLE;

    int state = s_radio.begin( config );

    if ( state != RADIOLIB_ERR_NONE ) {
        log_print( "[lora] SX1276 init failed, code: %d\n", state );
        return false;
    }

    log_print( "[lora] SX1276 ready — %.3f MHz  SF%u  BW%u kHz  CR4/%u  %d dBm\n",
               static_cast<float>( LoRaCfg::FREQ_HZ ) / 1e6f,
               LoRaCfg::SF, LoRaCfg::BW, LoRaCfg::CR, LoRaCfg::TX_DBM );
    return true;
}

// -- Helpers -------------------------------------------------------------------

// Push a serialized frame onto the TX queue. Non-blocking — drops if full.
static void enqueue( const uint8_t* buf, size_t len )
{
    if ( len == 0 || len > sizeof( TxFrame::buf ) ) return;
    TxFrame f;
    memcpy( f.buf, buf, len );
    f.len = static_cast<uint16_t>( len );
    if ( xQueueSend( g_tx_queue, &f, 0 ) != pdTRUE )
        log_print( "[lora] tx queue full — frame dropped\n" );
}

// Days since Unix epoch (1970-01-01) for a given date, using the Gregorian
// calendar formula.  Valid for any date from 1970 onward.
static uint32_t days_since_epoch( uint16_t year, uint8_t month, uint8_t day )
{
    // Shift so March = month 0, making leap-day the last day of the "year".
    uint32_t m = month;
    uint32_t y = year;
    if ( m <= 2 ) { m += 10; y -= 1; } else { m -= 3; }
    uint32_t c  = y / 100;
    uint32_t yr = y % 100;
    // Days from epoch to start of this day (Gregorian proleptic formula).
    return ( 146097u * c ) / 4u
         + ( 1461u   * yr ) / 4u
         + ( 153u * m + 2u ) / 5u
         + day
         - 719469u;   // offset so 1970-01-01 = 0
}

// Build and enqueue a UTC time-sync frame.
static void enqueue_time_sync( const GpsData& gps )
{
    SIGMA::TimeSyncData ts;
    ts.boot_us    = static_cast<uint32_t>( time_us_64() );
    ts.gps_tow_ms = gps.utc_ms;  // ms since midnight UTC (no full TOW available)

    // Reconstruct Unix epoch ms from GPS date + time-of-day.
    uint64_t utc_unix_ms = 0;
    if ( gps.utc_year >= 2020 ) {
        const uint32_t days = days_since_epoch( gps.utc_year, gps.utc_month, gps.utc_day );
        utc_unix_ms = static_cast<uint64_t>( days ) * 86400000ULL
                    + static_cast<uint64_t>( gps.utc_ms );
    }
    ts.utc_unix_ms = utc_unix_ms;
    ts.flags       = SIGMA::FLAG_GPS_VALID | SIGMA::FLAG_TIME_VALID;

    uint8_t frame[ SIGMA::MAX_FRAME ];
    size_t  n = ts.serialize( frame, sizeof( frame ) );
    enqueue( frame, n );
}

// Build and enqueue a GPS_NAV frame.
static void enqueue_gps_nav( const GpsData& gps )
{
    SIGMA::GpsNavData gn;
    gn.lat          = gps.lat;
    gn.lon          = gps.lon;
    gn.alt_gps_m    = static_cast<float>( gps.alt_m );
    gn.vel_ned_ms[0]= static_cast<float>( gps.vel_north_mms ) * 0.001f;
    gn.vel_ned_ms[1]= static_cast<float>( gps.vel_east_mms  ) * 0.001f;
    gn.vel_ned_ms[2]= static_cast<float>( gps.vel_down_mms  ) * 0.001f;
    gn.satellites   = gps.satellites;
    gn.flags        = SIGMA::FLAG_GPS_VALID;

    uint8_t frame[ SIGMA::MAX_FRAME ];
    size_t  n = gn.serialize( frame, sizeof( frame ) );
    enqueue( frame, n );
}

// Build and enqueue a NAV_STATE frame.
// No IMU/fusion on this board: identity quaternion, baro-only altitude,
// no fused velocity.
static void enqueue_nav_state( const BaroData& baro )
{
    const float alt_m = static_cast<float>( baro.altitude_dm ) * 0.1f;

    SIGMA::NavStateData ns;
    ns.alt_baro_m  = alt_m;
    ns.alt_fused_m = alt_m;   // no fusion — mirror baro
    // vel_ned_ms stays zero, q stays identity {1,0,0,0}
    ns.state = g_flight_state;
    ns.flags = SIGMA::FLAG_BARO_VALID;

    uint8_t frame[ SIGMA::MAX_FRAME ];
    size_t  n = ns.serialize( frame, sizeof( frame ) );
    enqueue( frame, n );
}

// -- Scheduler task (core 0, pri 3) -------------------------------------------
// Runs at exactly 1 Hz using vTaskDelayUntil.
// Tracks coarser rates with tick counters:
//   Every tick  (1 s)  — NavState + GpsNav
//   Every 5     (5 s)  — burst of 5 TimeSync frames
static void lora_sched_task( void* )
{
    // Wait for the radio task to finish initializing.
    vTaskDelay( pdMS_TO_TICKS( 12000 ) );

    log_print( "[lora] scheduler running: NavState@1Hz  GpsNav@1Hz  TimeSync@0.2Hz×5\n" );

    TickType_t wake       = xTaskGetTickCount();
    uint32_t   tick_count = 0;

    for ( ;; ) {
        vTaskDelayUntil( &wake, pdMS_TO_TICKS( 1000 ) );
        tick_count++;

        GpsData  gps  = {};
        BaroData baro = {};
        const bool have_gps  = ( xQueuePeek( g_gps_queue,  &gps,  0 ) == pdTRUE );
        const bool have_baro = ( xQueuePeek( g_baro_queue, &baro, 0 ) == pdTRUE );

        // 1 Hz — NavState (baro only)
        if ( have_baro )
            enqueue_nav_state( baro );

        // 1 Hz — GpsNav
        if ( have_gps )
            enqueue_gps_nav( gps );

        // 0.2 Hz — burst of 5 TimeSync frames every 5 seconds
        if ( tick_count % 5 == 0 && have_gps ) {
            for ( int i = 0; i < 5; ++i )
                enqueue_time_sync( gps );
        }
    }
}

// -- Radio task (core 0, pri 4) ------------------------------------------------
// Blocks on g_tx_queue and transmits each frame in order.
static void lora_radio_task( void* )
{
    for ( int i = 10; i > 0; --i ) {
        log_print( "[lora] radio init in %d...\n", i );
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }

    if ( !radio_init() ) {
        log_print( "[lora] init failed — radio task halting\n" );
        while ( true ) vTaskDelay( portMAX_DELAY );
    }

    TxFrame  f;
    uint32_t tx_ok  = 0;
    uint32_t tx_err = 0;

    for ( ;; ) {
        if ( xQueueReceive( g_tx_queue, &f, portMAX_DELAY ) != pdTRUE )
            continue;

        int state = s_radio.transmit( f.buf, f.len );
        if ( state != RADIOLIB_ERR_NONE ) {
            tx_err++;
            log_print( "[lora] tx failed code=%d  len=%u  ok=%lu err=%lu\n",
                       state, ( unsigned ) f.len,
                       ( unsigned long ) tx_ok, ( unsigned long ) tx_err );
        } else {
            tx_ok++;
            if ( tx_ok % 10 == 0 )
                log_print( "[lora] tx ok=%lu err=%lu\n",
                           ( unsigned long ) tx_ok, ( unsigned long ) tx_err );
        }
    }
}

// -- Static task storage -------------------------------------------------------
static StaticTask_t s_sched_tcb;
static StackType_t  s_sched_stack[ 1024 ];

static StaticTask_t s_radio_tcb;
static StackType_t  s_radio_stack[ 2048 ];

void lora_task_init()
{
    TaskHandle_t h;

    h = xTaskCreateStatic( lora_sched_task, "lora_sched", 1024,
                           NULL, tskIDLE_PRIORITY + 3,
                           s_sched_stack, &s_sched_tcb );
    configASSERT( h );
    vTaskCoreAffinitySet( h, ( 1u << 0 ) );

    h = xTaskCreateStatic( lora_radio_task, "lora_radio", 2048,
                           NULL, tskIDLE_PRIORITY + 4,
                           s_radio_stack, &s_radio_tcb );
    configASSERT( h );
    vTaskCoreAffinitySet( h, ( 1u << 0 ) );
}
