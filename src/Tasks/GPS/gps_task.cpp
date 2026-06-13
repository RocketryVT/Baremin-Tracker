#include "gps_task.hpp"
#include "shared.hpp"

// Include SDK UART headers before gps_driver.hpp so that uart_inst_t and
// uart0 are declared in the global namespace first.
#include "hardware/uart.h"

#include "gps/gps_driver.hpp"
#include <cstddef>
#include <new>
#include <cstdio>

// -- Lazy-initialized driver storage -------------------------------------------
using Driver = gps::PicoGpsDriver;

alignas( Driver ) static uint8_t s_driver_buf[ sizeof( Driver ) ];

static Driver* s_driver = nullptr;

static_assert(HAS_GPS, "bareman_tracker requires GPS support");
static_assert(Board::GpsCount > 0, "bareman_tracker requires Board::Gpses[0]");
static_assert(Board::Gpses[0].bus == Board::Bus::UART0,
              "bareman_tracker GPS task currently supports UART0 only");
static_assert(Board::Gpses[0].nav_hz > 0, "GPS nav_hz must be non-zero");
static_assert(Board::Gpses[0].nav_hz <= Board::spec_of(Board::Gpses[0].model).max_nav_hz,
              "GPS nav_hz exceeds selected receiver spec");

static constexpr Board::GpsInstance GPS = Board::Gpses[0];
static constexpr uint16_t GPS_RATE_MS = static_cast<uint16_t>(1000u / GPS.nav_hz);

// -- GPS task ------------------------------------------------------------------
static void gps_task( void* param )
{
    ( void ) param;

    // Driver owns UART setup, autobaud detection, baud switching, and RX mode.
    s_driver = new( s_driver_buf ) Driver( gps::PicoGpsConfig{
        .uart = uart0,
        .tx_pin = Pins::GPS_UART_TX,
        .rx_pin = Pins::GPS_UART_RX,
        .desired_baud = GPS.baud,
        .rx_mode = gps::UartRxMode::DmaRing,
        .dma_ring_size = gps::DmaRingBufferSize::Bytes1K,
    } );

    // Wait for the M9N to finish its internal boot before accepting config.
    vTaskDelay( pdMS_TO_TICKS( 500 ) );

    // -- GPS init sequence --------------------------------------------------------
    // Drain helper: pump the driver RX buffer for `ms` milliseconds in 10 ms slices.
    auto drain = [&]( uint32_t ms ) {
        const uint32_t slices = ms / 10;
        for ( uint32_t i = 0; i < slices; ++i ) {
            uint8_t tmp[128];
            const std::size_t n = s_driver->read_raw( tmp, sizeof( tmp ) );
            s_driver->feed_ubx_only( tmp, n );
            vTaskDelay( pdMS_TO_TICKS( 10 ) );
        }
    };

    auto send = [&]( const gps::UbxFrame& f, uint32_t ms = 100 ) {
        s_driver->send_ubx( f );
        drain( ms );
    };

    if ( !s_driver->initialized() ) {
        log_print( "[gps] autobaud failed; no checksum-valid GPS UART data detected\n" );
    } else {
        log_print( "[gps] autobaud detected %lu baud; target %lu baud %s\n",
                   ( unsigned long ) s_driver->detected_baud(),
                   ( unsigned long ) s_driver->desired_baud(),
                   s_driver->baud_change_ok() ? "ok" : "failed" );
    }

    // Step 1: CFG-PRT — sets UBX-in/UBX-out atomically at the driver's current baud.
    // Drain for 500 ms afterwards to clear residual NMEA bursts after port reset.
    send( gps::Ubx::cfg_prt(
        gps::Port::UART1, s_driver->current_baud(),
        gps::InProto::UBX,
        gps::OutProto::UBX ), 500 );

    // Step 2: rate and fix mode.
    send( gps::Ubx::valset_rate_meas( GPS_RATE_MS, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_rate_nav(   1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_fix_mode(   3, gps::ValLayer::RAM ) ); // AUTO

    // Step 3: message enables.
    send( gps::Ubx::valset_nav_pvt_uart1( 1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_nav_dop_uart1( 1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_nav_odo_uart1( 1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_nav_sat_uart1( 1, gps::ValLayer::RAM ) );

    // Step 4: dynmodel last — triggers a nav-engine restart on the M9N.
    send( gps::Ubx::valset_dyn_model( gps::Ubx::DynModel::Airborne4g,
                                      gps::ValLayer::RAM ), 500 );

    log_print( "[gps] %s init done — %lu baud, AIR4, %u Hz, UBX-only\n",
               Board::spec_of(GPS.model).name,
               ( unsigned long ) s_driver->current_baud(),
               ( unsigned ) GPS.nav_hz );
    log_print( "[gps] UART0  RX=GPIO%u  TX=GPIO%u  DMA ch%d  ring=%luB\n",
               Pins::GPS_UART_RX, Pins::GPS_UART_TX,
               s_driver->dma_channel(),
               ( unsigned long ) s_driver->dma_ring_size_bytes() );

    bool     had_fix      = false;
    uint32_t diag_ticks   = 0;
    uint32_t last_ubx_pvt = 0;

    for ( ;; ) {
        // Drain the DMA ring buffer — feed bytes to the UBX parser.
        uint8_t rx_buf[ 256 ];
        const std::size_t n = s_driver->read_raw( rx_buf, sizeof( rx_buf ) );

        s_driver->feed_ubx_only( rx_buf, n );

        if ( g_ubx_hex_enabled && n > 0 ) {
            char hex_line[ 56 ];   // "[x]" + 16×" XX" + "\n" + NUL = 55 chars
            for ( std::size_t i = 0; i < n; i += 16 ) {
                int pos = 0;
                pos += snprintf( hex_line + pos, sizeof( hex_line ) - (size_t)pos, "[x]" );
                for ( std::size_t j = i; j < n && j < i + 16; ++j )
                    pos += snprintf( hex_line + pos, sizeof( hex_line ) - (size_t)pos,
                                     " %02X", rx_buf[j] );
                snprintf( hex_line + pos, sizeof( hex_line ) - (size_t)pos, "\n" );
                log_print( "%s", hex_line );
            }
        }

        const gps::Coordinate& c = s_driver->coordinate();

        if ( c.valid ) {
            if ( !had_fix ) {
                log_print( "[gps] fix acquired: %.7f, %.7f  alt %.1f m  sats %d\n",
                           c.latitude, c.longitude,
                           ( double ) c.altitude, c.satellites );
                had_fix = true;
            }

            GpsData data;
            data.lat           = c.latitude;
            data.lon           = c.longitude;
            data.alt_m         = static_cast<double>( c.altitude );
            data.satellites    = static_cast<uint8_t>( c.satellites );
            data.speed_mps     = c.speed_mps;
            data.course_deg    = c.course_deg;
            data.utc_ms        = c.utc_ms;
            data.utc_year      = c.utc_year;
            data.utc_month     = c.utc_month;
            data.utc_day       = c.utc_day;
            data.vel_north_mms = c.vel_north_mms;
            data.vel_east_mms  = c.vel_east_mms;
            data.vel_down_mms  = c.vel_down_mms;
            data.fix_type      = c.fix_type == gps::FixType::Fix3D  ? 3 :
                                 c.fix_type == gps::FixType::Fix2D  ? 2 :
                                 c.fix_type == gps::FixType::GnssDR ? 4 : 0;
            data.hdop          = c.hdop;
            data.vdop          = c.vdop;
            data.best_cno      = c.best_cno;
            data.num_sv_used   = c.num_sv_used;

            xQueueOverwrite( g_gps_queue, &data );

        } else if ( had_fix ) {
            log_print( "[gps] fix lost\n" );
            had_fix = false;
        }

        // Log parser diagnostics every 5 s.
        diag_ticks += 20;
        if ( diag_ticks >= 5000 ) {
            diag_ticks = 0;
            const gps::Diagnostics& d = s_driver->diagnostics();
            uint32_t new_pvt = d.ubx_pvt - last_ubx_pvt;
            last_ubx_pvt = d.ubx_pvt;
            log_print( "[gps] diag: frames=%lu pvt=%lu(+%lu) dop=%lu odo=%lu"
                       " ack=%lu nak=%lu  ring_avail=%lu\n",
                       ( unsigned long ) d.ubx_frames,
                       ( unsigned long ) d.ubx_pvt,
                       ( unsigned long ) new_pvt,
                       ( unsigned long ) d.ubx_dop,
                       ( unsigned long ) d.ubx_odo,
                       ( unsigned long ) d.ubx_ack,
                       ( unsigned long ) d.ubx_nak,
                       ( unsigned long ) s_driver->rx_available() );
        }

        // 20 ms sleep — the configured driver RX mode buffers incoming bytes while
        // the task is suspended.
        vTaskDelay( pdMS_TO_TICKS( 20 ) );
    }
}

static StaticTask_t s_gps_tcb;
static StackType_t  s_gps_stack[ 1536 ];

void gps_task_init()
{
    TaskHandle_t h = xTaskCreateStatic( gps_task, "gps", 1536,
                                        NULL, tskIDLE_PRIORITY + 3,
                                        s_gps_stack, &s_gps_tcb );
    configASSERT( h );
    vTaskCoreAffinitySet( h, ( 1u << 0 ) );
}
