#include "gps_task.hpp"
#include "shared.hpp"

// Include SDK UART/DMA headers before gps_driver.hpp so that uart_inst_t and
// uart0 are declared in the global namespace first.
#include "hardware/uart.h"
#include "hardware/dma.h"

#include "gps/gps_driver.hpp"
#include <new>
#include <cstdio>
#include <cstring>

// -- DMA circular RX buffer ---------------------------------------------------
// One DMA channel in ring mode drains the UART0 RX FIFO continuously into a
// 1024-byte buffer.  The Pico ring mode constrains the write address to a
// power-of-2-aligned region, so we align the buffer to its own size.
//
// The GPS task tracks s_rx_read, advancing it modulo BUF_SIZE as it consumes
// bytes.  The DMA write pointer wraps automatically; data between s_rx_read and
// the current DMA write position is unread.
//
// No locking needed: DMA writes from HW, task reads — they access different
// cache lines and the read pointer is only touched by one FreeRTOS task.

static constexpr uint32_t DMA_BUF_SIZE = 1024u;   // must be a power of 2

// Aligned to BUF_SIZE so ring_size bits wrap the write address correctly.
alignas( DMA_BUF_SIZE ) static uint8_t s_rx_dma_buf[ DMA_BUF_SIZE ];

static int      s_dma_chan   = -1;
static uint32_t s_rx_read   = 0;   // next byte index to consume (0..BUF_SIZE-1)

// Return how many unread bytes are available.
static inline uint32_t dma_rx_available() noexcept
{
    // DMA write address wraps within s_rx_dma_buf due to ring mode.
    const uint32_t write_ptr =
        dma_channel_hw_addr( s_dma_chan )->write_addr;
    const uint32_t write_idx =
        static_cast<uint32_t>( write_ptr -
                               reinterpret_cast<uint32_t>( s_rx_dma_buf ) )
        & ( DMA_BUF_SIZE - 1u );
    return ( write_idx - s_rx_read ) & ( DMA_BUF_SIZE - 1u );
}

// Read up to `max_bytes` from the DMA ring buffer into `dst`.
// Returns actual count copied.
static uint32_t dma_rx_read( uint8_t* dst, uint32_t max_bytes ) noexcept
{
    const uint32_t avail = dma_rx_available();
    const uint32_t n     = ( avail < max_bytes ) ? avail : max_bytes;

    for ( uint32_t i = 0; i < n; ++i ) {
        dst[i] = s_rx_dma_buf[ s_rx_read & ( DMA_BUF_SIZE - 1u ) ];
        s_rx_read = ( s_rx_read + 1u ) & ( DMA_BUF_SIZE - 1u );
    }
    return n;
}

// Start (or re-start) the DMA channel.
// Call once at startup and again after a baud-rate change so the channel
// descriptor reflects any UART address changes (the UART DREQ is fixed, but
// re-claiming ensures a clean state).
static void dma_rx_start( uart_inst_t* uart ) noexcept
{
    if ( s_dma_chan >= 0 ) {
        dma_channel_abort( s_dma_chan );
    } else {
        s_dma_chan = dma_claim_unused_channel( true );
    }

    dma_channel_config cfg = dma_channel_get_default_config( s_dma_chan );
    channel_config_set_transfer_data_size( &cfg, DMA_SIZE_8 );
    channel_config_set_read_increment( &cfg, false );   // UART DR is fixed
    channel_config_set_write_increment( &cfg, true );
    channel_config_set_dreq( &cfg, uart_get_dreq( uart, false ) );  // RX DREQ
    // Ring mode: wrap write address every DMA_BUF_SIZE bytes.
    // ring_size = log2(DMA_BUF_SIZE) = 10
    channel_config_set_ring( &cfg, true /* write */, 10u /* log2(1024) */ );

    dma_channel_configure(
        s_dma_chan,
        &cfg,
        s_rx_dma_buf,               // write to ring buffer
        &uart_get_hw( uart )->dr,   // read from UART data register
        0xFFFFFFFFu,                // transfer count — effectively infinite
        true                        // start immediately
    );
}

// -- Lazy-initialized driver storage -------------------------------------------
using Driver    = gps::GpsDriver<gps::UartTransport>;
using Transport = gps::UartTransport;

alignas( Transport ) static uint8_t s_transport_buf[ sizeof( Transport ) ];
alignas( Driver )    static uint8_t s_driver_buf   [ sizeof( Driver )    ];

static Transport* s_transport = nullptr;
static Driver*    s_driver    = nullptr;

// -- GPS task ------------------------------------------------------------------
static void gps_task( void* param )
{
    ( void ) param;

    // Construct transport at 38400 — the M9N factory default.
    s_transport = new( s_transport_buf ) Transport(
        uart0, Pins::GPS_UART_TX, Pins::GPS_UART_RX, 38400 );
    s_driver    = new( s_driver_buf ) Driver( *s_transport );

    // Start DMA ring-buffer drain on UART0 RX.
    dma_rx_start( uart0 );

    // Wait for the M9N to finish its internal boot before accepting config.
    vTaskDelay( pdMS_TO_TICKS( 500 ) );

    // -- GPS init sequence --------------------------------------------------------
    // Drain helper: pump the DMA ring buffer for `ms` milliseconds in 10 ms slices.
    auto drain = [&]( uint32_t ms ) {
        const uint32_t slices = ms / 10;
        for ( uint32_t i = 0; i < slices; ++i ) {
            uint8_t tmp[128];
            uint32_t n = dma_rx_read( tmp, sizeof( tmp ) );
            for ( uint32_t j = 0; j < n; ++j )
                s_driver->ubx_parser().feed( tmp[j] );
            vTaskDelay( pdMS_TO_TICKS( 10 ) );
        }
    };

    auto send = [&]( const gps::UbxFrame& f, uint32_t ms = 100 ) {
        s_driver->send_ubx( f );
        drain( ms );
    };

    // Step 1: CFG-PRT — sets UBX-in/UBX-out atomically.
    // Drain for 500 ms afterwards to clear residual NMEA burst after port reset.
    send( gps::Ubx::cfg_prt(
        gps::Port::UART1, 38400,
        gps::InProto::UBX,
        gps::OutProto::UBX ), 500 );

    // Step 2: switch baud rate to 115200.
    // Send at 38400; module switches immediately on receipt.  No ACK at old baud.
    // Re-start DMA after changing the host UART so the DREQ mapping stays valid.
    s_driver->send_ubx( gps::Ubx::valset_uart1_baud( 115200, gps::ValLayer::RAM ) );
    vTaskDelay( pdMS_TO_TICKS( 10 ) );   // let TX FIFO drain before changing baud
    dma_channel_abort( s_dma_chan );      // pause DMA while UART re-inits
    uart_set_baudrate( uart0, 115200 );
    s_rx_read = 0;                        // discard any bytes from the old baud
    memset( s_rx_dma_buf, 0, sizeof( s_rx_dma_buf ) );
    dma_rx_start( uart0 );               // restart DMA at 115200
    drain( 100 );

    // Step 3: rate and fix mode.
    send( gps::Ubx::valset_rate_meas( 40, gps::ValLayer::RAM ) ); // 25 Hz
    send( gps::Ubx::valset_rate_nav(   1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_fix_mode(   3, gps::ValLayer::RAM ) ); // AUTO

    // Step 4: message enables.
    send( gps::Ubx::valset_nav_pvt_uart1( 1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_nav_dop_uart1( 1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_nav_odo_uart1( 1, gps::ValLayer::RAM ) );
    send( gps::Ubx::valset_nav_sat_uart1( 1, gps::ValLayer::RAM ) );

    // Step 5: dynmodel last — triggers a nav-engine restart on the M9N. (8 = AIR 4G)
    send( gps::Ubx::valset_dyn_model( 8, gps::ValLayer::RAM ), 500 ); // AIR4

    log_print( "[gps] NEO-M9N init done — 115200 baud, AIR4, 25 Hz, UBX-only\n" );
    log_print( "[gps] UART0  RX=GPIO%u  TX=GPIO%u  DMA ch%d  ring=1024B\n",
               Pins::GPS_UART_RX, Pins::GPS_UART_TX, s_dma_chan );

    bool     had_fix      = false;
    uint32_t diag_ticks   = 0;
    uint32_t last_ubx_pvt = 0;

    for ( ;; ) {
        // Drain the DMA ring buffer — feed bytes to the UBX parser.
        uint8_t rx_buf[ 256 ];
        const uint32_t n = dma_rx_read( rx_buf, sizeof( rx_buf ) );

        for ( uint32_t i = 0; i < n; ++i )
            s_driver->ubx_parser().feed( rx_buf[i] );

        if ( g_ubx_hex_enabled && n > 0 ) {
            char hex_line[ 56 ];   // "[x]" + 16×" XX" + "\n" + NUL = 55 chars
            for ( uint32_t i = 0; i < n; i += 16 ) {
                int pos = 0;
                pos += snprintf( hex_line + pos, sizeof( hex_line ) - (size_t)pos, "[x]" );
                for ( uint32_t j = i; j < n && j < i + 16; ++j )
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
                       ( unsigned long ) dma_rx_available() );
        }

        // 20 ms sleep — DMA buffers incoming bytes while the task is suspended.
        // At 115200 baud, 25 Hz, peak data is ~154 bytes/40 ms — well within
        // the 1024-byte ring before the next wake-up.
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
