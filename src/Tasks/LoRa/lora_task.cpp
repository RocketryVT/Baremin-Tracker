#include "lora_task.hpp"
#include "shared.hpp"

#include "lr11xx_hal_context.h"
#include "lr11xx_hal.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

// ── Radio context — board pin mapping for the Bareman Tracker ─────────────────
// LR11XX_HAL_CONTEXT_INIT zero-initialises the internal mutex fields;
// lr11xx_hal_init() creates the FreeRTOS mutex at runtime.
static lr11xx_hal_context_t s_radio = LR11XX_HAL_CONTEXT_INIT(
    spi0,          // SPI bus
    8'000'000u,    // 8 MHz clock (LR11XX max: 16 MHz)
    Pins::LR_SCK,
    Pins::LR_MOSI,
    Pins::LR_MISO,
    Pins::LR_NSS,
    Pins::LR_BUSY,
    Pins::LR_NRESET
);

// ── Radio init ────────────────────────────────────────────────────────────────
static bool radio_init( void )
{
    log_print( "[lora] pre-init  BUSY=%d  NRESET=%d\n",
               gpio_get( Pins::LR_BUSY ), gpio_get( Pins::LR_NRESET ) );

    lr11xx_hal_init( &s_radio );

    log_print( "[lora] post-init BUSY=%d\n", gpio_get( Pins::LR_BUSY ) );

    if ( lr11xx_hal_reset( &s_radio ) != LR11XX_HAL_STATUS_OK ) {
        log_print( "[lora] reset failed — BUSY=%d\n", gpio_get( Pins::LR_BUSY ) );
        return false;
    }

    log_print( "[lora] chip ready  BUSY=%d\n", gpio_get( Pins::LR_BUSY ) );

    // Verify chip is alive and print firmware version
    lr11xx_system_version_t ver = {};
    if ( lr11xx_system_get_version( &s_radio, &ver ) == LR11XX_STATUS_OK ) {
        log_print( "[lora] LR11XX hw=0x%02X  type=0x%02X  fw=%u.%u\n",
                   ver.hw, ver.type, ver.fw >> 8, ver.fw & 0xFF );
    } else {
        log_print( "[lora] get_version failed — SPI may be broken\n" );
        return false;
    }

    // Put chip in standby (RC oscillator) before configuring
    if ( lr11xx_system_set_standby( &s_radio, LR11XX_SYSTEM_STANDBY_CFG_RC )
         != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_standby failed\n" );
        return false;
    }

    // RF switch — tells the chip how to drive RFSW0/RFSW1 (DIO5/DIO6) in each
    // mode so the external RF switch routes the signal correctly.
    // ⚠ Bitmask values must match your PCB schematic.
    // Typical Semtech EVK mapping for HP PA (sub-GHz):
    //   standby -> both low   rx   -> RFSW0 high
    //   tx_hp   -> RFSW1 high (HP PA path, matches pa_sel=HP above)
    //   tx      -> both high  (LP PA path — unused here but set for completeness)
    const lr11xx_system_rfswitch_cfg_t rf_sw = {
        .enable  = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
        .standby = 0,
        .rx      = LR11XX_SYSTEM_RFSW0_HIGH,
        .tx      = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
        .tx_hp   = LR11XX_SYSTEM_RFSW1_HIGH,
        .tx_hf   = 0,
        .gnss    = 0,
        .wifi    = 0,
    };
    if ( lr11xx_system_set_dio_as_rf_switch( &s_radio, &rf_sw )
         != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_dio_as_rf_switch failed\n" );
        return false;
    }

    // LoRa packet type
    if ( lr11xx_radio_set_pkt_type( &s_radio, LR11XX_RADIO_PKT_TYPE_LORA )
         != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_pkt_type failed\n" );
        return false;
    }

    // RF frequency
    if ( lr11xx_radio_set_rf_freq( &s_radio, LoRaCfg::FREQ_HZ )
         != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_rf_freq failed\n" );
        return false;
    }

    // PA: High-power PA, VBAT supply — for external sub-GHz antenna.
    // Adjust pa_duty_cycle / pa_hp_sel for your specific RF front-end.
    lr11xx_radio_pa_cfg_t pa_cfg = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_duty_cycle = 0x04,
        .pa_hp_sel     = 0x07,
    };
    if ( lr11xx_radio_set_pa_cfg( &s_radio, &pa_cfg ) != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_pa_cfg failed\n" );
        return false;
    }

    // TX power and ramp time
    if ( lr11xx_radio_set_tx_params( &s_radio,
                                      LoRaCfg::TX_DBM,
                                      LR11XX_RADIO_RAMP_48_US )
         != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_tx_params failed\n" );
        return false;
    }

    // LoRa modulation — SF7, BW 125 kHz, CR 4/5
    // LDRO off: symbol duration ~1 ms, well below the 16 ms threshold
    lr11xx_radio_mod_params_lora_t mod = {
        .sf   = LR11XX_RADIO_LORA_SF7,
        .bw   = LR11XX_RADIO_LORA_BW_125,
        .cr   = LR11XX_RADIO_LORA_CR_4_5,
        .ldro = 0,
    };
    if ( lr11xx_radio_set_lora_mod_params( &s_radio, &mod ) != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_lora_mod_params failed\n" );
        return false;
    }

    // Sync word 0x12 = private network (matches RadioLib SX1276/SX126x default)
    if ( lr11xx_radio_set_lora_sync_word( &s_radio, LoRaCfg::SYNC_WORD )
         != LR11XX_STATUS_OK ) {
        log_print( "[lora] set_lora_sync_word failed\n" );
        return false;
    }

    log_print( "[lora] LR1121 ready — %.3f MHz  SF%u  BW%u kHz  %d dBm\n",
            LoRaCfg::FREQ_HZ / 1e6f,
            LoRaCfg::SF, LoRaCfg::BW, LoRaCfg::TX_DBM );
    return true;
}

// ── Transmit one packet ───────────────────────────────────────────────────────
static bool radio_transmit( const uint8_t* payload, uint8_t len )
{
    // Packet parameters — update payload length before each TX
    lr11xx_radio_pkt_params_lora_t pkt = {
        .preamble_len_in_symb = LoRaCfg::PREAMBLE,
        .header_type          = LR11XX_RADIO_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes     = len,
        .crc                  = LR11XX_RADIO_LORA_CRC_ON,
        .iq                   = LR11XX_RADIO_LORA_IQ_STANDARD,
    };

    if ( lr11xx_radio_set_lora_pkt_params( &s_radio, &pkt ) != LR11XX_STATUS_OK ) {
        return false;
    }

    // Load payload into the radio TX buffer
    if ( lr11xx_regmem_write_buffer8( &s_radio, payload, len ) != LR11XX_STATUS_OK ) {
        return false;
    }

    // Start single TX — timeout 0: no radio timeout, returns to standby after TX
    if ( lr11xx_radio_set_tx( &s_radio, 0 ) != LR11XX_STATUS_OK ) {
        return false;
    }

    // Wait for BUSY to go low (TX done, back in standby).
    // lr11xx_hal_wait_busy yields with vTaskDelay(1) rather than spinning.
    return lr11xx_hal_wait_busy( &s_radio ) == LR11XX_HAL_STATUS_OK;
}

// ── Debug helpers ─────────────────────────────────────────────────────────────
static void log_hex( const char* prefix, const uint8_t* data, size_t len )
{
    log_print( "%s (%u bytes):\n", prefix, ( unsigned ) len );
    for ( size_t i = 0; i < len; ++i ) {
        if ( i % 16 == 0 ) log_print( "  %04X: ", ( unsigned ) i );
        log_print( "%02X ", data[i] );
        if ( i % 16 == 15 || i == len - 1 ) log_print( "\n" );
    }
}

static void log_sigma_lora( const SigmaLoRaData& d )
{
    log_print( "[lora] struct: boot_ms=%lu  state=%u  sats=%u  flags=0x%02X\n",
               ( unsigned long ) d.boot_ms, ( unsigned ) d.state,
               ( unsigned ) d.satellites,   ( unsigned ) d.flags );
    log_print( "  lat=%.7f  lon=%.7f  alt_gps=%.1f m  alt_baro=%.1f m  speed=%.2f m/s\n",
               d.lat, d.lon,
               ( double ) d.alt_gps_m, ( double ) d.alt_baro_m, ( double ) d.speed_ms );
    log_print( "  q=[%.5f  %.5f  %.5f  %.5f]\n",
               ( double ) d.q[0], ( double ) d.q[1],
               ( double ) d.q[2], ( double ) d.q[3] );
}

// ── Task ─────────────────────────────────────────────────────────────────────
void lora_task( void* param )
{
    ( void ) param;

    // For 10 seconds print "lora init..." every second, then try to init the radio.
    for ( int i = 0; i < 10; i++ ) {
        log_print( "[lora] init in %d seconds...\n", 10 - i );
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }

    if ( !radio_init() ) {
        log_print( "[lora] init failed — task halting\n" );
        while ( true ) vTaskDelay( portMAX_DELAY );
    }

    // SIGMA LoRa frame — 36-byte payload + 9-byte framing = 45 bytes on-air.
    uint8_t frame[ SIGMA_MAX_FRAME ];

    for ( ;; ) {
        SigmaLoRaData d;
        d.boot_ms = to_ms_since_boot( get_absolute_time() );
        d.state   = g_flight_state;

        // Priority 1: GPS lat/lon/alt — always prefer GPS position when available.
        GpsData gps;
        if ( xQueuePeek( g_gps_queue, &gps, 0 ) == pdTRUE ) {
            d.lat        = gps.lat;
            d.lon        = gps.lon;
            d.alt_gps_m  = static_cast<float>( gps.alt_m );
            d.satellites = gps.satellites;
            d.flags     |= SIGMA_FLAG_GPS_VALID;
        }

        // Baro altitude and pressure.
        BaroData baro;
        if ( xQueuePeek( g_baro_queue, &baro, 0 ) == pdTRUE ) {
            d.alt_baro_m = static_cast<float>( baro.altitude_dm ) * 0.1f;
            d.flags     |= SIGMA_FLAG_BARO_VALID;
        }

        log_sigma_lora( d );

        size_t n = d.serialize( frame, sizeof( frame ) );
        if ( n > 0 ) {
            log_hex( "[lora] raw", frame, n );
            bool ok = radio_transmit( frame, static_cast<uint8_t>( n ) );
            log_print( "[lora] tx %u bytes  gps=%s baro=%s  %s\n",
                       (unsigned) n,
                       ( d.flags & SIGMA_FLAG_GPS_VALID )  ? "ok" : "--",
                       ( d.flags & SIGMA_FLAG_BARO_VALID ) ? "ok" : "--",
                       ok ? "ok" : "FAIL" );
        }

        vTaskDelay( pdMS_TO_TICKS( LoRaCfg::TX_PERIOD_MS ) );
    }
}

static StaticTask_t s_lora_tcb;
static StackType_t  s_lora_stack[ 2048 ];

void lora_task_init()
{
    TaskHandle_t lora_handle = xTaskCreateStatic( lora_task, "lora", 2048,
                                NULL, tskIDLE_PRIORITY + 4,
                                s_lora_stack, &s_lora_tcb );
    configASSERT( lora_handle );
    vTaskCoreAffinitySet( lora_handle, ( 1u << 0 ) );
}
