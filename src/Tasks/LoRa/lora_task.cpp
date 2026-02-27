#include "lora_task.hpp"
#include "shared.hpp"

#include "lr11xx_hal_context.h"
#include "lr11xx_hal.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
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

// Payload buffer — max 255 bytes; JSON fits easily within 128.
#define MAX_PAYLOAD 128

// ── Radio init ────────────────────────────────────────────────────────────────
static bool radio_init( void )
{
    lr11xx_hal_init( &s_radio );

    if ( lr11xx_hal_reset( &s_radio ) != LR11XX_HAL_STATUS_OK ) {
        log_print( "[lora] reset failed\n" );
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

// ── Task ─────────────────────────────────────────────────────────────────────
static void lora_task( void* param )
{
    ( void ) param;

    if ( !radio_init() ) {
        log_print( "[lora] init failed — task halting\n" );
        while ( true ) vTaskDelay( portMAX_DELAY );
    }

    char       json[ MAX_PAYLOAD ];
    GpsData    gps;
    TickType_t last_tx = 0;

    for ( ;; ) {
        TickType_t now = xTaskGetTickCount();

        if ( ( now - last_tx ) >= pdMS_TO_TICKS( LoRaCfg::TX_PERIOD_MS ) ) {
            last_tx = now;

            if ( xQueuePeek( g_gps_queue, &gps, 0 ) == pdTRUE ) {
                int len = snprintf( json, sizeof( json ),
                                    "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"sats\":%u}",
                                    gps.lat, gps.lon, gps.alt_m, gps.satellites );

                if ( len > 0 && len < ( int ) sizeof( json ) ) {
                    bool ok = radio_transmit(
                        reinterpret_cast<const uint8_t*>( json ),
                        static_cast<uint8_t>( len ) );

                    log_print( "[lora] tx %s  (%d B)\n", ok ? "ok" : "FAIL", len );
                }
            } else {
                log_print( "[lora] waiting for GPS fix...\n" );
            }
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

static StaticTask_t s_lora_tcb;
static StackType_t  s_lora_stack[ 2048 ];

void lora_task_init()
{
    configASSERT( xTaskCreateStatic( lora_task, "lora", 2048,
                                      NULL, tskIDLE_PRIORITY + 2,
                                      s_lora_stack, &s_lora_tcb ) );
}
