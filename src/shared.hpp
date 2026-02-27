#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// ── Log queue ─────────────────────────────────────────────────────────────────
// Tasks call log_print() instead of printf() directly to avoid stdio contention
// under FreeRTOS SMP.  The USB task is the sole consumer and the only code that
// calls printf().
//
// log_print() is non-blocking: if the queue is full, the message is dropped.
// Increase LOG_QUEUE_DEPTH if drops are observed.
// IMPORTANT: do NOT call log_print() from ISR context.
struct LogMessage {
    char buf[ 256 ];
};

#define LOG_QUEUE_DEPTH  16
extern QueueHandle_t g_log_queue;

void log_print( const char* fmt, ... ) __attribute__(( format( printf, 1, 2 ) ));

// ── GPS fix data ──────────────────────────────────────────────────────────────
// Written by the GPS task; read by the LoRa task.
struct GpsData {
    double   lat;
    double   lon;
    double   alt_m;
    uint8_t  satellites;
};

// Depth-1 overwrite queue — LoRa task always reads the freshest fix.
#define GPS_QUEUE_DEPTH  1
extern QueueHandle_t g_gps_queue;

// ── Pin assignments ───────────────────────────────────────────────────────────
namespace Pins {
    // LR1121 — SPI0
    static constexpr uint LR_SCK     = 6;
    static constexpr uint LR_MOSI    = 4;
    static constexpr uint LR_MISO    = 7;
    static constexpr uint LR_NSS     = 5;
    static constexpr uint LR_BUSY    = 0;
    static constexpr uint LR_NRESET  = 1;

    // Status LED
    static constexpr uint STATUS     = 12;

    // Debug UART — TX/RX
    static constexpr uint DBG_TX     = 13;
    static constexpr uint DBG_RX     = 14;

    // GPS — UART0
    static constexpr uint GPS_UART_TX = 16;   // RP2350 TX → GPS RX (not driven by us)
    static constexpr uint GPS_UART_RX = 17;   // GPS TX → RP2350 RX (NMEA input)
}

// ── LoRa radio parameters (must match ground station receiver) ────────────────
namespace LoRaCfg {
    static constexpr uint32_t FREQ_HZ   = 915'000'000;
    static constexpr uint8_t  SF        = 7;      // Spreading Factor 7
    static constexpr uint8_t  BW        = 125;    // 125 kHz
    static constexpr uint8_t  CR        = 5;      // 4/5
    static constexpr uint8_t  SYNC_WORD = 0x12;   // Private network (matches SX1276 GS)
    static constexpr int8_t   TX_DBM    = 22;     // dBm (HPA, VBAT supply)
    static constexpr uint16_t PREAMBLE  = 8;      // symbols

    // Transmit interval
    static constexpr uint32_t TX_PERIOD_MS = 1000;
}
