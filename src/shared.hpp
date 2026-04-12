#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SIGMA.hpp"   // packet / record definitions (include path added via CMakeLists)

// -- Log queue -----------------------------------------------------------------
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

#define LOG_QUEUE_DEPTH  64
extern QueueHandle_t g_log_queue;

void log_print( const char* fmt, ... ) __attribute__(( format( printf, 1, 2 ) ));

// -- GPS fix data --------------------------------------------------------------
// Written by the GPS task; read by the LoRa task.
struct GpsData {
    double   lat;
    double   lon;
    double   alt_m;
    uint8_t  satellites;
    float    speed_mps;       // ground speed, m/s
    float    course_deg;      // course over ground, degrees true
    uint32_t utc_ms;          // ms since midnight UTC
    uint16_t utc_year;
    uint8_t  utc_month;
    uint8_t  utc_day;
    int32_t  vel_north_mms;   // NED north velocity, mm/s
    int32_t  vel_east_mms;    // NED east velocity,  mm/s
    int32_t  vel_down_mms;    // NED down velocity,  mm/s (+ve = descending)
    uint8_t  fix_type;        // 0=none 2=2D 3=3D 4=GNSS+DR (UBX only)
    float    hdop;            // horizontal DOP (NAV-DOP; 0 if not received)
    float    vdop;            // vertical DOP   (NAV-DOP; 0 if not received)
    uint8_t  best_cno;        // best C/N0 in dBHz  (NAV-SAT; 0 if not received)
    uint8_t  num_sv_used;     // SVs with svUsed flag (NAV-SAT; 0 if not received)
};

// Depth-1 overwrite queue — LoRa task always reads the freshest fix.
#define GPS_QUEUE_DEPTH  1
extern QueueHandle_t g_gps_queue;

// -- Barometer data ------------------------------------------------------------
// Written by baro_reader_task; read by LoRa task.
// Units match MS5607 driver output (ALTITUDE_SCALE=10):
//   pressure_pa      — Pa        (divide by 100 for hPa / mbar)
//   temperature_cdeg — 1/100 °C  (e.g. 2134 = 21.34 °C)
//   altitude_dm      — 1/10 m    (divide by 10 for metres)
struct BaroData {
    int32_t pressure_pa;
    int32_t temperature_cdeg;
    int32_t altitude_dm;     ///< decimetres (ALTITUDE_SCALE = 10)
};

#define BARO_QUEUE_DEPTH  1
extern QueueHandle_t g_baro_queue;

// -- Flash logger queue --------------------------------------------------------
// baro_reader_task pushes one SigmaStorageFullRecord per sample.
// logger_task drains the queue and commits records to flash via pico_logger.
// Depth-32 absorbs short bursts at 50 Hz; records dropped if logger falls behind.
#define LOGGER_QUEUE_DEPTH  32
extern QueueHandle_t g_logger_queue;

// -- Global flight state -------------------------------------------------------
// Written by baro_reader_task; read by logger_task and lora_task.
// volatile ensures cross-core visibility without a mutex (single writer).
extern volatile FlightState g_flight_state;

// -- NMEA raw stream flag -------------------------------------------------------
// Set by USB console ("nmea on/off"). Read by GPS task to gate raw sentence output.
extern volatile bool g_nmea_raw_enabled;

// -- UBX hex dump flag ---------------------------------------------------------
// Set by USB console ("hex on/off"). Dumps every raw UART byte as hex.
extern volatile bool g_ubx_hex_enabled;

// -- i2c0 bus serialisation ----------------------------------------------------
// MS5607 barometer and ICM-40609-D IMU share i2c0.
// All i2c0 transfers are routed through the i2c task queue (Tasks/I2C/i2c_task).
// Do NOT call i2c_write_blocking / i2c_read_blocking directly on i2c0.

// -- Pin assignments -----------------------------------------------------------
namespace Pins {
    // SX1276 — SPI1
    static constexpr uint LR_SCK     = 26;
    static constexpr uint LR_MOSI    = 27;
    static constexpr uint LR_MISO    = 28;
    static constexpr uint LR_NSS     = 29;
    static constexpr uint LR_DIO0    = 22;  // TxDone / RxDone interrupt
    static constexpr uint LR_NRESET  = 23;

    // MS5607 barometer — I2C0
    static constexpr uint BARO_SDA   = 20;
    static constexpr uint BARO_SCL   = 21;

    // Status LED
    static constexpr uint STATUS     = 12;

    // Debug UART — TX/RX
    static constexpr uint DBG_TX     = 13;
    static constexpr uint DBG_RX     = 14;

    // GPS — UART0
    static constexpr uint GPS_UART_TX = 16;   // RP2350 TX  GPS RX
    static constexpr uint GPS_UART_RX = 17;   // GPS TX  RP2350 RX (NMEA input)
}

// -- IMU data ------------------------------------------------------------------
// Written by imu_task; read by any task needing inertial data.
// Units: accel in m/s², gyro in °/s, temp in °C.
struct ImuData {
    float accel_x_mss;
    float accel_y_mss;
    float accel_z_mss;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
};

// Depth-1 overwrite queue — always holds the freshest sample.
#define IMU_QUEUE_DEPTH  1
extern QueueHandle_t g_imu_queue;

// -- LoRa TX frame queue -------------------------------------------------------
// Pre-serialized SIGMA frames queued by the scheduler and drained by the radio.
// Depth of 16 absorbs a burst of 5 TIME_SYNC frames plus normal 10 Hz traffic.
struct TxFrame {
    uint8_t  buf[ SIGMA::MAX_FRAME ];
    uint16_t len;
};

#define TX_QUEUE_DEPTH  16
extern QueueHandle_t g_tx_queue;

// -- LoRa radio parameters (must match ground station receiver) ----------------
namespace LoRaCfg {
    static constexpr uint32_t FREQ_HZ   = 915'000'000;
    static constexpr uint8_t  SF        = 7;      // Spreading Factor 7
    static constexpr uint8_t  BW        = 125;    // 125 kHz
    static constexpr uint8_t  CR        = 7;      // 4/5
    static constexpr uint8_t  SYNC_WORD = 0x12;   // Private network (matches SX1276 GS)
    static constexpr int8_t   TX_DBM    = 20;     // dBm (RFM95W PA_BOOST max)
    static constexpr uint16_t PREAMBLE  = 8;      // symbols

    // Transmit interval
    static constexpr uint32_t TX_PERIOD_MS = 1000;
}
