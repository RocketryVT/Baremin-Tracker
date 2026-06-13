#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "boards/board.hpp"
#include "mesh.hpp"
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

// -- Fused navigation data -----------------------------------------------------
// Written by fusion_task; read by mesh/lora. Queue items are SIGMA2::NavSnapshot.
#define FUSION_QUEUE_DEPTH  1
extern QueueHandle_t g_fusion_queue;

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
