#pragma once

// board_profile.hpp — operational device profile for bareman_tracker.
//
// Bareman hardware:
//   Baro: MS5607
//   IMU:  ICM-40609-D footprint, currently not usable
//   GPS:  offboard u-blox NEO-M9N module
//   LoRa: RFM95W breakout, Semtech SX1276 silicon
//
// Keep this active profile to devices that are working in flight firmware today.
// Do not declare APP_HAS_IMU until the physical IMU path is fixed.

#define APP_HAS_GPS     1
#define APP_HAS_BARO    1
#define APP_HAS_RADIO   1
#define APP_HAS_SX1276  1

namespace Board {

inline constexpr RadioInstance Radios[] = {
    { RadioModel::SX1276, Bus::SPI1, Pins::LR_NSS, 915.0f, "915-lora" },
};
inline constexpr int RadioCount = static_cast<int>(std::size(Radios));

inline constexpr GpsInstance Gpses[] = {
    { GpsModel::UbloxM9N, Bus::UART0, /*baud*/115200, /*nav_hz*/10, "primary" },
};
inline constexpr int GpsCount = static_cast<int>(std::size(Gpses));

inline constexpr BaroInstance Baros[] = {
    { BaroModel::MS5607, Bus::I2C0, /*addr*/0x77, /*odr_hz*/50, "primary" },
};
inline constexpr int BaroCount = static_cast<int>(std::size(Baros));

namespace Lora915 {
    inline constexpr uint8_t  SF        = 7;
    inline constexpr uint8_t  BW_KHZ    = 125;
    inline constexpr uint8_t  CR        = 5;      // RadioLib ConfigLoRa_t coding rate value
    inline constexpr uint8_t  SYNC_WORD = 0x12;   // private network
    inline constexpr int8_t   TX_DBM    = 20;
    inline constexpr uint16_t PREAMBLE  = 8;
}

} // namespace Board
