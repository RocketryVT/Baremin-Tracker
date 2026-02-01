#include <cstdint>
#include <cstdio>
#include <span>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hal/rp2040_hal.hpp"
#include "SX127x/sx127x.hpp"

namespace board {
// TODO: update these pin assignments for your RP2350 board + RFM95W wiring.
constexpr uint8_t kSpiSck  = 18;
constexpr uint8_t kSpiMosi = 19;
constexpr uint8_t kSpiMiso = 16;
constexpr uint8_t kSpiCs   = 17;

constexpr uint8_t kLoraReset = 20;
constexpr uint8_t kLoraDio0  = 21;
}  // namespace board

static void lora_task(void* param) {
    auto* radio = static_cast<SX127x<Rp2040Hal>*>(param);

    const bool init_ok = radio->initialize();
    printf("[lora] init: %s\n", init_ok ? "ok" : "fail");

    const uint8_t payload[] = {'h', 'e', 'l', 'l', 'o'};

    for (;;) {
        const bool sent = radio->send(std::span<const uint8_t>(payload, sizeof(payload)));
        printf("[lora] send: %s\n", sent ? "ok" : "fail");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(board::kLoraReset);
    gpio_set_dir(board::kLoraReset, GPIO_OUT);
    gpio_put(board::kLoraReset, 1);

    gpio_init(board::kLoraDio0);
    gpio_set_dir(board::kLoraDio0, GPIO_IN);

    static Rp2040Hal hal({
        .spi         = spi0,
        .pin_sck     = board::kSpiSck,
        .pin_mosi    = board::kSpiMosi,
        .pin_miso    = board::kSpiMiso,
        .pin_cs      = board::kSpiCs,
        .spi_freq_hz = 1'000'000,
    });

    static SX127x<Rp2040Hal> radio(hal, {
        .pin_reset       = board::kLoraReset,
        .pin_dio0        = board::kLoraDio0,
        .frequency_hz    = 915'000'000,
        .bandwidth       = sx127x::bw::index(sx127x::bw::Value::k125),
        .spreading_factor= static_cast<uint8_t>(sx127x::sf::Value::k9),
        .coding_rate     = static_cast<uint8_t>(sx127x::cr::Value::k4_5),
        .tx_power_dbm    = 14,
    });

    xTaskCreate(lora_task, "lora", 1024, &radio, tskIDLE_PRIORITY + 1, nullptr);
    vTaskStartScheduler();

    for (;;) {
        tight_loop_contents();
    }
}
