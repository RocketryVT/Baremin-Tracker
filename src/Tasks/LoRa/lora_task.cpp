#include "lora_task.hpp"
#include "shared.hpp"

#include "mesh.hpp"

#include "hardware/gpio.h"
#include "pico/time.h"

extern "C" {
#include "lr11xx_hal.h"
#include "lr11xx_hal_context.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
}

#include <cstring>

static_assert(HAS_RADIO, "bareman_tracker requires radio support");
static_assert(Board::RadioCount > 0, "bareman_tracker requires Board::Radios[0]");

static constexpr Board::RadioInstance RADIO = Board::Radios[0];

// LR1121 RF switch control bit (DIO used to drive the TX-HP antenna path).
static constexpr uint8_t LR11XX_RFSW1_BIT = (1u << 1u);

// ===========================================================================
// LR1121 LoRa radio — active bareman telemetry modem (onboard, SPI0).
//
// Pins (boards/bareman_pcb_v2_pins.hpp, confirmed from the working bringup
// sketch lora_reference/LoRa_Tracker_Code.cpp):
//   SCK=6  MOSI=7  MISO=4  NSS=5  BUSY=0  NRESET=1  (spi0)
//
// LoRa air config is taken from the mesh RadioConfig (Board::Lora915):
// 915 MHz, SF7, BW 125 kHz, CR 4/5, sync 0x12, explicit header, CRC on,
// standard IQ — identical to the SX1276 link so the antenna tracker keeps
// decoding it unchanged.  Packet IRQ status is polled over SPI (no DIO GPIO).
// ===========================================================================
class Lr1121LoraRadio final : public SIGMA2::Radio {
public:
    const char* name() const override { return "LR1121-LoRa"; }
    const char* init_stage() const { return init_stage_; }
    int last_status() const { return last_status_; }

    int begin(const SIGMA2::RadioConfig& cfg) override
    {
        init_stage_ = "hal_init";
        lr11xx_hal_init(&ctx_);

        init_stage_ = "reset";
        if (lr11xx_hal_reset(&ctx_) != LR11XX_HAL_STATUS_OK) { last_status_ = -1; return -1; }

        init_stage_ = "wakeup";
        if (lr11xx_hal_wakeup(&ctx_) != LR11XX_HAL_STATUS_OK) { last_status_ = -1; return -2; }

        lr11xx_system_version_t ver = {};
        init_stage_ = "get_version";
        lr11xx_status_t s = lr11xx_system_get_version(&ctx_, &ver);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -3; }

        init_stage_ = "set_reg_mode";
        s = lr11xx_system_set_reg_mode(&ctx_, LR11XX_SYSTEM_REG_MODE_DCDC);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -4; }

        init_stage_ = "cfg_lfclk";
        s = lr11xx_system_cfg_lfclk(&ctx_, LR11XX_SYSTEM_LFCLK_RC, true);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -5; }

        const lr11xx_system_rfswitch_cfg_t rf_switch_cfg = {
            .enable  = LR11XX_RFSW1_BIT,
            .standby = 0x00,
            .rx      = 0x00,
            .tx      = 0x00,
            .tx_hp   = LR11XX_RFSW1_BIT,
            .tx_hf   = 0x00,
            .gnss    = 0x00,
            .wifi    = 0x00,
        };
        init_stage_ = "set_dio_as_rf_switch";
        s = lr11xx_system_set_dio_as_rf_switch(&ctx_, &rf_switch_cfg);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -6; }

        init_stage_ = "set_tcxo_mode";
        s = lr11xx_system_set_tcxo_mode(&ctx_, LR11XX_SYSTEM_TCXO_CTRL_2_7V, 164);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -7; }

        init_stage_ = "clear_errors";
        s = lr11xx_system_clear_errors(&ctx_);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -8; }

        init_stage_ = "calibrate";
        s = lr11xx_system_calibrate(&ctx_, 0x3F);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -9; }

        init_stage_ = "calibrate_image";
        s = lr11xx_system_calibrate_image_in_mhz(&ctx_, 902, 928);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -10; }

        init_stage_ = "configure_lora";
        s = configure_lora(cfg);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -11; }

        init_stage_ = "set_dio_irq_params";
        s = lr11xx_system_set_dio_irq_params(&ctx_, LR11XX_SYSTEM_IRQ_ALL_MASK, 0);
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -12; }

        init_stage_ = "start_rx";
        s = start_rx();
        if (s != LR11XX_STATUS_OK) { last_status_ = s; return -13; }

        init_stage_ = "ready";
        last_status_ = 0;
        return 0;
    }

    int transmit(const uint8_t* data, std::size_t len) override
    {
        if (!data || len == 0u || len > SIGMA2::MAX_FRAME) return -20;

        pkt_params_.pld_len_in_bytes = static_cast<uint8_t>(len);
        if (lr11xx_radio_set_lora_pkt_params(&ctx_, &pkt_params_) != LR11XX_STATUS_OK) return -21;
        if (lr11xx_regmem_write_buffer8(&ctx_, data, static_cast<uint8_t>(len)) != LR11XX_STATUS_OK) return -22;
        if (lr11xx_system_clear_irq_status(&ctx_, LR11XX_SYSTEM_IRQ_ALL_MASK) != LR11XX_STATUS_OK) return -23;

        const uint32_t toa_ms = lr11xx_radio_get_lora_time_on_air_in_ms(&pkt_params_, &mod_params_);
        if (lr11xx_radio_set_tx(&ctx_, toa_ms + 100u) != LR11XX_STATUS_OK) {
            start_rx();
            return -24;
        }

        const uint32_t start_ms = to_ms_since_boot(get_absolute_time());
        while ((to_ms_since_boot(get_absolute_time()) - start_ms) < (toa_ms + 1000u)) {
            lr11xx_system_irq_mask_t irq = 0;
            if (lr11xx_system_get_and_clear_irq_status(&ctx_, &irq) != LR11XX_STATUS_OK) {
                start_rx();
                return -25;
            }
            if ((irq & LR11XX_SYSTEM_IRQ_TX_DONE) != 0u) {
                start_rx();
                return 0;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        start_rx();
        return -26;
    }

    bool receive(SIGMA2::RadioRx& rx) override
    {
        lr11xx_system_irq_mask_t irq = 0;
        if (lr11xx_system_get_and_clear_irq_status(&ctx_, &irq) != LR11XX_STATUS_OK || irq == 0u) {
            return false;
        }
        if ((irq & LR11XX_SYSTEM_IRQ_RX_DONE) == 0u) {
            if ((irq & (LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_CRC_ERROR |
                        LR11XX_SYSTEM_IRQ_HEADER_ERROR)) != 0u) {
                start_rx();
            }
            return false;
        }

        lr11xx_radio_pkt_status_lora_t pkt_status = {};
        if (lr11xx_radio_get_lora_pkt_status(&ctx_, &pkt_status) != LR11XX_STATUS_OK) {
            start_rx();
            return false;
        }

        lr11xx_radio_rx_buffer_status_t buf_status = {};
        if (lr11xx_radio_get_rx_buffer_status(&ctx_, &buf_status) != LR11XX_STATUS_OK ||
            buf_status.pld_len_in_bytes == 0u ||
            buf_status.pld_len_in_bytes > sizeof(rx.data)) {
            start_rx();
            return false;
        }

        std::memset(&rx, 0, sizeof(rx));
        if (lr11xx_regmem_read_buffer8(&ctx_, rx.data, buf_status.buffer_start_pointer,
                                       buf_status.pld_len_in_bytes) != LR11XX_STATUS_OK) {
            start_rx();
            return false;
        }

        rx.len = buf_status.pld_len_in_bytes;
        rx.rssi_dbm = pkt_status.rssi_pkt_in_dbm;
        rx.snr_q4 = static_cast<int8_t>(pkt_status.snr_pkt_in_db * 4);
        rx.received_ms = to_ms_since_boot(get_absolute_time());
        start_rx();
        return true;
    }

private:
    lr11xx_status_t configure_lora(const SIGMA2::RadioConfig& cfg)
    {
        lr11xx_status_t s = lr11xx_radio_set_pkt_type(&ctx_, LR11XX_RADIO_PKT_TYPE_LORA);
        if (s != LR11XX_STATUS_OK) return s;

        s = lr11xx_radio_set_lora_sync_word(&ctx_, cfg.sync_word);
        if (s != LR11XX_STATUS_OK) return s;

        s = lr11xx_radio_set_rf_freq(&ctx_, static_cast<uint32_t>(cfg.freq_mhz * 1.0e6f));
        if (s != LR11XX_STATUS_OK) return s;

        const lr11xx_radio_pa_cfg_t pa_cfg = {
            .pa_sel        = LR11XX_RADIO_PA_SEL_HP,
            .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
            .pa_duty_cycle = 0x04,
            .pa_hp_sel     = 0x07,
        };
        s = lr11xx_radio_set_pa_cfg(&ctx_, &pa_cfg);
        if (s != LR11XX_STATUS_OK) return s;

        s = lr11xx_radio_set_tx_params(&ctx_, cfg.tx_dbm, LR11XX_RADIO_RAMP_208_US);
        if (s != LR11XX_STATUS_OK) return s;

        // Bareman link is fixed at SF7 / BW125 / CR4-5 (Board::Lora915).
        mod_params_.sf   = LR11XX_RADIO_LORA_SF7;
        mod_params_.bw   = LR11XX_RADIO_LORA_BW_125;
        mod_params_.cr   = LR11XX_RADIO_LORA_CR_4_5;
        mod_params_.ldro = 0;
        s = lr11xx_radio_set_lora_mod_params(&ctx_, &mod_params_);
        if (s != LR11XX_STATUS_OK) return s;

        pkt_params_.preamble_len_in_symb = cfg.preamble_len;
        pkt_params_.header_type          = LR11XX_RADIO_LORA_PKT_EXPLICIT;
        pkt_params_.pld_len_in_bytes     = SIGMA2::MAX_FRAME;
        pkt_params_.crc                  = LR11XX_RADIO_LORA_CRC_ON;
        pkt_params_.iq                   = LR11XX_RADIO_LORA_IQ_STANDARD;
        return lr11xx_radio_set_lora_pkt_params(&ctx_, &pkt_params_);
    }

    lr11xx_status_t start_rx()
    {
        lr11xx_status_t s = lr11xx_system_clear_irq_status(&ctx_, LR11XX_SYSTEM_IRQ_ALL_MASK);
        if (s != LR11XX_STATUS_OK) return s;
        // Continuous RX (max RTC-step timeout).
        return lr11xx_radio_set_rx_with_timeout_in_rtc_step(&ctx_, 0xFFFFFFu);
    }

    lr11xx_hal_context_t ctx_ = LR11XX_HAL_CONTEXT_INIT(
        spi0, 1000000u,
        Pins::LR1121_SCK, Pins::LR1121_MOSI, Pins::LR1121_MISO,
        Pins::LR1121_NSS, Pins::LR1121_BUSY, Pins::LR1121_NRESET);

    lr11xx_radio_mod_params_lora_t mod_params_ = {};
    lr11xx_radio_pkt_params_lora_t pkt_params_ = {};
    const char* init_stage_ = "not-started";
    int last_status_ = 0;
};

// ---------------------------------------------------------------------------
// External SX1276 breakout path — DISABLED (bareman now uses the onboard
// LR1121).  Kept for reference / easy revert.
// ---------------------------------------------------------------------------
#if 0
#include <RadioLib.h>
#include "PicoHal.h"

class RadioLibSx1276 final : public SIGMA2::Radio {
public:
    RadioLibSx1276()
        : hal_(spi1, Pins::LR_SCK, Pins::LR_MOSI, Pins::LR_MISO)
        , module_(&hal_, Pins::LR_NSS, Pins::LR_DIO0, Pins::LR_NRESET, RADIOLIB_NC)
        , radio_(&module_)
    {}

    const char* name() const override { return "SX1276"; }

    int begin(const SIGMA2::RadioConfig& cfg) override
    {
        ConfigLoRa_t config;
        config.frequency = cfg.freq_mhz;
        config.bandwidth = cfg.bandwidth_khz;
        config.spreadingFactor = cfg.spreading_factor;
        config.codingRate = cfg.coding_rate;
        config.syncWord = cfg.sync_word;
        config.power = cfg.tx_dbm;
        config.preambleLength = cfg.preamble_len;

        const int err = radio_.begin(config);
        if (err == RADIOLIB_ERR_NONE) {
            radio_.startReceive();
        }
        return err;
    }

    int transmit(const uint8_t* data, std::size_t len) override
    {
        const int err = radio_.transmit(const_cast<uint8_t*>(data), len);
        radio_.startReceive();
        return err;
    }

    bool receive(SIGMA2::RadioRx& rx) override
    {
        if (!gpio_get(Pins::LR_DIO0)) {
            return false;
        }

        std::memset(&rx, 0, sizeof(rx));
        const std::size_t len = radio_.getPacketLength();
        if (len == 0u || len > sizeof(rx.data)) {
            radio_.startReceive();
            return false;
        }

        const int err = radio_.readData(rx.data, len);
        if (err != RADIOLIB_ERR_NONE) {
            radio_.startReceive();
            return false;
        }

        rx.len = len;
        rx.rssi_dbm = static_cast<int16_t>(radio_.getRSSI());
        rx.snr_q4 = static_cast<int8_t>(radio_.getSNR() * 4.0f);
        radio_.startReceive();
        return true;
    }

private:
    PicoHal hal_;
    Module module_;
    SX1276 radio_;
};
#endif  // SX1276 disabled

static SIGMA2::MeshConfig mesh_config()
{
    SIGMA2::MeshConfig cfg;
    cfg.device_type = SIGMA2::DeviceType::Bareman;
    cfg.node_id = SIGMA2::NodeID::NOSE_CONE;
    cfg.default_destination = SIGMA2::NodeID::ANTENNA_TRACKER;
    cfg.radio.freq_mhz = RADIO.freq_mhz;
    cfg.radio.bandwidth_khz = static_cast<float>(Board::Lora915::BW_KHZ);
    cfg.radio.spreading_factor = Board::Lora915::SF;
    cfg.radio.coding_rate = Board::Lora915::CR;
    cfg.radio.sync_word = Board::Lora915::SYNC_WORD;
    cfg.radio.tx_dbm = Board::Lora915::TX_DBM;
    cfg.radio.preamble_len = Board::Lora915::PREAMBLE;
    return cfg;
}

static Lr1121LoraRadio s_radio;
static mesh::Mesh s_mesh(mesh_config());

static SIGMA2::MeshSnapshot build_snapshot()
{
    SIGMA2::MeshSnapshot snap;
    snap.boot_ms = to_ms_since_boot(get_absolute_time());

    GpsData gps = {};
    if (xQueuePeek(g_gps_queue, &gps, 0) == pdTRUE) {
        snap.have_gps = true;
        snap.gps.lat = gps.lat;
        snap.gps.lon = gps.lon;
        snap.gps.alt_m = gps.alt_m;
        snap.gps.satellites = gps.satellites;
        snap.gps.vel_ned_ms[0] = static_cast<float>(gps.vel_north_mms) * 0.001f;
        snap.gps.vel_ned_ms[1] = static_cast<float>(gps.vel_east_mms) * 0.001f;
        snap.gps.vel_ned_ms[2] = static_cast<float>(gps.vel_down_mms) * 0.001f;
        snap.gps.utc_ms = gps.utc_ms;
        snap.gps.utc_year = gps.utc_year;
        snap.gps.utc_month = gps.utc_month;
        snap.gps.utc_day = gps.utc_day;
        snap.gps.fix_type = gps.fix_type;
        snap.gps.flags = SIGMA2::DATA_VALID_FLAG::GPS_VALID;
    }

    SIGMA2::NavSnapshot fusion = {};
    if (xQueuePeek(g_fusion_queue, &fusion, 0) == pdTRUE) {
        snap.have_nav = true;
        snap.nav = fusion;
        if (snap.have_gps) {
            snap.nav.lat = snap.gps.lat;
            snap.nav.lon = snap.gps.lon;
            snap.nav.nav_source |= SIGMA2::TRANSMIT_PACKETS::NavSource::GPS_POS |
                                   SIGMA2::TRANSMIT_PACKETS::NavSource::GPS_VEL;
            snap.nav.flags |= SIGMA2::DATA_VALID_FLAG::GPS_VALID;
        }
    }

    return snap;
}

// -- Scheduler task ------------------------------------------------------------
// Mesh owns packet construction and transition/rate policy. This task just
// gives it the freshest application state at the current 1 Hz policy rate.
static void lora_sched_task(void*)
{
    vTaskDelay(pdMS_TO_TICKS(12000));

    log_print("[mesh] scheduler running: NavState@1Hz GpsNav@1Hz TimeSync@0.2Hzx5\n");

    TickType_t wake = xTaskGetTickCount();
    uint32_t last_dropped = 0;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));

        s_mesh.tick_1hz(build_snapshot());

        const SIGMA2::MeshTxStats& stats = s_mesh.stats();
        if (stats.dropped != last_dropped) {
            last_dropped = stats.dropped;
            log_print("[mesh] tx queue dropped=%lu queued=%lu\n",
                      static_cast<unsigned long>(stats.dropped),
                      static_cast<unsigned long>(s_mesh.queued()));
        }
    }
}

// -- Radio task ----------------------------------------------------------------
// Mesh owns the radio interface; this task clocks queued transmissions.
static void lora_radio_task(void*)
{
    for (int i = 10; i > 0; --i) {
        log_print("[mesh] radio init in %d...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    s_mesh.set_primary_radio(s_radio);

    if (!s_mesh.begin()) {
        log_print("[mesh] %s init failed - task halting\n", s_radio.name());
        while (true) vTaskDelay(portMAX_DELAY);
    }

    log_print("[mesh] %s ready - %.3f MHz SF%u BW%.0f kHz CR4/%u %d dBm\n",
              s_radio.name(),
              static_cast<double>(RADIO.freq_mhz),
              Board::Lora915::SF,
              static_cast<double>(Board::Lora915::BW_KHZ),
              Board::Lora915::CR,
              Board::Lora915::TX_DBM);
    log_print("[mesh] primary path %.3f MHz role=%s\n",
              static_cast<double>(RADIO.freq_mhz), RADIO.role);

    uint32_t last_ok_report = 0;
    uint32_t last_err_report = 0;

    for (;;) {
        (void) s_mesh.poll_receive(to_ms_since_boot(get_absolute_time()));

        int radio_state = 0;
        if (!s_mesh.transmit_one(&radio_state)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        const SIGMA2::MeshTxStats& stats = s_mesh.stats();
        if (radio_state != 0 && stats.err != last_err_report) {
            last_err_report = stats.err;
            log_print("[mesh] tx failed code=%d ok=%lu err=%lu drop=%lu\n",
                      radio_state,
                      static_cast<unsigned long>(stats.ok),
                      static_cast<unsigned long>(stats.err),
                      static_cast<unsigned long>(stats.dropped));
        } else if (stats.ok != last_ok_report && (stats.ok % 10u) == 0u) {
            last_ok_report = stats.ok;
            log_print("[mesh] tx ok=%lu err=%lu queued=%lu drop=%lu\n",
                      static_cast<unsigned long>(stats.ok),
                      static_cast<unsigned long>(stats.err),
                      static_cast<unsigned long>(s_mesh.queued()),
                      static_cast<unsigned long>(stats.dropped));
        }
    }
}

// -- Static task storage -------------------------------------------------------
static StaticTask_t s_sched_tcb;
static StackType_t  s_sched_stack[1024];

static StaticTask_t s_radio_tcb;
static StackType_t  s_radio_stack[2048];

void lora_task_init()
{
    TaskHandle_t h;

    h = xTaskCreateStatic(lora_sched_task, "mesh_sched", 1024,
                          nullptr, tskIDLE_PRIORITY + 3,
                          s_sched_stack, &s_sched_tcb);
    configASSERT(h);
    vTaskCoreAffinitySet(h, (1u << 0));

    h = xTaskCreateStatic(lora_radio_task, "mesh_radio", 2048,
                          nullptr, tskIDLE_PRIORITY + 4,
                          s_radio_stack, &s_radio_tcb);
    configASSERT(h);
    vTaskCoreAffinitySet(h, (1u << 0));
}
