#include "lora_task.hpp"
#include "shared.hpp"

#include "mesh.hpp"

#include <RadioLib.h>
#include "PicoHal.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <cstring>

static_assert(HAS_RADIO, "bareman_tracker requires radio support");
static_assert(HAS_SX1276, "bareman_tracker requires an SX1276 radio");
static_assert(Board::RadioCount > 0, "bareman_tracker requires Board::Radios[0]");
static_assert(Board::Radios[0].model == Board::RadioModel::SX1276,
              "bareman_tracker currently supports SX1276 as Board::Radios[0]");
static_assert(Board::Radios[0].bus == Board::Bus::SPI1,
              "bareman_tracker SX1276 is expected on SPI1");
static_assert(Board::Radios[0].freq_mhz >= Board::spec_of(Board::Radios[0].model).freq_min_mhz &&
              Board::Radios[0].freq_mhz <= Board::spec_of(Board::Radios[0].model).freq_max_mhz,
              "SX1276 operating frequency outside device spec");

static constexpr Board::RadioInstance RADIO = Board::Radios[0];

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

static RadioLibSx1276 s_radio;
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
        if (radio_state != RADIOLIB_ERR_NONE && stats.err != last_err_report) {
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
