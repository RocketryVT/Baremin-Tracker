#include "imu_task.hpp"
#include "shared.hpp"

#include "icm40609d/ICM40609D.hpp"
#include "Tasks/I2C/i2c_task.hpp"

// ---------------------------------------------------------------------------
// I2C bus + GPIO assignment
// ICM-40609-D shares i2c0 with the MS5607 barometer.
// i2c0 is already initialised by baro_task_init() — do NOT re-init here.
// ---------------------------------------------------------------------------
static constexpr uint IMU_SDA_PIN = Pins::BARO_SDA;
static constexpr uint IMU_SCL_PIN = Pins::BARO_SCL;
// AP_AD0 is strapped high (pin 9 -> /VREG_3V3) on the bareman V2 PCB, so the
// I2C address LSB is 1 -> 0x69 (not the 0x68 power-on default).
// NOTE: the V2 board does NOT DC-power the IMU — VDD/VDDIO reach the 3V3 rail
// only through series decoupling caps (no direct connection). The part cannot
// respond until that is bodged on the hardware. See board_profile.hpp.
static constexpr uint8_t IMU_ADDR = 0x69u;

// ---------------------------------------------------------------------------
// Transport shims — adapt the i2c task queue API to the icm40609d::Transport
// ---------------------------------------------------------------------------

static bool imu_write_reg(void* /*ctx*/, uint8_t reg, uint8_t val)
{
    I2cRequest req;
    uint8_t buf[2] = { reg, val };
    i2c_req_write(&req, IMU_ADDR, buf, 2u);
    return i2c_req_submit_wait(&req);
}

static bool imu_read_regs(void* /*ctx*/, uint8_t reg, uint8_t* buf, size_t len)
{
    I2cRequest req;
    i2c_req_write_read(&req, IMU_ADDR, &reg, 1u, buf, static_cast<uint8_t>(len));
    return i2c_req_submit_wait(&req);
}

static void delay_ms_shim(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ---------------------------------------------------------------------------
// Driver instance — ctx unused (address baked into shims above)
// ---------------------------------------------------------------------------

static icm40609d::Device s_imu({
    .ctx       = nullptr,
    .write_reg = imu_write_reg,
    .read_regs = imu_read_regs,
});

// ---------------------------------------------------------------------------
// Task storage
// ---------------------------------------------------------------------------

static StaticTask_t s_imu_tcb;
static StackType_t  s_imu_stack[512];

// ---------------------------------------------------------------------------
// IMU task — 100 Hz
// ---------------------------------------------------------------------------

static void imu_task(void*)
{
    icm40609d::Config cfg;
    cfg.gyro_range    = icm40609d::GyroRange::dps_2000;
    cfg.accel_range   = icm40609d::AccelRange::g16;
    cfg.gyro_odr      = icm40609d::ODR::hz_100;
    cfg.accel_odr     = icm40609d::ODR::hz_100;
    cfg.gyro_aaf_hz   = 450u;
    cfg.accel_aaf_hz  = 450u;
    cfg.gyro_ui_order = icm40609d::UiFiltOrder::order_2nd;
    cfg.accel_ui_order= icm40609d::UiFiltOrder::order_2nd;
    cfg.temp_filt_bw  = icm40609d::TempFiltBw::hz_4000;
    cfg.gyro_notch_en = true;
    cfg.gyro_nf_bw    = icm40609d::GyroNfBw::hz_162;
    cfg.gyro_notch_khz= 1.0f;
    cfg.gyro_hpf_en   = true;
    cfg.gyro_hpf_bw   = 1u;
    cfg.gyro_hpf_order= icm40609d::HpfOrder::order_1st;
    cfg.delay_ms      = delay_ms_shim;

    if (!s_imu.initialize(cfg)) {
        log_print("[imu] ICM-40609-D init failed — check I2C wiring/address\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(5000));
    }

    log_print("[imu] ICM-40609-D ready  sda=%u scl=%u  ±2000dps ±16g 100Hz\n",
              IMU_SDA_PIN, IMU_SCL_PIN);

    for (;;) {
        icm40609d::Sample s;
        if (s_imu.read_sample(s)) {
            ImuData data{
                .accel_x_mss = s.accel_x,
                .accel_y_mss = s.accel_y,
                .accel_z_mss = s.accel_z,
                .gyro_x_dps  = s.gyro_x,
                .gyro_y_dps  = s.gyro_y,
                .gyro_z_dps  = s.gyro_z,
                .temp_c      = s.temp_c,
            };
            xQueueOverwrite(g_imu_queue, &data);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void imu_task_init()
{
    // i2c0 on GPIO 20/21 is already initialised by baro_task_init().
    // i2c_task_init() must be called before this (i2c task queue must exist).

    TaskHandle_t h = xTaskCreateStatic(
        imu_task, "imu", 512u,
        nullptr, tskIDLE_PRIORITY + 2u,
        s_imu_stack, &s_imu_tcb);
    configASSERT(h);
    vTaskCoreAffinitySet(h, 1u << 0u);
}
