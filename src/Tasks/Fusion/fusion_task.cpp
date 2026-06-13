#include "fusion_task.hpp"
#include "shared.hpp"

// Baro/GPS fusion task. This intentionally does not read the IMU/mag queues
// while those physical devices are unavailable.

static StaticTask_t s_fusion_tcb;
static StackType_t  s_fusion_stack[512];

static SIGMA2::FLIGHT_STATE to_sigma2_state(FlightState state)
{
    switch (state) {
    case FlightState::GROUND_IDLE:
    case FlightState::ARMED:
        return SIGMA2::FLIGHT_STATE::PAD;
    case FlightState::POWERED_ASCENT:
        return SIGMA2::FLIGHT_STATE::BOOST;
    case FlightState::COAST_ASCENT:
        return SIGMA2::FLIGHT_STATE::COAST;
    case FlightState::APOGEE:
        return SIGMA2::FLIGHT_STATE::APOGEE;
    case FlightState::DESCENT_DROGUE:
    case FlightState::DESCENT_MAIN:
        return SIGMA2::FLIGHT_STATE::DESCENT;
    case FlightState::LANDED:
        return SIGMA2::FLIGHT_STATE::LANDED;
    case FlightState::FAULT:
    default:
        return SIGMA2::FLIGHT_STATE::UNKOWN;
    }
}

static bool gps_velocity_valid(const GpsData& gps)
{
    return gps.fix_type >= 2u;
}

static float mms_to_mps(int32_t value)
{
    return static_cast<float>(value) * 0.001f;
}

static void fusion_task(void*)
{
    bool have_prev = false;
    float prev_alt_m = 0.0f;
    TickType_t prev_tick = xTaskGetTickCount();

    for (;;) {
        BaroData baro = {};
        if (xQueuePeek(g_baro_queue, &baro, pdMS_TO_TICKS(100)) == pdTRUE) {
            const TickType_t now_tick = xTaskGetTickCount();
            const float alt_m = static_cast<float>(baro.altitude_dm) * 0.1f;
            float vel_down_mps = 0.0f;
            float vel_north_mps = 0.0f;
            float vel_east_mps = 0.0f;
            uint8_t flags = SIGMA2::DATA_VALID_FLAG::BARO_VALID;

            GpsData gps = {};
            const bool have_gps_velocity =
                xQueuePeek(g_gps_queue, &gps, 0) == pdTRUE &&
                gps_velocity_valid(gps);
            if (have_gps_velocity) {
                vel_north_mps = mms_to_mps(gps.vel_north_mms);
                vel_east_mps = mms_to_mps(gps.vel_east_mms);
                vel_down_mps = mms_to_mps(gps.vel_down_mms);
                flags |= SIGMA2::DATA_VALID_FLAG::GPS_VALID;
            }

            if (have_prev) {
                const TickType_t dt_ticks = now_tick - prev_tick;
                const float dt_s = static_cast<float>(dt_ticks) /
                                   static_cast<float>(configTICK_RATE_HZ);
                if (dt_s > 0.0f) {
                    // NED down is positive while descending.
                    vel_down_mps = -(alt_m - prev_alt_m) / dt_s;
                }
            }

            prev_alt_m = alt_m;
            prev_tick = now_tick;
            have_prev = true;

            SIGMA2::NavSnapshot fusion = {};
            fusion.alt_baro_m = alt_m;
            fusion.alt_fused_m = alt_m;
            fusion.vel_ned_ms[0] = vel_north_mps;
            fusion.vel_ned_ms[1] = vel_east_mps;
            fusion.vel_ned_ms[2] = vel_down_mps;
            fusion.q[0] = 1.0f;
            fusion.q[1] = 0.0f;
            fusion.q[2] = 0.0f;
            fusion.q[3] = 0.0f;
            fusion.frame = SIGMA2::CoordinateFrame::NED;
            fusion.nav_source = SIGMA2::TRANSMIT_PACKETS::NavSource::BARO_ALT |
                                SIGMA2::TRANSMIT_PACKETS::NavSource::BARO_VEL;
            fusion.flags = flags;
            fusion.state = to_sigma2_state(g_flight_state);
            if (have_gps_velocity) {
                fusion.lat = gps.lat;
                fusion.lon = gps.lon;
                fusion.nav_source |= SIGMA2::TRANSMIT_PACKETS::NavSource::GPS_POS |
                                     SIGMA2::TRANSMIT_PACKETS::NavSource::GPS_VEL;
            }
            xQueueOverwrite(g_fusion_queue, &fusion);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void fusion_task_init()
{
    TaskHandle_t h = xTaskCreateStatic(
        fusion_task, "fusion", 512,
        nullptr, tskIDLE_PRIORITY + 2,
        s_fusion_stack, &s_fusion_tcb);
    configASSERT(h);
    vTaskCoreAffinitySet(h, 1u << 0u);
}
