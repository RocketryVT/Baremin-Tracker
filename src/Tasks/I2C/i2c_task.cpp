#include "i2c_task.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

// ---------------------------------------------------------------------------
// Internal queue — holds pointers to caller-owned I2cRequest structs.
// Depth of 16 is generous; in practice baro + IMU requests interleave.
// ---------------------------------------------------------------------------
#define I2C_QUEUE_DEPTH 16

static StaticQueue_t  s_queue_buf;
static uint8_t        s_queue_storage[ I2C_QUEUE_DEPTH * sizeof(I2cRequest*) ];
static QueueHandle_t  s_queue = nullptr;

// Set to true inside the i2c task's first iteration — signals that the
// scheduler is running and the queue is being drained.
static volatile bool  s_ready = false;

bool i2c_task_ready() { return s_ready; }

// ---------------------------------------------------------------------------
// Task storage
// ---------------------------------------------------------------------------
static StaticTask_t s_tcb;
static StackType_t  s_stack[256];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void i2c_req_write(I2cRequest* req,
                   uint8_t addr,
                   const uint8_t* write_data, uint8_t write_len)
{
    req->addr      = addr;
    req->write_len = write_len;
    req->read_buf  = nullptr;
    req->read_len  = 0;
    req->ok        = false;

    for (uint8_t i = 0; i < write_len; i++) {
        req->write_buf[i] = write_data[i];
    }

    req->sem = xSemaphoreCreateBinaryStatic(&req->sem_buf);
}

void i2c_req_write_read(I2cRequest* req,
                        uint8_t addr,
                        const uint8_t* write_data, uint8_t write_len,
                        uint8_t* read_buf, uint8_t read_len)
{
    req->addr      = addr;
    req->write_len = write_len;
    req->read_buf  = read_buf;
    req->read_len  = read_len;
    req->ok        = false;

    for (uint8_t i = 0; i < write_len; i++) {
        req->write_buf[i] = write_data[i];
    }

    req->sem = xSemaphoreCreateBinaryStatic(&req->sem_buf);
}

bool i2c_req_submit_wait(I2cRequest* req)
{
    // Pre-scheduler (called from initialize() before vTaskStartScheduler):
    // execute the transfer directly — the bus is idle and we have exclusive access.
    if (!s_ready) {
        bool ok = true;
        if (req->read_buf && req->read_len > 0) {
            int wr = i2c_write_blocking(i2c0, req->addr,
                                        req->write_buf, req->write_len, true);
            ok = (wr == (int)req->write_len);
            if (ok) {
                int rd = i2c_read_blocking(i2c0, req->addr,
                                           req->read_buf, req->read_len, false);
                ok = (rd == (int)req->read_len);
            }
        } else {
            int wr = i2c_write_blocking(i2c0, req->addr,
                                        req->write_buf, req->write_len, false);
            ok = (wr == (int)req->write_len);
        }
        req->ok = ok;
        return ok;
    }

    // Post-scheduler: hand off to the i2c task and block until complete.
    I2cRequest* ptr = req;
    xQueueSend(s_queue, &ptr, portMAX_DELAY);
    xSemaphoreTake(req->sem, portMAX_DELAY);
    return req->ok;
}

// ---------------------------------------------------------------------------
// I2C task — sole executor of i2c0 transfers
// ---------------------------------------------------------------------------
static void i2c_task(void*)
{
    s_ready = true;
    for (;;) {
        I2cRequest* req = nullptr;
        xQueueReceive(s_queue, &req, portMAX_DELAY);

        if (!req) continue;

        bool ok = true;

        if (req->read_buf && req->read_len > 0) {
            // Write register address (or any prefix), then read.
            int wr = i2c_write_blocking(i2c0, req->addr,
                                        req->write_buf, req->write_len,
                                        true);   // nostop = true
            if (wr != (int)req->write_len) {
                ok = false;
            } else {
                int rd = i2c_read_blocking(i2c0, req->addr,
                                           req->read_buf, req->read_len,
                                           false);
                ok = (rd == (int)req->read_len);
            }
        } else {
            // Pure write.
            int wr = i2c_write_blocking(i2c0, req->addr,
                                        req->write_buf, req->write_len,
                                        false);
            ok = (wr == (int)req->write_len);
        }

        req->ok = ok;
        xSemaphoreGive(req->sem);
    }
}

// ---------------------------------------------------------------------------
// Suspend / resume (for i2c scan)
// ---------------------------------------------------------------------------
static TaskHandle_t s_task_handle = nullptr;

void i2c_task_suspend()
{
    if (s_task_handle) vTaskSuspend(s_task_handle);
}

void i2c_task_resume()
{
    if (s_task_handle) vTaskResume(s_task_handle);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void i2c_task_init()
{
    // Own the i2c0 hardware initialisation — must be called before any task
    // that uses the bus (including baro_task_init which calls s_baro.initialize()).
    i2c_init( i2c0, 400'000 );
    gpio_set_function( 20u, GPIO_FUNC_I2C );  // BARO_SDA / IMU_SDA
    gpio_set_function( 21u, GPIO_FUNC_I2C );  // BARO_SCL / IMU_SCL
    gpio_pull_up( 20u );
    gpio_pull_up( 21u );

    s_queue = xQueueCreateStatic(I2C_QUEUE_DEPTH,
                                 sizeof(I2cRequest*),
                                 s_queue_storage,
                                 &s_queue_buf);
    configASSERT(s_queue);

    s_task_handle = xTaskCreateStatic(
        i2c_task, "i2c0", 256,
        nullptr, tskIDLE_PRIORITY + 5,   // highest priority — never blocks long
        s_stack, &s_tcb);
    configASSERT(s_task_handle);
    // Run on core 0 with the other i2c users (baro ISR callbacks also core 0)
    vTaskCoreAffinitySet(s_task_handle, 1u << 0u);
}
