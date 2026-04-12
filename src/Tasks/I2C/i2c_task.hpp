#pragma once

#include <stdint.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "semphr.h"

// ---------------------------------------------------------------------------
// I2C0 task — serialises all i2c0 bus access from any task context.
//
// Usage:
//   I2cRequest req;
//   uint8_t    wbuf[2] = { reg, val };
//   i2c_req_write(&req, addr, wbuf, 2);          // fire-and-forget write
//   i2c_req_write_read(&req, addr, &reg, 1,      // write reg addr, then read
//                      rbuf, len);
//   i2c_req_submit_wait(&req);                   // blocks until done
//
// The request struct must remain valid until i2c_req_submit_wait() returns.
// Do NOT call from ISR context.
// ---------------------------------------------------------------------------

// Maximum payload that can be written or read in one request.
// Raise if needed (costs stack on the caller's side).
#define I2C_REQ_MAX_WRITE  8
#define I2C_REQ_MAX_READ   32

struct I2cRequest {
    uint8_t  addr;                      ///< 7-bit I2C device address

    uint8_t  write_buf[I2C_REQ_MAX_WRITE];
    uint8_t  write_len;

    uint8_t* read_buf;                  ///< pointer to caller's buffer (may be NULL)
    uint8_t  read_len;

    bool     ok;                        ///< written by i2c task — true if transfer succeeded

    // Completion signal — initialised by i2c_req_write / i2c_req_write_read.
    StaticSemaphore_t sem_buf;
    SemaphoreHandle_t sem;
};

// ---------------------------------------------------------------------------
// Helpers to fill a request and submit it.
// ---------------------------------------------------------------------------

/// Pure write: sends write_data[write_len] to addr.
void i2c_req_write(I2cRequest* req,
                   uint8_t addr,
                   const uint8_t* write_data, uint8_t write_len);

/// Write-then-read: sends write_data[write_len] (e.g. register address),
/// then reads read_len bytes into read_buf.
void i2c_req_write_read(I2cRequest* req,
                        uint8_t addr,
                        const uint8_t* write_data, uint8_t write_len,
                        uint8_t* read_buf, uint8_t read_len);

/// Submit the request to the i2c task queue and block until complete.
/// Returns true if the transfer succeeded.
bool i2c_req_submit_wait(I2cRequest* req);

// ---------------------------------------------------------------------------
// Task lifecycle — call once in main() before baro_task_init().
// Initialises i2c0 hardware, creates the queue, and registers the task.
// ---------------------------------------------------------------------------
void i2c_task_init();

// Returns true once the scheduler is running and the i2c task is active.
// i2c_req_submit_wait() checks this; if false it falls back to direct SDK calls
// so that initialize() (called pre-scheduler) works without a running task.
bool i2c_task_ready();

// Suspend / resume the i2c task — used by i2c scan to safely take the bus.
// Caller must also suspend all tasks that submit to the queue (baro tasks)
// before calling suspend, otherwise submitted requests will never complete.
void i2c_task_suspend();
void i2c_task_resume();
