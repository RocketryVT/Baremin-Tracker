#pragma once

#include "FreeRTOS.h"
#include "task.h"

// Create and register the flash-logger FreeRTOS task.
// Call once before vTaskStartScheduler().
void logger_task_init();

// -- Console helpers -----------------------------------------------------------
// Intended for use from the serial console (USB task, core 0).

// Return the logger task handle so the caller can vTaskSuspend/Resume it
// around any flash operation.
TaskHandle_t logger_task_get_handle();

// Force-start recording immediately, bypassing launch detection.
// The logger task will flush the circular buffer and switch to direct writes
// on its next iteration.  Has no effect if recording is already active.
void logger_task_force_record_on();

// Force-stop recording.  Incoming records are silently dropped until
// logger_task_force_record_on() is called or the device is power-cycled.
void logger_task_force_record_off();

// Decode and print every SigmaStorageFullRecord in flash to stdout.
// MUST be called with the logger task suspended to prevent concurrent flash
// access.  The caller is responsible for vTaskSuspend / vTaskResume.
void logger_task_read_decoded();

// Decode and print every SigmaStorageFullRecord in flash to stdout as CSV.
// MUST be called with the logger task suspended (same reason as above).
void logger_task_read_csv();

// Hex-dump all raw flash bytes via Logger::read_memory() to stdout.
// MUST be called with the logger task suspended (same reason as above).
void logger_task_read_raw();

// Erase the entire flash log and reset logger state so it can be reused.
// MUST be called with the logger task suspended.
void logger_task_erase();
