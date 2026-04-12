#pragma once

// Create and register all barometer FreeRTOS tasks.
// Call once before vTaskStartScheduler().
void baro_task_init();

// Suspend / resume all three baro tasks (for i2c0 bus access from other tasks).
void baro_task_suspend();
void baro_task_resume();
