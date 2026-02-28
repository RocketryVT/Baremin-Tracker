#pragma once

// Create and register all barometer FreeRTOS tasks.
// Call once before vTaskStartScheduler().
void baro_task_init();
