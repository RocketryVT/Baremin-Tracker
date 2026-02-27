#pragma once

// Create and register the GPS FreeRTOS task.
// Call once before vTaskStartScheduler().
void gps_task_init();
