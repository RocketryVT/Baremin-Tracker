#pragma once

// Create and register the ICM-40609-D IMU FreeRTOS task.
// Call once before vTaskStartScheduler().
void imu_task_init();
