#pragma once

// Create and register the USB CDC logging FreeRTOS task.
// Call once before vTaskStartScheduler().
void usb_task_init();
