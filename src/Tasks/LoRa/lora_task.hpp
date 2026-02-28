#pragma once

// Create and register the LoRa TX FreeRTOS task.
// Call once before vTaskStartScheduler().
void lora_task_init();
void lora_task( void* param );
