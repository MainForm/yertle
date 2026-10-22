#ifndef SAFE_STOP_TEST_TASK_H
#define SAFE_STOP_TEST_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *previous, TickType_t increment);
#endif
