#ifndef SAFE_STOP_TEST_QUEUE_H
#define SAFE_STOP_TEST_QUEUE_H
#include <stddef.h>
#include "FreeRTOS.h"
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned length, unsigned item_size);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout);
BaseType_t xQueueReset(QueueHandle_t queue);
#endif
