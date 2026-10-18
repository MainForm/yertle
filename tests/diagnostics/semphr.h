#ifndef TEST_SEMPHR_H
#define TEST_SEMPHR_H
#include "FreeRTOS.h"
typedef void *SemaphoreHandle_t;
#ifdef SENSOR_TEST
SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout);
int xSemaphoreGive(SemaphoreHandle_t handle);
#else
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
static inline int xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout) { (void)timeout; return handle != 0; }
static inline int xSemaphoreGive(SemaphoreHandle_t handle) { return handle != 0; }
#endif
#define pdTRUE 1
#endif
