#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
#define portTICK_PERIOD_MS 1U
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portMAX_DELAY UINT32_MAX
/* Reintroducing diagnostic critical sections must fail the test build. */
#define taskENTER_CRITICAL() DIAGNOSTICS_MUST_NOT_MASK_INTERRUPTS
#define taskEXIT_CRITICAL() DIAGNOSTICS_MUST_NOT_MASK_INTERRUPTS
#endif
