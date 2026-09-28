#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
#define portTICK_PERIOD_MS 1U
/* Reintroducing diagnostic critical sections must fail the test build. */
#define taskENTER_CRITICAL() DIAGNOSTICS_MUST_NOT_MASK_INTERRUPTS
#define taskEXIT_CRITICAL() DIAGNOSTICS_MUST_NOT_MASK_INTERRUPTS
#endif
