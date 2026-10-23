#ifndef PWM_TEST_FREERTOS_H
#define PWM_TEST_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
void pwm_test_enter(void);
void pwm_test_exit(void);
#define taskENTER_CRITICAL() pwm_test_enter()
#define taskEXIT_CRITICAL() pwm_test_exit()
#endif
