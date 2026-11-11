#include "actuator_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "motor_control.h"

#define ACTUATOR_PERIOD_MS 10U

void actuator_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        motor_control_service();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ACTUATOR_PERIOD_MS));
    }
}
