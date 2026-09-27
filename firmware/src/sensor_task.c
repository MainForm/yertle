#include "sensor_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define SENSOR_PERIOD_MS 20U

static SemaphoreHandle_t feedback_mutex;
static motor_feedback_set_t latest_feedback;
static uint32_t feedback_sequence;

bool sensor_task_init(void)
{
    if (feedback_mutex == NULL) {
        feedback_mutex = xSemaphoreCreateMutex();
    }
    return feedback_mutex != NULL;
}

bool sensor_get_motor_feedback(motor_feedback_set_t *feedback)
{
    if ((feedback == NULL) || (feedback_mutex == NULL) ||
        (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(5U)) != pdTRUE)) {
        return false;
    }
    *feedback = latest_feedback;
    (void) xSemaphoreGive(feedback_mutex);
    return true;
}

void sensor_task(void *argument)
{
    (void) argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        motor_feedback_set_t sample = { 0 };
        /* Driver reads USART1's ISR ring while this sensor task owns the bus. */
        (void) motor_device_read_feedback(&sample);
        sample.sequence = ++feedback_sequence;

        if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) == pdTRUE) {
            latest_feedback = sample;
            (void) xSemaphoreGive(feedback_mutex);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
