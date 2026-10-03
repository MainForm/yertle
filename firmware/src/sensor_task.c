#include "sensor_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define SENSOR_PERIOD_MS 20U

static SemaphoreHandle_t feedback_mutex;
static motor_feedback_set_t latest_feedback;
static imu_feedback_t latest_imu_feedback;
static uint32_t feedback_sequence;

bool sensor_task_init(void)
{
    if (feedback_mutex == NULL) {
        feedback_mutex = xSemaphoreCreateMutex();
    }
    return feedback_mutex != NULL;
}

bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu)
{
    if ((motor == NULL) || (imu == NULL) || (feedback_mutex == NULL) ||
        (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(5U)) != pdTRUE)) {
        return false;
    }
    *motor = latest_feedback;
    *imu = latest_imu_feedback;
    (void) xSemaphoreGive(feedback_mutex);
    return true;
}

void sensor_task(void *argument)
{
    (void) argument;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t next_imu_probe = last_wake;
    bool imu_ready = false;

    for (;;) {
        motor_feedback_set_t sample = { 0 };
        imu_feedback_t imu = { 0 };
        /* Driver reads USART1's DMA buffer while this sensor task owns the bus. */
        (void) motor_device_read_feedback(&sample);
        sample.sequence = ++feedback_sequence;
        sample.uptime_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        const TickType_t now = xTaskGetTickCount();
        if (!imu_ready && (int32_t)(now - next_imu_probe) >= 0) {
            imu_ready = imu_device_init();
            next_imu_probe = now + pdMS_TO_TICKS(1000U);
        }
        imu.sequence = sample.sequence;
        imu.uptime_ms = sample.uptime_ms;
        if (imu_ready) {
            imu.valid = imu_device_read(&imu.raw);
            if (!imu.valid) {
                imu_ready = false;
                next_imu_probe = xTaskGetTickCount() + pdMS_TO_TICKS(1000U);
            }
        }

        if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) == pdTRUE) {
            latest_feedback = sample;
            latest_imu_feedback = imu;
            (void) xSemaphoreGive(feedback_mutex);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
