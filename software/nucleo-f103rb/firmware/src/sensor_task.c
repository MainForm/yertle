#include "sensor_task.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define SENSOR_PERIOD_MS 20U
#define SENSOR_RETRY_MS 1000U

static SemaphoreHandle_t feedback_mutex;
static motor_feedback_set_t latest_feedback;
static imu_feedback_t latest_imu_feedback;
static pmw3901_feedback_t latest_flow_feedback;
static uint32_t feedback_sequence;

bool sensor_task_init(const pmw3901_feedback_t *flow_feedback)
{
    if (feedback_mutex == NULL) {
        feedback_mutex = xSemaphoreCreateMutex();
    }
    latest_flow_feedback = *flow_feedback;
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

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu, pmw3901_feedback_t *flow)
{
    if(!motor || !imu || !flow || !feedback_mutex || xSemaphoreTake(feedback_mutex,pdMS_TO_TICKS(5U))!=pdTRUE)return false;
    *motor=latest_feedback;*imu=latest_imu_feedback;*flow=latest_flow_feedback;
    (void)xSemaphoreGive(feedback_mutex);return true;
}

void sensor_task(void *argument)
{
    pmw3901_device_t *flow = argument;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t next_imu_probe = last_wake + pdMS_TO_TICKS(SENSOR_RETRY_MS);
    TickType_t next_flow_probe = last_wake + pdMS_TO_TICKS(SENSOR_RETRY_MS);

    for (;;) {
        motor_feedback_set_t sample = { 0 };
        imu_feedback_t imu = { 0 };
        /* The driver serializes motor I/O on both buses. */
        (void) motor_device_read_feedback(&sample);
        sample.sequence = ++feedback_sequence;
        sample.uptime_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        imu.sequence = sample.sequence;
        imu.uptime_ms = sample.uptime_ms;
        const bool imu_ready = imu_device_is_ready();
        if (imu_ready || (int32_t)(xTaskGetTickCount() - next_imu_probe) >= 0) {
            if (imu_ready || imu_device_init()) {
                imu.valid = imu_device_read(&imu.raw);
            }
            if (!imu.valid) {
                next_imu_probe = xTaskGetTickCount() + pdMS_TO_TICKS(SENSOR_RETRY_MS);
            }
        }

        if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) == pdTRUE) {
            latest_feedback = sample;
            latest_imu_feedback = imu;
            (void) xSemaphoreGive(feedback_mutex);
        }

        /* Publish motor and IMU feedback before any blocking optical work. */
        const TickType_t flow_now = xTaskGetTickCount();
        if (flow->feedback.ready || (int32_t)(flow_now - next_flow_probe) >= 0) {
            const bool ready = flow->feedback.ready ? pmw3901_device_service(flow)
                                                   : pmw3901_device_init(flow);
            if (!ready) {
                next_flow_probe = xTaskGetTickCount() + pdMS_TO_TICKS(SENSOR_RETRY_MS);
            }
            if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) == pdTRUE) {
                latest_flow_feedback = flow->feedback;
                (void) xSemaphoreGive(feedback_mutex);
            }
        }

        /* Keep the periodic schedule but skip missed periods after long I/O. */
        const TickType_t period = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
        const TickType_t elapsed = xTaskGetTickCount() - last_wake;
        if (elapsed > period) {
            last_wake += (elapsed / period) * period;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}
