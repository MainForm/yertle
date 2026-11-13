#include "sensor_task.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define SENSOR_PERIOD_MS 20U
#define OPTICAL_RETRY_MS 1000U

static SemaphoreHandle_t feedback_mutex;
static motor_feedback_set_t latest_feedback;
static imu_feedback_t latest_imu_feedback;
static pmw3901_feedback_t latest_flow_feedback;
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

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu, pmw3901_feedback_t *flow)
{
    if(!motor || !imu || !flow || !feedback_mutex || xSemaphoreTake(feedback_mutex,pdMS_TO_TICKS(5U))!=pdTRUE)return false;
    *motor=latest_feedback;*imu=latest_imu_feedback;*flow=latest_flow_feedback;
    (void)xSemaphoreGive(feedback_mutex);return true;
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
        /* The driver serializes motor I/O on both buses. */
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

static void publish_optical_feedback(const pmw3901_feedback_t *feedback)
{
    if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) == pdTRUE) {
        latest_flow_feedback = *feedback;
        (void)xSemaphoreGive(feedback_mutex);
    }
}

void optical_flow_task(void *argument)
{
    (void)argument;
    pmw3901_device_t device = {0};

    for (;;) {
        bool ready = pmw3901_device_init(&device);
        publish_optical_feedback(&device.feedback);

        TickType_t last_wake = xTaskGetTickCount();
        while (ready) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
            ready = pmw3901_device_service(&device);
            publish_optical_feedback(&device.feedback);
        }

        vTaskDelay(pdMS_TO_TICKS(OPTICAL_RETRY_MS));
    }
}
