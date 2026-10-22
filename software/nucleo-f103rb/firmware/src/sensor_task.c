#include "sensor_task.h"
#include "host_protocol.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define SENSOR_PERIOD_MS 20U

static SemaphoreHandle_t feedback_mutex;
static motor_feedback_set_t latest_feedback;
static imu_feedback_t latest_imu_feedback;
static uint32_t feedback_sequence;
static motor_position_offsets_t position_offsets;
static bool offsets_pending;
static bool offsets_ready;
static uint8_t offsets_next_index;
static uint8_t offsets_attempt_count;
static motor_register_result_t register_result;
static bool register_pending, register_ready;
static motor_register_write_request_t register_write_request;
static motor_register_write_result_t register_write_result;
static bool register_write_pending, register_write_ready;

static bool diagnostic_job_available(void)
{
    return !offsets_pending && !offsets_ready && !register_pending && !register_ready &&
           !register_write_pending && !register_write_ready;
}

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

bool sensor_request_position_offsets(uint16_t request_sequence)
{
    if ((feedback_mutex == NULL) ||
        (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(5U)) != pdTRUE)) return false;
    const bool accepted = diagnostic_job_available();
    if (accepted) {
        memset(&position_offsets, 0, sizeof(position_offsets));
        position_offsets.request_sequence = request_sequence;
        offsets_next_index = 0U;
        offsets_attempt_count = 0U;
        offsets_pending = true;
    }
    (void)xSemaphoreGive(feedback_mutex);
    return accepted;
}

bool sensor_take_position_offsets(motor_position_offsets_t *result)
{
    if ((result == NULL) || (feedback_mutex == NULL) ||
        (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(5U)) != pdTRUE)) return false;
    const bool ready = offsets_ready;
    if (ready) {
        *result = position_offsets;
        offsets_ready = false;
    }
    (void)xSemaphoreGive(feedback_mutex);
    return ready;
}

/* One attempt per sensor iteration, not a blocking five-read loop. Sensor task
 * is the sole worker; never hold the snapshot mutex during UART I/O. */
static void collect_next_position_offset(void)
{
    if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) != pdTRUE) return;
    const bool pending = offsets_pending;
    const uint8_t index = offsets_next_index;
    (void)xSemaphoreGive(feedback_mutex);
    if (!pending) return;

    motor_position_offset_t offset = { 0 };
    const motor_device_status_t status = motor_device_read_position_offset(index + 1U, &offset);
    if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) == pdTRUE) {
        position_offsets.motor[index] = offset;
        position_offsets.status[index] = status;
        ++offsets_attempt_count;
        if (status == MOTOR_DEVICE_OK) position_offsets.valid_mask |= (uint16_t)(1U << index);
        if ((status == MOTOR_DEVICE_OK) ||
            (offsets_attempt_count >= SENSOR_POSITION_OFFSET_MAX_ATTEMPTS)) {
            ++offsets_next_index;
            offsets_attempt_count = 0U;
        }
        if (offsets_next_index == YERTLE_MOTOR_COUNT) {
            position_offsets.uptime_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            offsets_pending = false;
            offsets_ready = true;
        }
        (void)xSemaphoreGive(feedback_mutex);
    }
}

bool sensor_request_motor_register(uint16_t sequence, uint8_t id, uint8_t address, uint8_t length)
{
    if (!motor_register_request_valid(id, address, length) || feedback_mutex == NULL ||
        xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(5U)) != pdTRUE) return false;
    const bool accepted = diagnostic_job_available();
    if (accepted) {
        register_result = (motor_register_result_t){
            .request_sequence = sequence, .motor_id = id, .address = address, .length = length
        };
        register_pending = true;
    }
    (void)xSemaphoreGive(feedback_mutex);
    return accepted;
}

bool sensor_take_motor_register(motor_register_result_t *result)
{
    if (result == NULL || feedback_mutex == NULL ||
        xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(5U)) != pdTRUE) return false;
    const bool ready = register_ready;
    if (ready) { *result = register_result; register_ready = false; }
    (void)xSemaphoreGive(feedback_mutex);
    return ready;
}

static void collect_motor_register(void)
{
    if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) != pdTRUE) return;
    const bool pending = register_pending;
    motor_register_result_t result = register_result;
    (void)xSemaphoreGive(feedback_mutex);
    if (!pending) return;
    /* Recheck just before I/O in case a fault/stop arrived after acceptance. */
    if (host_protocol_get_state() != HOST_FSM_DISARMED || host_protocol_is_stop_requested()) {
        result.status = MOTOR_REGISTER_BUSY;
    } else {
        result.status = motor_device_read_register(result.motor_id, result.address, result.length, &result.reading);
    }
    if (xSemaphoreTake(feedback_mutex, portMAX_DELAY) == pdTRUE) {
        register_result = result;
        register_pending = false;
        register_ready = true;
        (void)xSemaphoreGive(feedback_mutex);
    }
}

static bool register_write_allowed(void)
{
    return host_protocol_get_state()==HOST_FSM_DISARMED && !host_protocol_is_stop_requested();
}

bool sensor_request_motor_register_write(uint16_t sequence, const motor_register_write_request_t *request)
{
    if (motor_register_validate_write(request)!=MOTOR_WRITE_OK || feedback_mutex==NULL ||
        xSemaphoreTake(feedback_mutex,pdMS_TO_TICKS(5U))!=pdTRUE) return false;
    const bool accepted=diagnostic_job_available();
    if (accepted) {
        register_write_request=*request;
        register_write_result=(motor_register_write_result_t){ .request_sequence=sequence };
        register_write_result.report=motor_register_initial_report();
        register_write_pending=true;
    }
    (void)xSemaphoreGive(feedback_mutex);
    return accepted;
}

bool sensor_take_motor_register_write(motor_register_write_result_t *result)
{
    if (result==NULL || feedback_mutex==NULL ||
        xSemaphoreTake(feedback_mutex,pdMS_TO_TICKS(5U))!=pdTRUE) return false;
    const bool ready=register_write_ready;
    if (ready) { *result=register_write_result; register_write_ready=false; }
    (void)xSemaphoreGive(feedback_mutex);
    return ready;
}

static void collect_motor_register_write(void)
{
    if (xSemaphoreTake(feedback_mutex,portMAX_DELAY)!=pdTRUE) return;
    const bool pending=register_write_pending;
    const motor_register_write_request_t request=register_write_request;
    motor_register_write_result_t result=register_write_result;
    (void)xSemaphoreGive(feedback_mutex);
    if (!pending) return;
    if (!register_write_allowed()) result.report.status=result.report.primary_status=MOTOR_WRITE_INVALID_STATE;
    else result.report=motor_device_write_register(&request,register_write_allowed);
    if (xSemaphoreTake(feedback_mutex,portMAX_DELAY)==pdTRUE) {
        register_write_result=result;
        register_write_pending=false;
        register_write_ready=true;
        (void)xSemaphoreGive(feedback_mutex);
    }
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
        collect_next_position_offset();
        collect_motor_register();
        collect_motor_register_write();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
