#include "actuator_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "host_protocol.h"

#define ACTUATOR_PERIOD_MS 10U
#define ACTUATOR_FAILURE_LIMIT 3U
#define TARGET_TIMEOUT_MS 500U

static QueueHandle_t target_queue;

bool actuator_task_init(void)
{
    if (target_queue == NULL) {
        target_queue = xQueueCreate(1U, sizeof(motor_target_set_t));
    }
    return target_queue != NULL;
}

bool actuator_submit_targets(const motor_target_set_t *targets)
{
    if ((target_queue == NULL) || !host_protocol_is_armed() ||
        !motor_device_validate_targets(targets)) {
        return false;
    }
    return xQueueOverwrite(target_queue, targets) == pdPASS;
}

void actuator_task(void *argument)
{
    (void) argument;
    motor_target_set_t active_targets = { 0 };
    bool target_valid = false;
    bool torque_enabled = false;
    uint8_t consecutive_failures = 0U;
    TickType_t last_target_tick = 0U;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (host_protocol_get_pending_safety_action() == HOST_SAFETY_ACTION_SAFE_STOP) {
            const motor_device_status_t stop_status = motor_device_set_torque(false);
            torque_enabled = false;
            target_valid = false;
            (void) xQueueReset(target_queue);
            if (stop_status == MOTOR_DEVICE_OK) {
                consecutive_failures = 0U;
                host_protocol_complete_safe_stop();
            } else if (++consecutive_failures >= ACTUATOR_FAILURE_LIMIT) {
                host_protocol_latch_fault(HOST_FAULT_SERVO);
            }
        } else if (!host_protocol_is_armed()) {
            if (torque_enabled) {
                (void) motor_device_set_torque(false);
                torque_enabled = false;
            }
            target_valid = false;
            (void) xQueueReset(target_queue);
        } else {
            motor_target_set_t received_targets;
            if (xQueueReceive(target_queue, &received_targets, 0U) == pdPASS) {
                active_targets = received_targets;
                target_valid = true;
                last_target_tick = xTaskGetTickCount();
            }

            if (target_valid && (xTaskGetTickCount() - last_target_tick) >= pdMS_TO_TICKS(TARGET_TIMEOUT_MS)) {
                target_valid = false;
                host_protocol_request_target_timeout();
            }

            if (target_valid) {
                motor_device_status_t status = motor_device_write_targets(&active_targets);
                if ((status == MOTOR_DEVICE_OK) && !torque_enabled) {
                    status = motor_device_set_torque(true);
                    torque_enabled = status == MOTOR_DEVICE_OK;
                }
                if (status == MOTOR_DEVICE_OK) {
                    consecutive_failures = 0U;
                } else if (++consecutive_failures >= ACTUATOR_FAILURE_LIMIT) {
                    host_protocol_latch_fault(HOST_FAULT_SERVO);
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ACTUATOR_PERIOD_MS));
    }
}
