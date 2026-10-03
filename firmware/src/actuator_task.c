#include "actuator_task.h"

#include <limits.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "host_protocol.h"
#include "sensor_task.h"

#define ACTUATOR_PERIOD_MS 10U
#define ACTUATOR_FAILURE_LIMIT 3U
#define TARGET_TIMEOUT_MS 500U
#define SAFE_STOP_CENTER_POSITION 2048U
#define SAFE_STOP_SPEED_RAW 200U
#define SAFE_STOP_ACCELERATION_RAW 10U
#define SAFE_STOP_COMMAND_PERIOD_MS 20U
#define SAFE_STOP_POSITION_TOLERANCE 16U
#define SAFE_STOP_SETTLED_SAMPLES 3U
#define SAFE_STOP_FEEDBACK_MAX_AGE_MS 150U
#define SAFE_STOP_STALL_TIMEOUT_MS 500U
#define SAFE_STOP_TOTAL_TIMEOUT_MS 15000U
#define SAFE_STOP_PROGRESS_STEP 12U

typedef struct {
    bool active;
    bool return_finished;
    TickType_t started_tick;
    TickType_t last_command_tick;
    TickType_t last_progress_tick;
    uint32_t last_sample_sequence;
    uint32_t best_total_error;
    uint8_t settled_samples;
} safe_stop_context_t;

static QueueHandle_t target_queue;

static motor_target_set_t center_targets(void)
{
    motor_target_set_t targets = { 0 };
    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        targets.motor[index].position = SAFE_STOP_CENTER_POSITION;
        targets.motor[index].speed = SAFE_STOP_SPEED_RAW;
        targets.motor[index].acceleration = SAFE_STOP_ACCELERATION_RAW;
    }
    return targets;
}

/* Only fresh, complete feedback may declare the robot centered. */
static bool center_reached(safe_stop_context_t *stop, TickType_t now)
{
    motor_feedback_set_t feedback;
    imu_feedback_t imu;
    if (!sensor_get_feedback_snapshot(&feedback, &imu) ||
        feedback.sequence == 0U || feedback.sequence == stop->last_sample_sequence) {
        return false;
    }
    stop->last_sample_sequence = feedback.sequence;
    if (feedback.valid_mask != YERTLE_ALL_MOTORS_MASK ||
        ((uint32_t)(now * portTICK_PERIOD_MS) - feedback.uptime_ms) >
            SAFE_STOP_FEEDBACK_MAX_AGE_MS) {
        stop->settled_samples = 0U;
        return false;
    }

    uint32_t total_error = 0U;
    bool all_centered = true;
    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        const motor_feedback_t *motor = &feedback.motor[index];
        if (!motor->valid || motor->servo_error != 0U || motor->position < 0 ||
            motor->position > 4095) {
            stop->settled_samples = 0U;
            return false;
        }
        const int32_t error = (int32_t)motor->position - (int32_t)SAFE_STOP_CENTER_POSITION;
        const uint32_t magnitude = (uint32_t)(error < 0 ? -error : error);
        total_error += magnitude;
        if (magnitude > SAFE_STOP_POSITION_TOLERANCE) all_centered = false;
    }

    if (stop->best_total_error == UINT32_MAX ||
        total_error + SAFE_STOP_PROGRESS_STEP <= stop->best_total_error) {
        stop->best_total_error = total_error;
        stop->last_progress_tick = now;
    }
    stop->settled_samples = all_centered ? (uint8_t)(stop->settled_samples + 1U) : 0U;
    return stop->settled_samples >= SAFE_STOP_SETTLED_SAMPLES;
}

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
    safe_stop_context_t stop = { 0 };
    const motor_target_set_t centered = center_targets();
    TickType_t last_target_tick = 0U;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (host_protocol_get_pending_safety_action() == HOST_SAFETY_ACTION_SAFE_STOP) {
            target_valid = false;
            (void) xQueueReset(target_queue);
            const TickType_t now = xTaskGetTickCount();
            if (!stop.active) {
                stop.active = true;
                stop.return_finished = false;
                stop.started_tick = now;
                stop.last_command_tick = now - pdMS_TO_TICKS(SAFE_STOP_COMMAND_PERIOD_MS);
                stop.last_progress_tick = now;
                stop.last_sample_sequence = 0U;
                stop.best_total_error = UINT32_MAX;
                stop.settled_samples = 0U;
                consecutive_failures = 0U;
            }

            bool disable_torque = !torque_enabled || stop.return_finished ||
                                  host_protocol_get_state() == HOST_FSM_FAULT;
            bool return_failed = false;
            if (!disable_torque) {
                if ((now - stop.started_tick) >= pdMS_TO_TICKS(SAFE_STOP_TOTAL_TIMEOUT_MS) ||
                    (now - stop.last_progress_tick) >= pdMS_TO_TICKS(SAFE_STOP_STALL_TIMEOUT_MS)) {
                    return_failed = true;
                } else {
                    if ((now - stop.last_command_tick) >=
                        pdMS_TO_TICKS(SAFE_STOP_COMMAND_PERIOD_MS)) {
                        stop.last_command_tick = now;
                        if (motor_device_write_targets(&centered) == MOTOR_DEVICE_OK) {
                            consecutive_failures = 0U;
                        } else if (++consecutive_failures >= ACTUATOR_FAILURE_LIMIT) {
                            return_failed = true;
                        }
                    }
                    if (!return_failed && center_reached(&stop, now)) {
                        stop.return_finished = true;
                        disable_torque = true;
                    }
                }
            }

            if (return_failed) {
                /* Never keep driving toward an unreachable pose indefinitely. */
                host_protocol_latch_fault(HOST_FAULT_SERVO);
                stop.return_finished = true;
                disable_torque = true;
            }
            if (disable_torque) {
                if (motor_device_set_torque(false) == MOTOR_DEVICE_OK) {
                    torque_enabled = false;
                    consecutive_failures = 0U;
                    host_protocol_complete_safe_stop();
                } else if (++consecutive_failures >= ACTUATOR_FAILURE_LIMIT) {
                    host_protocol_latch_fault(HOST_FAULT_SERVO);
                }
            }
        } else if (!host_protocol_is_armed()) {
            stop.active = false;
            if (torque_enabled) {
                if (motor_device_set_torque(false) == MOTOR_DEVICE_OK) torque_enabled = false;
            }
            target_valid = false;
            (void) xQueueReset(target_queue);
        } else {
            stop.active = false;
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
