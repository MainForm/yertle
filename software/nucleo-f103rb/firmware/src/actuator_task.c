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

typedef enum {
    SAFE_STOP_IDLE = 0,
    SAFE_STOP_RETURNING,
    SAFE_STOP_DISABLING_TORQUE,
    SAFE_STOP_WAITING_FOR_FAULT,
    SAFE_STOP_TORQUE_OFF_ONLY,
    SAFE_STOP_COMPLETE
} safe_stop_phase_t;

typedef struct {
    safe_stop_phase_t phase;
    bool fault_recovery;
    bool torque_on_sent;
    uint8_t consecutive_failures;
    TickType_t started_tick;
    TickType_t last_command_tick;
    TickType_t last_progress_tick[YERTLE_MOTOR_COUNT];
    uint16_t best_error[YERTLE_MOTOR_COUNT];
    uint32_t last_sample_sequence;
    uint8_t settled_samples;
} safe_stop_context_t;

/* Keep the acceptance time with its targets: delayed consumption must not
 * extend the host command watchdog, and failed enqueue must not refresh it. */
typedef struct {
    motor_target_set_t targets;
    TickType_t accepted_tick;
} queued_target_t;

static QueueHandle_t target_queue;

static void begin_stop_attempt(safe_stop_context_t *stop, TickType_t now,
                               bool return_to_center, bool torque_on_sent)
{
    stop->phase = return_to_center ? SAFE_STOP_RETURNING : SAFE_STOP_DISABLING_TORQUE;
    stop->torque_on_sent = torque_on_sent;
    stop->consecutive_failures = 0U;
    stop->started_tick = now;
    stop->last_command_tick = now - pdMS_TO_TICKS(SAFE_STOP_COMMAND_PERIOD_MS);
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        stop->last_progress_tick[i] = now;
        stop->best_error[i] = UINT16_MAX;
    }
    stop->last_sample_sequence = 0U;
    stop->settled_samples = 0U;
}

static bool stop_has_stalled(const safe_stop_context_t *stop, TickType_t now)
{
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        if ((TickType_t)(now - stop->last_progress_tick[i]) >=
            pdMS_TO_TICKS(SAFE_STOP_STALL_TIMEOUT_MS)) return true;
    }
    return false;
}

static void stop_attempt_failed(safe_stop_context_t *stop)
{
    if (!stop->fault_recovery) {
        host_protocol_latch_fault(HOST_FAULT_SERVO);
        stop->phase = SAFE_STOP_WAITING_FOR_FAULT;
    } else {
        /* The single FAULT return attempt failed; only retry torque-off now. */
        stop->phase = SAFE_STOP_TORQUE_OFF_ONLY;
    }
}

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

    bool all_centered = true;
    /* Validate the entire sample before refreshing any motor's progress. */
    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        const motor_feedback_t *motor = &feedback.motor[index];
        if (!motor->valid || motor->servo_error != 0U || motor->position < 0 ||
            motor->position > 4095) {
            stop->settled_samples = 0U;
            return false;
        }
    }
    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        const motor_feedback_t *motor = &feedback.motor[index];
        const int32_t error = (int32_t)motor->position - (int32_t)SAFE_STOP_CENTER_POSITION;
        const uint32_t magnitude = (uint32_t)(error < 0 ? -error : error);
        if (magnitude > SAFE_STOP_POSITION_TOLERANCE) all_centered = false;
        if (magnitude <= SAFE_STOP_POSITION_TOLERANCE ||
            stop->best_error[index] == UINT16_MAX ||
            magnitude + SAFE_STOP_PROGRESS_STEP <= stop->best_error[index]) {
            stop->best_error[index] = (uint16_t)magnitude;
            stop->last_progress_tick[index] = now;
        }
    }
    stop->settled_samples = all_centered ? (uint8_t)(stop->settled_samples + 1U) : 0U;
    return stop->settled_samples >= SAFE_STOP_SETTLED_SAMPLES;
}

bool actuator_task_init(void)
{
    if (target_queue == NULL) {
        target_queue = xQueueCreate(1U, sizeof(queued_target_t));
    }
    return target_queue != NULL;
}

bool actuator_submit_targets(const motor_target_set_t *targets)
{
    if ((target_queue == NULL) || !host_protocol_is_armed() || host_protocol_is_stop_requested() ||
        !motor_device_validate_targets(targets)) {
        return false;
    }
    const queued_target_t queued = {
        .targets = *targets,
        .accepted_tick = xTaskGetTickCount()
    };
    return xQueueOverwrite(target_queue, &queued) == pdPASS;
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
        const bool in_fault = host_protocol_get_state() == HOST_FSM_FAULT;
        if (in_fault || host_protocol_is_stop_requested()) {
            target_valid = false;
            (void) xQueueReset(target_queue);
            const TickType_t now = xTaskGetTickCount();
            if (in_fault && !stop.fault_recovery) {
                stop.fault_recovery = true;
                begin_stop_attempt(&stop, now, true, false);
            } else if (stop.phase == SAFE_STOP_IDLE) {
                /* Boot never enables torque just to return to center. */
                begin_stop_attempt(&stop, now, torque_enabled, torque_enabled);
            }

            switch (stop.phase) {
            case SAFE_STOP_RETURNING:
                if ((now - stop.started_tick) >= pdMS_TO_TICKS(SAFE_STOP_TOTAL_TIMEOUT_MS) ||
                    stop_has_stalled(&stop, now)) {
                    stop_attempt_failed(&stop);
                    break;
                }
                if ((now - stop.last_command_tick) >= pdMS_TO_TICKS(SAFE_STOP_COMMAND_PERIOD_MS)) {
                    stop.last_command_tick = now;
                    motor_device_status_t status = motor_device_write_targets(&centered);
                    if (status == MOTOR_DEVICE_OK && !stop.torque_on_sent) {
                        status = motor_device_set_torque(true);
                        if (status == MOTOR_DEVICE_OK) {
                            stop.torque_on_sent = true;
                            torque_enabled = true;
                        }
                    }
                    if (status == MOTOR_DEVICE_OK) {
                        stop.consecutive_failures = 0U;
                    } else if (++stop.consecutive_failures >= ACTUATOR_FAILURE_LIMIT) {
                        stop_attempt_failed(&stop);
                        break;
                    }
                }
                if (stop.torque_on_sent && center_reached(&stop, now)) {
                    stop.phase = SAFE_STOP_DISABLING_TORQUE;
                    stop.consecutive_failures = 0U;
                }
                break;
            case SAFE_STOP_DISABLING_TORQUE:
                if (motor_device_set_torque(false) == MOTOR_DEVICE_OK) {
                    torque_enabled = false;
                    stop.phase = SAFE_STOP_COMPLETE;
                    consecutive_failures = 0U;
                    host_protocol_complete_safe_stop();
                } else if (++stop.consecutive_failures >= ACTUATOR_FAILURE_LIMIT) {
                    stop_attempt_failed(&stop);
                }
                break;
            case SAFE_STOP_TORQUE_OFF_ONLY:
                if (motor_device_set_torque(false) == MOTOR_DEVICE_OK) {
                    torque_enabled = false;
                    stop.phase = SAFE_STOP_COMPLETE;
                    consecutive_failures = 0U;
                    host_protocol_complete_safe_stop();
                }
                break;
            case SAFE_STOP_WAITING_FOR_FAULT:
            case SAFE_STOP_COMPLETE:
            case SAFE_STOP_IDLE:
                break;
            }
        } else if (!host_protocol_is_armed()) {
            stop = (safe_stop_context_t){ 0 };
            if (torque_enabled) {
                if (motor_device_set_torque(false) == MOTOR_DEVICE_OK) torque_enabled = false;
            }
            target_valid = false;
            (void) xQueueReset(target_queue);
        } else {
            stop = (safe_stop_context_t){ 0 };
            queued_target_t received_targets;
            if (xQueueReceive(target_queue, &received_targets, 0U) == pdPASS) {
                active_targets = received_targets.targets;
                target_valid = true;
                last_target_tick = received_targets.accepted_tick;
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
