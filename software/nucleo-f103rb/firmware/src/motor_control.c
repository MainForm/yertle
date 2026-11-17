#include "motor_control.h"
#include "motor_control_state.h"
#include "motor_safe_stop.h"

#include "FreeRTOS.h"
#include "task.h"

#include "host_protocol.h"
#include "pwm_control.h"

#define ACTUATOR_FAILURE_LIMIT 3U
#define SAFE_STOP_SPEED_RAW 200U
#define SAFE_STOP_ACCELERATION_RAW 10U
#define SAFE_STOP_FEEDBACK_MAX_AGE_MS 150U

static motor_safe_stop_t stop;
static bool torque_enabled;
static uint8_t consecutive_failures;

static motor_stop_outcome_t send_position_return(
    const void *mode, const int16_t position[YERTLE_MOTOR_COUNT], uint32_t dt_ms)
{
    (void)mode;
    (void)position;
    (void)dt_ms;
    motor_target_set_t targets = { 0 };
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        targets.motor[i].position = MOTOR_STOP_CENTER;
        targets.motor[i].speed = SAFE_STOP_SPEED_RAW;
        targets.motor[i].acceleration = SAFE_STOP_ACCELERATION_RAW;
    }
    return motor_device_write_targets(&targets) == MOTOR_DEVICE_OK ? MOTOR_STOP_NONE : MOTOR_STOP_TX;
}

static const motor_safe_stop_ops_t position_stop_ops = {
    .send_return = send_position_return,
    .max_feedback_age_ms = SAFE_STOP_FEEDBACK_MAX_AGE_MS,
    .return_tx_failure_limit = ACTUATOR_FAILURE_LIMIT
};

void motor_control_init(void)
{
    pwm_control_init();
    stop = (motor_safe_stop_t){ 0 };
    torque_enabled = false;
    consecutive_failures = 0U;
}

static void service_position_stop(uint32_t now, bool stop_requested)
{
    motor_control_discard_targets();
    if (stop.outcome == MOTOR_STOP_NONE) {
        motor_safe_stop_begin(&stop, now, stop_requested && torque_enabled);
    }
    motor_safe_stop_service(&stop, now, YERTLE_ALL_MOTORS_MASK, &position_stop_ops, NULL);
    if (!stop.off_confirmed) return;

    torque_enabled = false;
    consecutive_failures = 0U;
    /* Completion is a flag consumed by the host, so repeated reports before
     * the host runs are safe. A subsequent stop still observes confirmed OFF. */
    if (stop_requested && !host_protocol_is_armed()) {
        host_protocol_complete_safe_stop();
    }
}

static void service_native_position(void)
{
    const bool stop_requested = host_protocol_get_state() == HOST_FSM_FAULT ||
                                host_protocol_is_stop_requested();
    const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (stop_requested || (!host_protocol_is_armed() && torque_enabled)) {
        service_position_stop(now, stop_requested);
        return;
    }
    if (stop.outcome != MOTOR_STOP_NONE) stop.outcome = MOTOR_STOP_NONE;
    if (!host_protocol_is_armed()) {
        motor_control_discard_targets();
        return;
    }

    motor_position_target_t target;
    if (!motor_control_read_targets(&target)) return;
    if (now - target.accepted_ms >= YPWM_TARGET_TIMEOUT_MS) {
        host_protocol_request_target_timeout();
        return;
    }

    if (host_protocol_is_armed() && !host_protocol_is_stop_requested()) {
        motor_device_status_t status = motor_device_write_targets(&target.targets);
        if (status == MOTOR_DEVICE_OK && !torque_enabled &&
            host_protocol_is_armed() && !host_protocol_is_stop_requested()) {
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

void motor_control_service(void)
{
    if (!pwm_control_service()) {
        service_native_position();
    } else {
        stop.outcome = MOTOR_STOP_NONE;
        torque_enabled = false;
        consecutive_failures = 0U;
    }
}
