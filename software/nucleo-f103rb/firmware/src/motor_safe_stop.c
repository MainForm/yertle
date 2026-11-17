#include "motor_safe_stop.h"
#include "sensor_task.h"

#define RETURN_PERIOD_MS 20U
#define RETURN_STALL_MS 500U
#define RETURN_TIMEOUT_MS 15000U
#define RETURN_MIN_PROGRESS 12U
#define RETURN_SETTLED_SAMPLES 3U
#define RETURN_FEEDBACK_FAILURE_LIMIT 3U

void motor_safe_stop_begin(motor_safe_stop_t *stop, uint32_t now, bool return_eligible)
{
    *stop = (motor_safe_stop_t){
        .outcome = return_eligible ? MOTOR_STOP_RETURNING : MOTOR_STOP_INELIGIBLE,
        .started_ms = now,
        .next_sample_ms = now,
        .unknown_torque = YERTLE_ALL_MOTORS_MASK
    };
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        stop->best_error[i] = UINT16_MAX;
        stop->last_progress_ms[i] = now;
    }
}

static bool read_stop_feedback(
    const motor_safe_stop_t *stop, uint32_t now, uint32_t max_feedback_age_ms,
    motor_feedback_set_t *feedback, int16_t position[YERTLE_MOTOR_COUNT], bool *servo_fault)
{
    imu_feedback_t imu;
    if (!sensor_get_feedback_snapshot(feedback, &imu) || !feedback->sequence ||
        feedback->sequence == stop->last_sequence ||
        feedback->valid_mask != YERTLE_ALL_MOTORS_MASK ||
        (max_feedback_age_ms && now - feedback->uptime_ms > max_feedback_age_ms)) {
        return false;
    }
    *servo_fault = false;
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        const motor_feedback_t *motor = &feedback->motor[i];
        if (!motor->valid || motor->position < 0 || motor->position > 4095) return false;
        if (motor->servo_error) *servo_fault = true;
        position[i] = motor->position;
    }
    return true;
}

static motor_stop_outcome_t check_return_progress(
    motor_safe_stop_t *stop, uint32_t now, uint16_t driven_mask,
    const int16_t position[YERTLE_MOTOR_COUNT])
{
    stop->max_error = 0U;
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        const int error = position[i] - MOTOR_STOP_CENTER;
        const uint16_t magnitude = (uint16_t)(error < 0 ? -error : error);
        if (magnitude > stop->max_error) stop->max_error = magnitude;
        if (magnitude <= MOTOR_STOP_TOLERANCE ||
            magnitude + RETURN_MIN_PROGRESS <= stop->best_error[i]) {
            stop->best_error[i] = magnitude;
            stop->last_progress_ms[i] = now;
        }
        if (!(driven_mask & (1U << i)) && magnitude > MOTOR_STOP_TOLERANCE) return MOTOR_STOP_INELIGIBLE;
        if (now - stop->last_progress_ms[i] >= RETURN_STALL_MS) return MOTOR_STOP_STALLED;
    }
    stop->settled = stop->max_error <= MOTOR_STOP_TOLERANCE ? (uint8_t)(stop->settled + 1U) : 0U;
    return stop->settled >= RETURN_SETTLED_SAMPLES ? MOTOR_STOP_CENTERED : MOTOR_STOP_RETURNING;
}

static motor_stop_outcome_t service_return(
    motor_safe_stop_t *stop, uint32_t now, uint16_t driven_mask,
    const motor_safe_stop_ops_t *ops, const void *mode)
{
    if (now - stop->started_ms >= RETURN_TIMEOUT_MS) return MOTOR_STOP_TIMEOUT;
    if ((int32_t)(now - stop->next_sample_ms) < 0) return MOTOR_STOP_RETURNING;
    stop->next_sample_ms = now + RETURN_PERIOD_MS;

    motor_feedback_set_t feedback;
    int16_t position[YERTLE_MOTOR_COUNT];
    bool servo_fault = false;
    const bool complete = read_stop_feedback(stop, now, ops->max_feedback_age_ms, &feedback,
                                             position, &servo_fault);
    if (complete && servo_fault) return MOTOR_STOP_FEEDBACK;

    motor_stop_outcome_t sample = MOTOR_STOP_FEEDBACK;
    if (complete) {
        sample = ops->prepare_feedback ? ops->prepare_feedback(mode, &feedback, position) : MOTOR_STOP_NONE;
    }
    const uint32_t dt_ms = sample == MOTOR_STOP_NONE && stop->last_sequence
        ? feedback.uptime_ms - stop->last_stamp : RETURN_PERIOD_MS;
    if (sample == MOTOR_STOP_FEEDBACK || !dt_ms || (dt_ms & 0x80000000UL)) {
        stop->settled = 0U;
        if (!stop->feedback_paused && ops->pause_return && !ops->pause_return(mode)) return MOTOR_STOP_TX;
        stop->feedback_paused = true;
        return ++stop->feedback_failures >= RETURN_FEEDBACK_FAILURE_LIMIT
            ? MOTOR_STOP_FEEDBACK : MOTOR_STOP_RETURNING;
    }
    if (sample != MOTOR_STOP_NONE) return sample;

    stop->feedback_paused = false;
    stop->feedback_failures = 0U;
    stop->last_sequence = feedback.sequence;
    stop->last_stamp = feedback.uptime_ms;
    const motor_stop_outcome_t progress = check_return_progress(stop, now, driven_mask, position);
    if (progress != MOTOR_STOP_RETURNING) return progress;

    const motor_stop_outcome_t sent = ops->send_return(mode, position, dt_ms);
    if (sent == MOTOR_STOP_NONE) {
        stop->send_failures = 0U;
    } else if (sent != MOTOR_STOP_TX || ++stop->send_failures >= ops->return_tx_failure_limit) {
        return sent;
    }
    return MOTOR_STOP_RETURNING;
}

void motor_safe_stop_service(motor_safe_stop_t *stop, uint32_t now, uint16_t driven_mask,
                             const motor_safe_stop_ops_t *ops, const void *mode)
{
    if (stop->outcome == MOTOR_STOP_NONE || stop->off_confirmed) return;
    if (stop->outcome == MOTOR_STOP_RETURNING) {
        stop->outcome = service_return(stop, now, driven_mask, ops, mode);
        if (stop->outcome == MOTOR_STOP_RETURNING) return;
    }

    /* PWM zero can enable torque. Send it once, then only retry torque OFF. */
    if (!stop->off_started) {
        stop->off_started = true;
        if (ops->before_off) ops->before_off(mode);
    }
    stop->unknown_torque = YERTLE_ALL_MOTORS_MASK;
    stop->off_confirmed = motor_device_set_torque(false) == MOTOR_DEVICE_OK &&
                         motor_device_verify_torque_off(&stop->unknown_torque);
}
