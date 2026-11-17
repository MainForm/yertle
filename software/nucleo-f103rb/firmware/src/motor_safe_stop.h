#ifndef YERTLE_MOTOR_SAFE_STOP_H
#define YERTLE_MOTOR_SAFE_STOP_H

#include "motor_device.h"

#define MOTOR_STOP_CENTER 2048
#define MOTOR_STOP_TOLERANCE 16
#define MOTOR_CONTROL_TX_FAILURE_LIMIT 3U

/* Outcome values 1–10 are part of the PWM stop-detail event. */
typedef enum {
    MOTOR_STOP_NONE = 0,
    MOTOR_STOP_RETURNING = 1,
    MOTOR_STOP_CENTERED = 2,
    MOTOR_STOP_INELIGIBLE = 3,
    MOTOR_STOP_FEEDBACK = 4,
    MOTOR_STOP_VOLTAGE = 5,
    MOTOR_STOP_TEMPERATURE = 6,
    MOTOR_STOP_STALLED = 7,
    MOTOR_STOP_TIMEOUT = 8,
    MOTOR_STOP_PID = 9,
    MOTOR_STOP_TX = 10
} motor_stop_outcome_t;

typedef struct {
    motor_stop_outcome_t outcome;
    uint32_t started_ms, next_sample_ms, last_sequence, last_stamp;
    uint32_t last_progress_ms[YERTLE_MOTOR_COUNT];
    uint16_t best_error[YERTLE_MOTOR_COUNT];
    uint16_t max_error, unknown_torque;
    uint8_t settled, feedback_failures, send_failures;
    bool feedback_paused, off_started, off_confirmed;
} motor_safe_stop_t;

typedef enum {
    MOTOR_FEEDBACK_VALID,
    MOTOR_FEEDBACK_INCOMPLETE,
    MOTOR_FEEDBACK_SERVO_ERROR
} motor_feedback_status_t;

motor_feedback_status_t motor_feedback_status(const motor_feedback_set_t *feedback);

typedef struct {
    /* Return NONE on success, FEEDBACK for a retryable unusable sample. */
    motor_stop_outcome_t (*prepare_feedback)(const void *mode, const motor_feedback_set_t *feedback,
                                           int16_t position[YERTLE_MOTOR_COUNT]);
    motor_stop_outcome_t (*send_return)(const void *mode, const int16_t position[YERTLE_MOTOR_COUNT],
                                      uint32_t dt_ms);
    bool (*pause_return)(const void *mode);
    void (*before_off)(const void *mode);
} motor_safe_stop_ops_t;

void motor_safe_stop_begin(motor_safe_stop_t *stop, uint32_t now, bool return_eligible);
void motor_safe_stop_service(motor_safe_stop_t *stop, uint32_t now, uint16_t driven_mask,
                             const motor_safe_stop_ops_t *ops, const void *mode);

#endif
