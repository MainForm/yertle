#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>

#include "motor_device.h"
#include "imu_device.h"

/* Total attempts per ID, including the initial read. */
#define SENSOR_POSITION_OFFSET_MAX_ATTEMPTS 5U

typedef struct {
    uint32_t sequence;
    uint32_t uptime_ms;
    bool valid;
    imu_raw_sample_t raw;
} imu_feedback_t;

typedef struct {
    uint16_t request_sequence;
    uint32_t uptime_ms;
    uint16_t valid_mask;
    motor_position_offset_t motor[YERTLE_MOTOR_COUNT];
    motor_device_status_t status[YERTLE_MOTOR_COUNT];
} motor_position_offsets_t;

typedef struct {
    uint16_t request_sequence;
    uint8_t motor_id, address, length;
    motor_register_status_t status;
    motor_register_data_t reading;
} motor_register_result_t;

bool sensor_task_init(void);
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu);
/* One in-flight read-only job; up to five attempts per ID, first valid reply
 * wins. Reject another job until the result is taken. */
bool sensor_request_position_offsets(uint16_t request_sequence);
bool sensor_take_position_offsets(motor_position_offsets_t *result);
/* Shares one diagnostic-job slot with offset queries. No UART I/O under the
 * snapshot mutex. A pending/completed result is never overwritten. */
bool sensor_request_motor_register(uint16_t sequence, uint8_t id, uint8_t address, uint8_t length);
bool sensor_take_motor_register(motor_register_result_t *result);
void sensor_task(void *argument);

#endif /* SENSOR_TASK_H */
