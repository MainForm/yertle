#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>

#include "motor_device.h"
#include "imu_device.h"

typedef struct {
    uint32_t sequence;
    uint32_t uptime_ms;
    bool valid;
    imu_raw_sample_t raw;
} imu_feedback_t;

bool sensor_task_init(void);
bool sensor_get_motor_feedback(motor_feedback_set_t *feedback);
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu);
void sensor_task(void *argument);

#endif /* SENSOR_TASK_H */
