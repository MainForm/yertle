#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>

#include "motor_device.h"
#include "imu_device.h"
#include "pmw3901_device.h"

typedef struct {
    uint32_t sequence;
    uint32_t uptime_ms;
    bool ready;
    bool valid;
    imu_raw_sample_t raw;
} imu_feedback_t;

/* Owned by main(), then used by sensor_task for the life of the scheduler. */
typedef struct {
    imu_device_t imu;
    pmw3901_device_t optical_flow;
} sensor_devices_t;

/* Cache both initial sensor states before starting the scheduler. */
bool sensor_task_init(const sensor_devices_t *devices);
bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu, pmw3901_feedback_t *flow);
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu);
void sensor_task(void *argument);

#endif /* SENSOR_TASK_H */
