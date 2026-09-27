#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>

#include "motor_device.h"

bool sensor_task_init(void);
bool sensor_get_motor_feedback(motor_feedback_set_t *feedback);
void sensor_task(void *argument);

#endif /* SENSOR_TASK_H */
