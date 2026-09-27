#ifndef ACTUATOR_TASK_H
#define ACTUATOR_TASK_H

#include <stdbool.h>

#include "motor_device.h"

bool actuator_task_init(void);
bool actuator_submit_targets(const motor_target_set_t *targets);
void actuator_task(void *argument);

#endif /* ACTUATOR_TASK_H */
