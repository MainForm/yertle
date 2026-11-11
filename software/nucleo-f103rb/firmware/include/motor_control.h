#ifndef YERTLE_MOTOR_CONTROL_H
#define YERTLE_MOTOR_CONTROL_H

#include "motor_device.h"

void motor_control_init(void);
/* Common acceptance gate and latest position target for both motor modes.
 * Submission copies targets and timestamps reception without motor I/O. */
bool motor_control_accepts_targets(void);
bool motor_control_submit_targets(const motor_target_set_t *targets);

/* Called by the actuator task on every tick, including without a new target. */
void motor_control_service(void);

#endif /* YERTLE_MOTOR_CONTROL_H */
