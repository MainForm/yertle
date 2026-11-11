#ifndef YERTLE_MOTOR_CONTROL_STATE_H
#define YERTLE_MOTOR_CONTROL_STATE_H

#include "motor_device.h"

typedef struct {
    motor_target_set_t targets;
    uint32_t accepted_ms;
} motor_position_target_t;

/* Native-position execution shares the command slot with PWM position PID. */
bool motor_control_read_targets(motor_position_target_t *target);
void motor_control_discard_targets(void);

#endif /* YERTLE_MOTOR_CONTROL_STATE_H */
