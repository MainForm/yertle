#ifndef YERTLE_MOTOR_CONTROL_H
#define YERTLE_MOTOR_CONTROL_H

#include "motor_device.h"

#define MOTOR_CONTROL_TARGET_TIMEOUT_MS 500U

void motor_control_init(void);
bool motor_control_can_arm(void);
void motor_control_on_armed(void);
void motor_control_connection_reset(void);
void motor_control_stop(uint8_t reason);
/* Serialize a disarmed 0x21 write with the actuator/ARM gate. */
bool motor_control_begin_mode_write(uint8_t id);
void motor_control_finish_mode_write(const motor_register_write_report_t *report);
/* Position-only operations such as reading servo position offsets. */
bool motor_control_position_ready(void);
/* Common acceptance gate and latest position target for both motor modes.
 * Submission copies targets and timestamps reception without motor I/O. */
bool motor_control_accepts_targets(void);
bool motor_control_submit_targets(const motor_target_set_t *targets);

/* Called by the actuator task on every tick, including without a new target. */
void motor_control_service(void);

#endif /* YERTLE_MOTOR_CONTROL_H */
