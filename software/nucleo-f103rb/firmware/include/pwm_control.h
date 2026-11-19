#ifndef YERTLE_PWM_CONTROL_H
#define YERTLE_PWM_CONTROL_H
#include "pwm_contract.h"
#include "motor_device.h"
/* Signed host duties, ID1..12. ACK is deferred until actuator TX completes.
 * Only all-zero diagnostics may run DISARMED; their ACK does not prove
 * torque OFF (PWM0 can activate servo torque without a global ARM). */
uint8_t pwm_control_submit_raw(const int16_t duty[12],uint16_t sequence);
bool pwm_control_take_raw_result(uint16_t *sequence,uint8_t *result);
bool pwm_control_status(uint8_t bytes[YPWM_STATUS_LENGTH],bool force);
/* Host-owned consumer, one low-priority frame per host pass. No bus reads. */
bool pwm_control_pid_detail(uint8_t bytes[YPWM_PID_DETAIL_LENGTH]);
void pwm_control_pid_detail_result(bool sent);
bool pwm_control_stop_detail(uint8_t bytes[20]);
bool pwm_control_pid_stats(uint8_t bytes[YPWM_PID_STATS_LENGTH]);
#endif
