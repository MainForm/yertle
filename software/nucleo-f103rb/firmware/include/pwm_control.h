#ifndef YERTLE_PWM_CONTROL_H
#define YERTLE_PWM_CONTROL_H
#include "pwm_contract.h"
#include "motor_device.h"
#include "pwm_mode_diagnostics.h"
/* Host functions only copy bounded state under a critical section; no motor I/O.
 * service() is exclusively called by actuator_task and owns all PWM writes. */
void pwm_control_init(void);
bool pwm_control_service(void);
bool pwm_control_native_ready(void);
bool pwm_control_accepts_positions(void);
bool pwm_control_can_arm(void);
void pwm_control_on_armed(void);
void pwm_control_connection_reset(void);
void pwm_control_stop(uint8_t reason);
uint8_t pwm_control_request_mode(const uint8_t *p,size_t n,uint16_t sequence);
bool pwm_control_mode_write_allowed(uint8_t id,uint8_t mode);
bool pwm_control_begin_mode_write(const motor_register_write_request_t *request);
void pwm_control_end_mode_write(const motor_register_write_report_t *report);
uint32_t pwm_control_staged_epoch(void);
bool pwm_control_preparation_pending(void);
bool pwm_control_submit_positions(const motor_target_set_t *goal);
/* Signed host duties, ID1..12. ACK is deferred until actuator TX completes.
 * Only all-zero diagnostics may run DISARMED; their ACK does not prove
 * torque OFF (PWM0 can activate servo torque without a global ARM). */
uint8_t pwm_control_submit_raw(const int16_t duty[12],uint16_t sequence);
bool pwm_control_take_raw_result(uint16_t *sequence,uint8_t *result);
bool pwm_control_take_mode_result(uint16_t *sequence,uint8_t *result);
bool pwm_control_status(uint8_t bytes[YPWM_STATUS_LENGTH],bool force);
bool pwm_control_mode_diagnostics(uint8_t detail[YPWM_DIAG_LENGTH],uint8_t write[64],bool *has_write,bool force);
#endif
