#ifndef YERTLE_PWM_CONTROL_H
#define YERTLE_PWM_CONTROL_H
#include "pwm_contract.h"
/* Host functions only copy bounded state under a critical section; no motor I/O.
 * service() is exclusively called by actuator_task and owns all PWM writes. */
void pwm_control_init(void);
bool pwm_control_service(void);
bool pwm_control_native_ready(void);
bool pwm_control_can_arm(void);
void pwm_control_connection_reset(void);
void pwm_control_stop(uint8_t reason);
uint8_t pwm_control_request_mode(const uint8_t *p,size_t n,uint16_t sequence);
uint8_t pwm_control_submit(const uint8_t *p,size_t n);
bool pwm_control_take_mode_result(uint16_t *sequence,uint8_t *result);
bool pwm_control_status(uint8_t bytes[YPWM_STATUS_LENGTH],bool force);
#endif
