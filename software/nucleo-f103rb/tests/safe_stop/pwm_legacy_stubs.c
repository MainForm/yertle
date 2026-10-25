/* Legacy task tests isolate the unchanged position path. Production PWM state
 * machine and its host integration are tested separately in tests/pwm. */
#include "pwm_control.h"
void pwm_control_init(void){}
bool pwm_control_service(void){return false;}
bool pwm_control_native_ready(void){return true;}
bool pwm_control_accepts_positions(void){return true;}
bool pwm_control_can_arm(void){return true;}
void pwm_control_on_armed(void){}
void pwm_control_connection_reset(void){}
void pwm_control_stop(uint8_t reason){(void)reason;}
uint8_t pwm_control_request_mode(const uint8_t *b,size_t n,uint16_t s){(void)b;(void)n;(void)s;return YPWM_INVALID_STATE;}
bool pwm_control_mode_write_allowed(uint8_t id,uint8_t mode){(void)id;(void)mode;return false;}
bool pwm_control_begin_mode_write(const motor_register_write_request_t *q){(void)q;return false;}
void pwm_control_end_mode_write(const motor_register_write_report_t *r){(void)r;}
uint32_t pwm_control_staged_epoch(void){return 0;}
bool pwm_control_preparation_pending(void){return false;}
motor_register_write_report_t motor_device_write_mode(uint8_t id,uint8_t mode,bool (*allowed)(void)){
 (void)id;(void)mode;(void)allowed;motor_register_write_report_t r=motor_register_initial_report();r.status=MOTOR_WRITE_INVALID_STATE;return r;
}
bool pwm_control_submit_positions(const motor_target_set_t *g){(void)g;return false;}
uint8_t pwm_control_submit_raw(const int16_t d[12],uint16_t s){(void)d;(void)s;return YPWM_PWM_MODE_REQUIRED;}
bool pwm_control_take_raw_result(uint16_t *s,uint8_t *r){(void)s;(void)r;return false;}
bool pwm_control_take_mode_result(uint16_t *s,uint8_t *r){(void)s;(void)r;return false;}
bool pwm_control_status(uint8_t b[YPWM_STATUS_LENGTH],bool force){(void)b;(void)force;return false;}
bool pwm_control_mode_diagnostics(uint8_t d[YPWM_DIAG_LENGTH],uint8_t w[64],bool *has,bool force){(void)d;(void)w;(void)has;(void)force;return false;}
