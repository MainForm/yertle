/* Legacy task tests isolate the unchanged position path. Production PWM state
 * machine and its host integration are tested separately in tests/pwm. */
#include "pwm_control.h"
void pwm_control_init(void){}
bool pwm_control_service(void){return false;}
bool pwm_control_native_ready(void){return true;}
bool pwm_control_accepts_positions(void){return true;}
bool pwm_control_can_arm(void){return true;}
void pwm_control_connection_reset(void){}
void pwm_control_stop(uint8_t reason){(void)reason;}
uint8_t pwm_control_request_mode(const uint8_t *b,size_t n,uint16_t s){(void)b;(void)n;(void)s;return YPWM_INVALID_STATE;}
uint8_t pwm_control_release(const uint8_t *b,size_t n){(void)b;(void)n;return YPWM_INVALID_STATE;}
bool pwm_control_submit_positions(const motor_target_set_t *g){(void)g;return false;}
bool pwm_control_take_mode_result(uint16_t *s,uint8_t *r){(void)s;(void)r;return false;}
bool pwm_control_status(uint8_t b[YPWM_STATUS_LENGTH],bool force){(void)b;(void)force;return false;}
bool pwm_control_mode_diagnostics(uint8_t d[YPWM_DIAG_LENGTH],uint8_t w[64],bool *has,bool force){(void)d;(void)w;(void)has;(void)force;return false;}
