/* Host-protocol tests mock dependencies outside the protocol under test. */
#include "pwm_control.h"
#include "motor_control.h"
/* Protocol-focused tests use stubs; actuator tests link the actual control FSM. */
#ifndef NATIVE_POSITION_TEST
bool motor_control_position_ready(void){return true;}
bool motor_control_accepts_targets(void){return true;}
bool motor_control_can_arm(void){return true;}
void motor_control_on_armed(void){}
void motor_control_connection_reset(void){}
bool motor_control_begin_mode_write(uint8_t id){(void)id;return false;}
void motor_control_finish_mode_write(const motor_register_write_report_t *r){(void)r;}
uint8_t pwm_control_submit_raw(const int16_t d[12],uint16_t s){(void)d;(void)s;return YPWM_PWM_MODE_REQUIRED;}
bool pwm_control_take_raw_result(uint16_t *s,uint8_t *r){(void)s;(void)r;return false;}
bool pwm_control_status(uint8_t b[YPWM_STATUS_LENGTH],bool force){(void)b;(void)force;return false;}

bool pwm_control_pid_detail(uint8_t b[YPWM_PID_DETAIL_LENGTH]){(void)b;return false;}
void pwm_control_pid_detail_result(bool sent){(void)sent;}
bool pwm_control_pid_stats(uint8_t b[YPWM_PID_STATS_LENGTH]){(void)b;return false;}

bool pwm_control_stop_detail(uint8_t b[20]){(void)b;return false;}

#endif


#include "pmw3901_spi_diagnostic.h"
void pmw3901_spi_send_diagnostics(void*c,pmw_spi_diag_sender_t s,uint16_t q){(void)c;(void)s;(void)q;}

void pmw3901_spi_call_begin(uint32_t t){(void)t;}
bool pmw3901_spi_call_end(void){return true;}

#ifndef OPTICAL_HOST_TEST
bool pmw3901_spi_auto_payload(uint8_t p[64]){(void)p;return false;}
void pmw3901_spi_auto_result(bool sent){(void)sent;}

#endif /* OPTICAL_HOST_TEST */
