#undef NDEBUG
/* Reuse the bounded UART/diagnostic mocks, not PWM stubs. Both production host
 * protocol and production PWM service are linked in this target. */
#define main legacy_heartbeat_main
#include "../diagnostics/heartbeat_test.c"
#undef main
static unsigned critical_depth, pwm_writes, offs;
void pwm_test_enter(void){++critical_depth;}
void pwm_test_exit(void){assert(critical_depth);--critical_depth;}
motor_device_status_t motor_device_set_torque(bool enabled){assert(!critical_depth&&!enabled);++offs;return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool enabled){assert(!critical_depth&&mask&&!enabled);++offs;return MOTOR_DEVICE_OK;}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical_depth);*unknown=0;return true;}
bool motor_device_verify_modes(uint8_t backend,uint16_t mask,uint16_t *verified){(void)backend;(void)mask;assert(!critical_depth);*verified=YPWM_ALL_MASK;return true;}
motor_register_write_report_t motor_device_write_mode(uint8_t id,uint8_t mode,bool (*allowed)(void)){
 assert(!critical_depth&&id>=1&&id<=12&&(mode==0||mode==2)&&allowed());
 motor_register_write_report_t r=motor_register_initial_report();r.status=MOTOR_WRITE_OK;return r;
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){
 assert(!critical_depth&&mask);for(unsigned i=0;i<12;++i)assert(values[i]==0);++pwm_writes;return MOTOR_DEVICE_OK;
}
static void receive_payload(protocol_context_t *c,uint8_t type,const uint8_t *p,uint16_t n){
 uint8_t raw[YERTLE_PROTOCOL_MAX_RAW]={1,type,0,42,0};raw[5]=(uint8_t)n;memcpy(raw+7,p,n);
 const uint16_t crc=crc16_ccitt(raw,7+n);ypwm_put16(raw+7+n,crc);handle_frame(c,raw,9+n);
}
int main(void){
 protocol_context_t c=reset_context(true,false);pwm_control_init();assert(pwm_control_service());assert(pwm_control_native_ready());
 uint8_t mode[8]={1,1,8,0};ypwm_put32(mode+4,123);
 c.offsets_job_active=true;receive_payload(&c,MESSAGE_SET_CONTROL_MODE,mode,8);assert(!c.pwm_mode_pending);
 c.offsets_job_active=false;receive_payload(&c,MESSAGE_SET_CONTROL_MODE,mode,8);assert(c.pwm_mode_pending&&!c.lease_established);
 receive_request(&c,MESSAGE_HEARTBEAT,true);try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED);
 assert(pwm_control_service());service_pwm(&c);assert(!c.pwm_mode_pending&&!c.lease_established);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED);
 receive_request(&c,MESSAGE_HEARTBEAT,true);try_enter_control_ready(&c);assert(c.state==HOST_FSM_ARMED);
 uint8_t targets[60]={0};receive_payload(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(!pwm_control_native_ready());
 uint8_t release[YPWM_TARGET_LENGTH]={0};release[12]=YPWM_RELEASE;
 receive_payload(&c,MESSAGE_SET_MOTOR_PWM,release,sizeof release);service_safety_state(&c);
 assert(c.state==HOST_FSM_DISARMED&&host_protocol_is_stop_requested());
 assert(pwm_control_service());service_safety_state(&c);assert(!host_protocol_is_stop_requested());
 assert(!can_arm(&c)&&pwm_writes==2&&offs>=3);
 /* A canceled preparation clears the asynchronous pending latch. */
 ypwm_put32(mode+4,124);receive_payload(&c,MESSAGE_SET_CONTROL_MODE,mode,8);assert(c.pwm_mode_pending);
 tick=1000;expire_host_link(&c);service_safety_state(&c);service_pwm(&c);
 assert(!c.pwm_mode_pending&&!c.host_connected);
 assert(pwm_control_service());service_safety_state(&c);
 /* Explicit torque-OFF native restoration requires fresh heartbeat again. */
 receive_request(&c,MESSAGE_HELLO,true);mode[1]=0;ypwm_put16(mode+2,YPWM_ALL_MASK);ypwm_put32(mode+4,125);
 receive_payload(&c,MESSAGE_SET_CONTROL_MODE,mode,8);assert(c.pwm_mode_pending);assert(pwm_control_service());service_pwm(&c);
 assert(pwm_control_native_ready()&&!can_arm(&c));receive_request(&c,MESSAGE_HEARTBEAT,true);try_enter_control_ready(&c);
 assert(c.state==HOST_FSM_ARMED);puts("Production host + PWM: lease order, cancel, target separation, native restoration passed");return 0;
}
