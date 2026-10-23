#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "pwm_control.h"
#include "motor_device.h"
#include "host_protocol.h"
#include "sensor_task.h"

static uint32_t tick;
static unsigned critical,on_count,off_count,write_count,completed;
static bool stop_requested,armed,off_ok,mode_ok,feedback_ok,write_ok,cancel_mode,cancel_on;
static uint16_t on_mask;
static int16_t sent[12];
static motor_feedback_set_t feedback;
void pwm_test_enter(void){++critical;}
void pwm_test_exit(void){assert(critical);--critical;}
uint32_t xTaskGetTickCount(void){return tick;}
host_fsm_state_t host_protocol_get_state(void){return armed?HOST_FSM_ARMED:HOST_FSM_DISARMED;}
bool host_protocol_is_armed(void){return armed;}
bool host_protocol_is_stop_requested(void){return stop_requested;}
void host_protocol_request_pwm_stop(void){stop_requested=true;armed=false;}
void host_protocol_complete_safe_stop(void){++completed;stop_requested=false;}
void host_protocol_latch_unsafe(host_unsafe_reason_t r){assert(r==HOST_UNSAFE_CURRENT_LIMIT);}
motor_device_status_t motor_device_set_torque(bool on){assert(!critical&&!on);++off_count;return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool on){assert(!critical);
 if(on){++on_count;on_mask=mask;if(cancel_on)pwm_control_stop(YPWM_REASON_RELEASE);}
 else ++off_count;
 return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;
}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical);*unknown=off_ok?0:YPWM_ALL_MASK;return off_ok;}
bool motor_device_verify_modes(uint8_t backend,uint16_t mask,uint16_t *verified){assert(!critical);(void)backend;(void)mask;*verified=mode_ok?YPWM_ALL_MASK:0;return mode_ok;}
motor_register_write_report_t motor_device_write_mode(uint8_t id,uint8_t mode,bool (*allowed)(void)){
 assert(!critical&&id>=1&&id<=12&&(mode==0||mode==2));
 if(cancel_mode)pwm_control_connection_reset();
 motor_register_write_report_t r=motor_register_initial_report();r.status=mode_ok&&allowed()?MOTOR_WRITE_OK:MOTOR_WRITE_INVALID_STATE;return r;
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){
 assert(!critical&&mask);++write_count;memcpy(sent,values,sizeof sent);return write_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;
}
bool sensor_get_feedback_snapshot(motor_feedback_set_t *f,imu_feedback_t *imu){assert(!critical);*f=feedback;memset(imu,0,sizeof *imu);return feedback_ok;}
static uint8_t phase(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[3];}
static uint8_t reason(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[7];}
static void reset(void){
 tick=100;critical=on_count=off_count=write_count=completed=0;on_mask=0;stop_requested=armed=false;
 off_ok=mode_ok=feedback_ok=write_ok=true;cancel_mode=cancel_on=false;memset(sent,0,sizeof sent);
 memset(&feedback,0,sizeof feedback);feedback.valid_mask=YPWM_ALL_MASK;feedback.uptime_ms=tick;
 for(unsigned i=0;i<12;++i){feedback.motor[i].position=2048;feedback.motor[i].valid=true;feedback.motor[i].voltage=60;}
 pwm_control_init();assert(!pwm_control_can_arm());assert(pwm_control_service());assert(pwm_control_native_ready());
 assert(!pwm_control_service());
}
static uint8_t prepare(uint32_t epoch){uint8_t b[8]={1,1,8,0};ypwm_put32(b+4,epoch);return pwm_control_request_mode(b,sizeof b,42);}
static uint8_t target(uint32_t epoch,uint32_t command,int16_t duty){
 uint8_t b[YPWM_TARGET_LENGTH]={0};ypwm_put32(b,epoch);ypwm_put32(b+4,command);ypwm_put16(b+8,8);
 ypwm_put16(b+10,100);ypwm_put16(b+13+6,(uint16_t)duty);return pwm_control_submit(b,sizeof b);
}
static void ready(void){assert(prepare(123)==0);assert(pwm_control_service());assert(phase()==YPWM_READY);
 assert(!pwm_control_can_arm());uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result));assert(seq==42&&result==0);
 assert(pwm_control_can_arm());armed=true;assert(target(123,1,100)==0);assert(pwm_control_service());assert(phase()==YPWM_ACTIVE);
 assert(on_count==1&&on_mask==8&&sent[3]==20);
}
static void stopped(uint8_t why){assert(phase()==YPWM_STOPPING&&reason()==why);assert(pwm_control_service());assert(phase()==YPWM_FAULT);assert(completed==1&&sent[3]==0&&!pwm_control_can_arm());}
int main(void){
 reset();ready();assert(target(122,2,100)==YPWM_STALE_EPOCH);assert(target(123,1,100)==YPWM_OLD_COMMAND);
 tick+=10;feedback.uptime_ms=tick;assert(target(123,2,-100)==0);assert(pwm_control_service());assert(sent[3]==0);
 tick+=100;assert(pwm_control_service());stopped(YPWM_REASON_TTL);assert(prepare(123)==YPWM_STALE_EPOCH);
 reset();ready();feedback.motor[3].position+=33;tick+=10;assert(pwm_control_service());stopped(YPWM_REASON_TRAVEL);
 reset();ready();feedback.motor[3].current=308;tick+=10;assert(pwm_control_service());stopped(YPWM_REASON_CURRENT);
 reset();ready();feedback.valid_mask=0;tick+=10;assert(pwm_control_service());stopped(YPWM_REASON_SENSOR);
 reset();ready();feedback.motor[3].speed=501;tick+=10;assert(pwm_control_service());stopped(YPWM_REASON_SPEED);
 reset();ready();feedback.motor[3].temperature=60;tick+=10;assert(pwm_control_service());stopped(YPWM_REASON_THERMAL_VOLTAGE);
 reset();ready();tick+=31;assert(pwm_control_service());stopped(YPWM_REASON_DEADLINE);
 reset();ready();tick+=10;write_ok=false;assert(pwm_control_service());stopped(YPWM_REASON_DRIVER);
 reset();ready();uint8_t release[YPWM_TARGET_LENGTH]={0};release[12]=YPWM_RELEASE;
 assert(pwm_control_submit(release,sizeof release)==0);off_ok=false;assert(pwm_control_service());assert(phase()==YPWM_STOPPING);
 uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));assert(ypwm_u16(b+18)==YPWM_ALL_MASK);
 off_ok=true;stopped(YPWM_REASON_RELEASE);
 reset();assert(prepare(123)==0);cancel_mode=true;assert(pwm_control_service());assert(phase()==YPWM_STOPPING);
 uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result)&&result!=0&&seq==42);stopped(YPWM_REASON_LINK);
 reset();assert(prepare(123)==0);assert(pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));armed=true;
 assert(target(123,1,100)==0);cancel_on=true;assert(pwm_control_service());assert(sent[3]==0);stopped(YPWM_REASON_RELEASE);
 reset();ready();for(unsigned i=0;i<100&&phase()==YPWM_ACTIVE;++i){tick+=10;feedback.uptime_ms=tick;assert(target(123,i+2,100)==0);assert(pwm_control_service());}
 stopped(YPWM_REASON_SESSION);
 reset();mode_ok=false;pwm_control_init();assert(pwm_control_service());assert(phase()==YPWM_FAULT&&!pwm_control_can_arm());
 reset();assert(prepare(123)==0);assert(pwm_control_service());pwm_control_stop(YPWM_REASON_RELEASE);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_INVALID_STATE);stopped(YPWM_REASON_RELEASE);
 reset();assert(pwm_control_submit(release,sizeof release)==0);assert(pwm_control_service());
 assert(phase()==YPWM_FAULT&&write_count==0&&on_count==0); /* native 0x2c must not receive goal0 */
 puts("PWM service: guards, cancellation, no I/O under critical sections, verified stop passed");return 0;
}
