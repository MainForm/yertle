#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "pwm_control.h"
#include "sensor_task.h"
#include "host_protocol.h"
static uint32_t tick;static unsigned critical,on_count,off_count,write_count,completed,unsafe_count;
static bool stop_requested,armed,off_ok,mode_ok,feedback_ok,write_ok,cancel_mode,cancel_on;
static uint16_t on_mask;static int16_t sent[12];static motor_feedback_set_t feedback;
static uint8_t mode_fail_id;static unsigned verify_off_calls,fail_off_call;static bool verify_modes_ok;
uint16_t motor_device_register_lock_uncertain_mask(void){return mode_fail_id?2U:0U;}
void pwm_test_enter(void){++critical;}void pwm_test_exit(void){assert(critical);--critical;}
uint32_t xTaskGetTickCount(void){return tick;}
host_fsm_state_t host_protocol_get_state(void){return armed?HOST_FSM_ARMED:HOST_FSM_DISARMED;}
bool host_protocol_is_armed(void){return armed;}bool host_protocol_is_stop_requested(void){return stop_requested;}
void host_protocol_request_pwm_stop(void){stop_requested=true;armed=false;}
void host_protocol_complete_safe_stop(void){++completed;stop_requested=false;}
void host_protocol_latch_unsafe(host_unsafe_reason_t r){assert(r==HOST_UNSAFE_CURRENT_LIMIT);++unsafe_count;}
bool motor_device_validate_targets(const motor_target_set_t *g){if(!g)return false;for(unsigned i=0;i<12;++i)if(g->motor[i].position>4095||g->motor[i].speed>32767)return false;return true;}
motor_device_status_t motor_device_set_torque(bool on){assert(!critical&&!on);++off_count;return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool on){assert(!critical);if(on){++on_count;on_mask=mask;if(cancel_on)pwm_control_stop(YPWM_REASON_RELEASE);}else ++off_count;return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical);*unknown=off_ok?0:YPWM_ALL_MASK;return off_ok;}
bool motor_device_verify_modes(uint8_t backend,uint16_t mask,uint16_t *verified){assert(!critical);(void)backend;(void)mask;*verified=mode_ok?YPWM_ALL_MASK:0;return mode_ok;}
bool motor_device_verify_torque_off_detailed(uint16_t *unknown,motor_verify_report_t *r){
 bool ok=motor_device_verify_torque_off(unknown);*r=motor_verify_initial_report();
 if(++verify_off_calls==fail_off_call){ok=false;*unknown=16;}
 if(!ok){r->motor_id=5;r->address=0x28;r->expected=0;r->actual=1;r->io_status=0;}return ok;
}
bool motor_device_verify_modes_detailed(uint8_t backend,uint16_t mask,uint16_t *verified,motor_verify_report_t *r){
 bool ok=motor_device_verify_modes(backend,mask,verified)&&verify_modes_ok;*r=motor_verify_initial_report();
 if(!ok){*verified=(uint16_t)(YPWM_ALL_MASK&~4U);r->motor_id=3;r->address=0x21;r->expected=2;r->actual=0;r->io_status=0;}return ok;
}
motor_register_write_report_t motor_device_write_mode(uint8_t id,uint8_t mode,bool (*allowed)(void)){
 assert(!critical&&id>=1&&id<=12&&(mode==0||mode==2));if(cancel_mode)pwm_control_connection_reset();
 motor_register_write_report_t r=motor_register_initial_report();r.status=mode_ok&&allowed()?MOTOR_WRITE_OK:MOTOR_WRITE_INVALID_STATE;
 if(id==mode_fail_id){r.status=MOTOR_WRITE_LOCK_RESTORE_FAILED;r.primary_status=MOTOR_WRITE_IO_ERROR;
  r.failure_stage=MOTOR_WRITE_STAGE_TARGET;r.target_write_attempted=r.lock_restore_attempted=true;
  r.lock_expected=1;r.lock_actual=0;r.lock_read_status=0;r.lock_write_status=2;r.lock_attempts=2;
  r.last_read_status=0;r.last_write_status=2;r.last_read_address=0x37;r.last_write_address=0x21;
  r.initial_value=0;r.elapsed_ms=75;r.lock_elapsed_ms=55;r.first_lock_failure=2;r.first_lock_write_status=2;}
 return r;
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){assert(!critical&&mask);++write_count;if(write_ok)memcpy(sent,values,sizeof sent);return write_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
bool sensor_get_feedback_snapshot(motor_feedback_set_t *f,imu_feedback_t *imu){assert(!critical);*f=feedback;memset(imu,0,sizeof *imu);return feedback_ok;}
static uint8_t phase(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[3];}
static uint8_t reason(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[7];}
static void reset(void){
 tick=100;critical=on_count=off_count=write_count=completed=unsafe_count=0;on_mask=0;stop_requested=armed=false;
 off_ok=mode_ok=feedback_ok=write_ok=true;cancel_mode=cancel_on=false;memset(sent,0,sizeof sent);
 mode_fail_id=0;verify_off_calls=fail_off_call=0;verify_modes_ok=true;
 memset(&feedback,0,sizeof feedback);feedback.valid_mask=YPWM_ALL_MASK;feedback.uptime_ms=tick;feedback.sequence=1;
 for(unsigned i=0;i<12;++i){feedback.motor[i].position=2048;feedback.motor[i].valid=true;feedback.motor[i].voltage=60;}
 pwm_control_init();assert(!pwm_control_can_arm());assert(pwm_control_service());assert(pwm_control_native_ready());assert(!pwm_control_service());
}
static uint8_t prepare(uint32_t epoch,uint16_t mask){uint8_t b[8]={YPWM_SCHEMA,YPWM_PID};ypwm_put16(b+2,mask);ypwm_put32(b+4,epoch);return pwm_control_request_mode(b,sizeof b,42);}
static motor_target_set_t goals(uint16_t mask){motor_target_set_t g={0};for(unsigned i=0;i<12;++i){g.motor[i].position=(uint16_t)((mask&(1U<<i))?2200:2048);g.motor[i].speed=200;}return g;}
static void ready(uint16_t mask){assert(prepare(123,mask)==0);assert(pwm_control_service());assert(phase()==YPWM_READY&&!pwm_control_can_arm());
 uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result)&&seq==42&&result==0);armed=true;
 motor_target_set_t g=goals(mask);assert(pwm_control_submit_positions(&g));assert(pwm_control_service());assert(phase()==YPWM_ACTIVE&&on_count==1&&on_mask==mask);
}
static void advance(void){tick+=20;feedback.uptime_ms=tick;++feedback.sequence;}
static void stopped(uint8_t why){assert(phase()==YPWM_STOPPING&&reason()==why);assert(pwm_control_service());assert(phase()==YPWM_FAULT&&completed==1&&!pwm_control_can_arm());}
static void current_is_host_owned(int16_t raw,bool all){
 reset();ready(8);
 if(all){for(unsigned i=0;i<12;++i)feedback.motor[i].current=raw;}
 else feedback.motor[3].current=raw;
 const unsigned writes=write_count;advance();assert(pwm_control_service());
 assert(phase()==YPWM_ACTIVE&&armed&&!stop_requested&&unsafe_count==0&&write_count==writes+1);
 for(unsigned i=0;i<12;++i)assert(feedback.motor[i].current==((all||i==3)?raw:0));
 /* A PC release still stops PWM and verifies torqueOFF after high telemetry. */
 const uint8_t release[1]={YPWM_SCHEMA};assert(pwm_control_release(release,1)==YPWM_OK);
 stopped(YPWM_REASON_RELEASE);assert(sent[3]==0&&off_count>0&&unsafe_count==0);
}
static void mode_diagnostics_test(unsigned fault,uint8_t expected_stage){
 reset();assert(prepare(123,YPWM_ALL_MASK)==0);
 if(fault==1)off_ok=false;
 if(fault==2)fail_off_call=1;
 if(fault==3){mode_fail_id=2;fail_off_call=2;} /* final-OFF failure must not overwrite primary */
 if(fault==4)verify_modes_ok=false;
 if(fault==5)write_ok=false;
 if(fault==6)fail_off_call=2;
 if(fault==7)cancel_mode=true;
 assert(pwm_control_service());uint8_t d[64],w[64],result;uint16_t seq;bool has=false;
 assert(pwm_control_take_mode_result(&seq,&result)&&seq==42);
 assert(result==(fault==0?YPWM_OK:fault==7?YPWM_INVALID_STATE:YPWM_MODE_FAILED));
 assert(pwm_control_mode_diagnostics(d,w,&has,false));
 assert(d[0]==5&&d[1]==1&&d[2]==1&&d[3]==expected_stage&&ypwm_u16(d+4)==42&&ypwm_u32(d+8)==123&&d[6]==result);
 assert(has==(fault==3||fault==7));
 if(fault==3){assert(d[7]==2&&d[24]==0x21&&d[25]==2&&d[31]==17&&d[32]==14&&d[33]==7);
  assert(d[34]==0&&d[35]==255&&d[36]==0&&d[37]==2&&d[42]==1&&d[43]==0);
  assert(ypwm_u16(d+14)==1&&ypwm_u16(d+40)==2&&w[2]==42&&w[4]==2&&w[5]==0x21&&w[7]==17&&w[19]==2);}
 if(fault==2||fault==6)assert(d[7]==5&&d[26]==1&&ypwm_u16(d+18)==16);
 if(fault==4)assert(d[7]==3&&d[26]==0&&ypwm_u16(d+16)==(YPWM_ALL_MASK&~4U));
 if(fault==0)assert(ypwm_u16(d+14)==YPWM_ALL_MASK&&ypwm_u16(d+16)==YPWM_ALL_MASK);
 assert(!pwm_control_mode_diagnostics(d,w,&has,false));
 assert(pwm_control_mode_diagnostics(d,w,&has,true));
 off_ok=true;verify_modes_ok=true;write_ok=true;
}
int main(void){uint8_t b[YPWM_STATUS_LENGTH];uint16_t seq;uint8_t result;
 reset();ready(8);assert(sent[3]>0&&sent[0]==0);unsigned writes=write_count;tick+=10;assert(pwm_control_service()&&write_count==writes); /* duplicate sample */
 advance();assert(pwm_control_service());assert(sent[3]>0); /* updated sample */
 tick+=500;assert(pwm_control_service());stopped(YPWM_REASON_TTL);assert(prepare(123,8)==YPWM_STALE_EPOCH);
 reset();ready(8);feedback.motor[3].position=1000;advance();assert(pwm_control_service());stopped(YPWM_REASON_TRAVEL);
 current_is_host_owned(307,false);current_is_host_owned(308,false);current_is_host_owned(309,false);
 current_is_host_owned(-308,false);current_is_host_owned(INT16_MIN,false);current_is_host_owned(INT16_MAX,false);
 current_is_host_owned(77,true);current_is_host_owned(-77,true); /* absolute sum924raw */
 reset();ready(8);feedback.valid_mask=0;advance();assert(pwm_control_service());stopped(YPWM_REASON_SENSOR);
 reset();ready(8);feedback.motor[3].current=308;feedback.motor[3].servo_error=1;advance();assert(pwm_control_service());stopped(YPWM_REASON_SENSOR);
 reset();ready(8);advance();feedback.uptime_ms=tick-51U;assert(pwm_control_service());stopped(YPWM_REASON_SENSOR);
 reset();ready(8);feedback.motor[3].temperature=60;advance();assert(pwm_control_service());stopped(YPWM_REASON_THERMAL_VOLTAGE);
 reset();ready(8);feedback.motor[3].voltage=49;advance();assert(pwm_control_service());stopped(YPWM_REASON_THERMAL_VOLTAGE);
 reset();ready(8);feedback.motor[3].voltage=75;advance();assert(pwm_control_service());stopped(YPWM_REASON_THERMAL_VOLTAGE);
 reset();ready(8);tick+=31;assert(pwm_control_service());stopped(YPWM_REASON_DEADLINE);
 reset();ready(8);advance();write_ok=false;assert(pwm_control_service());write_ok=true;stopped(YPWM_REASON_DRIVER);
 reset();ready(8);const uint8_t release[1]={YPWM_SCHEMA};assert(pwm_control_release(release,1)==0);off_ok=false;
 assert(pwm_control_service()&&phase()==YPWM_STOPPING);assert(pwm_control_status(b,true)&&ypwm_u16(b+18)==YPWM_ALL_MASK);off_ok=true;stopped(YPWM_REASON_RELEASE);
 reset();assert(prepare(123,8)==0);cancel_mode=true;assert(pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result)&&result!=0&&seq==42);stopped(YPWM_REASON_LINK);
 reset();assert(prepare(123,8)==0);assert(pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));armed=true;
 motor_target_set_t g=goals(8);assert(pwm_control_submit_positions(&g));cancel_on=true;assert(pwm_control_service());assert(sent[3]==0);stopped(YPWM_REASON_RELEASE);
 reset();ready(YPWM_ALL_MASK);for(unsigned i=0;i<12;++i)assert(sent[i]>0);
 for(unsigned i=0;i<80;++i){advance();g=goals(YPWM_ALL_MASK);for(unsigned j=0;j<12;++j)g.motor[j].position=2048;
  assert(pwm_control_submit_positions(&g));assert(pwm_control_service());assert(phase()==YPWM_ACTIVE);}
 assert(tick>1000); /* no obsolete one-second bench cutoff */
 reset();assert(pwm_control_release(release,1)==0);assert(pwm_control_service());assert(phase()==YPWM_FAULT&&write_count==0&&on_count==0);
 reset();assert(prepare(123,8)==0);assert(pwm_control_service());pwm_control_stop(YPWM_REASON_RELEASE);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_INVALID_STATE);stopped(YPWM_REASON_RELEASE);
 reset();uint8_t legacy[37]={0};assert(pwm_control_release(legacy,sizeof legacy)==YPWM_INVALID_PAYLOAD);
 assert(prepare(123,YPWM_ALL_MASK)==0);assert(pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));armed=true;
 g=goals(YPWM_ALL_MASK);g.motor[0].speed=0;assert(!pwm_control_submit_positions(&g));
 mode_diagnostics_test(0,YPWM_DIAG_COMPLETE);mode_diagnostics_test(1,YPWM_DIAG_TORQUE_OFF_TX);
 mode_diagnostics_test(2,YPWM_DIAG_TORQUE_OFF_VERIFY);mode_diagnostics_test(3,YPWM_DIAG_MODE_TRANSACTION);
 mode_diagnostics_test(4,YPWM_DIAG_MODE_VERIFY);mode_diagnostics_test(5,YPWM_DIAG_ZERO_TX);
 mode_diagnostics_test(6,YPWM_DIAG_FINAL_OFF_VERIFY);mode_diagnostics_test(7,YPWM_DIAG_MODE_TRANSACTION);
 puts("F103 PID service: detailed mode stages, first failure, host-only current, release and remaining guards passed");return 0;
}
