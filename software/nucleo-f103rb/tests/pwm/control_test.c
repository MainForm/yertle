#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "pwm_control.h"
#include "sensor_task.h"
#include "host_protocol.h"
#include "mode_store.h"
#include "calibration_store.h"
void calibration_memory_reset(void);
static void seed_calibration(void){calibration_memory_reset();calibration_cache_init();
 const uint8_t zero[2]={0};for(unsigned id=1;id<=12;++id)assert(calibration_cache_observe((uint8_t)id,31,zero,2));}
#include "mode_memory.h"
static uint32_t tick;static unsigned critical,on_count,off_count,write_count,completed,unsafe_count;
static bool pwm_auto_enable,fail_off_after_pwm;static uint16_t servo_torque_on;
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
motor_device_status_t motor_device_set_torque(bool on){assert(!critical&&!on);++off_count;if(off_ok)servo_torque_on=0;return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool on){assert(!critical);if(on){++on_count;on_mask=mask;if(cancel_on)pwm_control_stop(YPWM_REASON_RELEASE);}else{++off_count;if(off_ok)servo_torque_on&=(uint16_t)~mask;}return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical);*unknown=off_ok?servo_torque_on:YPWM_ALL_MASK;return off_ok&&!servo_torque_on;}
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
bool motor_device_snapshot_modes(motor_mode_snapshot_t *m,motor_verify_report_t *r){
 assert(!critical);memset(m,0,sizeof *m);*r=motor_verify_initial_report();
 if(cancel_mode)pwm_control_connection_reset();
 m->valid_mask=YPWM_ALL_MASK;return true;
}
bool motor_device_restore_modes(const motor_mode_snapshot_t *m,bool (*allowed)(void),uint16_t *restored,uint8_t *id,motor_register_write_report_t *r){
 assert(!critical&&m->valid_mask==YPWM_ALL_MASK&&allowed());*r=motor_register_initial_report();
 *id=mode_fail_id;*restored=mode_fail_id?(uint16_t)(YPWM_ALL_MASK&~2U):YPWM_ALL_MASK;return !mode_fail_id;
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){assert(!critical&&mask);++write_count;if(write_ok){memcpy(sent,values,sizeof sent);if(pwm_auto_enable)servo_torque_on=mask;if(fail_off_after_pwm)off_ok=false;}return write_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
bool sensor_get_feedback_snapshot(motor_feedback_set_t *f,imu_feedback_t *imu){assert(!critical);*f=feedback;memset(imu,0,sizeof *imu);return feedback_ok;}
static uint8_t phase(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[3];}
static uint8_t reason(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[7];}
static void reset(void){
 seed_calibration();mode_memory_reset();pwm_auto_enable=fail_off_after_pwm=false;servo_torque_on=0;
 tick=100;critical=on_count=off_count=write_count=completed=unsafe_count=0;on_mask=0;stop_requested=armed=false;
 off_ok=mode_ok=feedback_ok=write_ok=true;cancel_mode=cancel_on=false;memset(sent,0,sizeof sent);
 mode_fail_id=0;verify_off_calls=fail_off_call=0;verify_modes_ok=true;
 memset(&feedback,0,sizeof feedback);feedback.valid_mask=YPWM_ALL_MASK;feedback.uptime_ms=tick;feedback.sequence=1;
 for(unsigned i=0;i<12;++i){feedback.motor[i].position=2048;feedback.motor[i].valid=true;feedback.motor[i].voltage=60;}
 pwm_control_init();assert(!pwm_control_can_arm());assert(pwm_control_service());assert(pwm_control_native_ready());assert(!pwm_control_service());
}
static uint8_t request(uint8_t op,uint32_t epoch,uint16_t mask){
 uint8_t b[YPWM_MODE_LENGTH]={YPWM_SCHEMA,op,YPWM_PID};ypwm_put16(b+3,mask);ypwm_put32(b+5,epoch);
 return pwm_control_request_mode(b,sizeof b,42);
}
static void stage(uint32_t epoch,uint16_t mask){
 assert(request(YPWM_PREPARE,epoch,mask)==0);assert(pwm_control_service());assert(phase()==YPWM_STAGED&&!pwm_control_can_arm());
 uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result)&&seq==42&&result==0);
}
static uint8_t prepare(uint32_t epoch,uint16_t mask){
 if(pwm_control_staged_epoch()!=epoch){
  const uint8_t r=request(YPWM_PREPARE,epoch,mask);if(r)return r;
  assert(pwm_control_service());uint16_t seq;uint8_t result;
  assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 }
 return request(YPWM_COMMIT,epoch,mask);
}
static motor_target_set_t goals(uint16_t mask){motor_target_set_t g={0};for(unsigned i=0;i<12;++i){g.motor[i].position=(uint16_t)((mask&(1U<<i))?2200:2048);g.motor[i].speed=200;}return g;}
static void ready(uint16_t mask){assert(prepare(123,mask)==0);assert(pwm_control_service());assert(phase()==YPWM_READY&&!pwm_control_can_arm());
 uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result)&&seq==42&&result==0);armed=true;
 motor_target_set_t g=goals(mask);assert(pwm_control_submit_positions(&g));assert(pwm_control_service());assert(phase()==YPWM_ACTIVE&&on_count==1&&on_mask==mask);
}
static void advance(void){tick+=20;feedback.uptime_ms=tick;++feedback.sequence;}
static void stopped(uint8_t why){assert(phase()==YPWM_STOPPING&&reason()==why);assert(pwm_control_service());assert(phase()==YPWM_FAILED&&completed==1&&!pwm_control_can_arm());}
static void current_is_host_owned(int16_t raw,bool all){
 reset();ready(8);
 if(all){for(unsigned i=0;i<12;++i)feedback.motor[i].current=raw;}
 else feedback.motor[3].current=raw;
 const unsigned writes=write_count;advance();assert(pwm_control_service());
 assert(phase()==YPWM_ACTIVE&&armed&&!stop_requested&&unsafe_count==0&&write_count==writes+1);
 for(unsigned i=0;i<12;++i)assert(feedback.motor[i].current==((all||i==3)?raw:0));
 /* A PC release still stops PWM and verifies torqueOFF after high telemetry. */
 pwm_control_stop(YPWM_REASON_RELEASE);
 stopped(YPWM_REASON_RELEASE);assert(sent[3]==0&&off_count>0&&unsafe_count==0);
}
static void transaction_tests(void){
 uint8_t result;uint16_t seq;
 reset();stage(321,8);assert(!pwm_control_can_arm()&&!pwm_control_accepts_positions());
 assert(!pwm_control_mode_write_allowed(0,2)&&!pwm_control_mode_write_allowed(13,2));
 assert(!pwm_control_mode_write_allowed(4,1)&&!pwm_control_mode_write_allowed(4,255));
 armed=true;assert(!pwm_control_mode_write_allowed(4,2));armed=false;
 stop_requested=true;assert(!pwm_control_mode_write_allowed(4,2));stop_requested=false;
 const uint32_t staged_tick=tick;tick+=YPWM_PREPARATION_TIMEOUT_MS-1;
 assert(pwm_control_mode_write_allowed(4,2));++tick;
 assert(!pwm_control_mode_write_allowed(4,2));tick=staged_tick;
 assert(!pwm_control_mode_write_allowed(4,0)&&pwm_control_mode_write_allowed(4,2)&&pwm_control_mode_write_allowed(1,0));
 motor_register_write_request_t q={0};q.motor_id=4;q.address=0x21;q.length=1;q.options=1;q.data[0]=2;
 assert(pwm_control_begin_mode_write(&q));assert(!pwm_control_begin_mode_write(&q));
 assert(request(YPWM_COMMIT,321,8)==YPWM_INVALID_STATE);
 assert(request(YPWM_CANCEL,321,8)==0);assert(pwm_control_service()&&phase()==YPWM_ROLLING_BACK);
 assert(!pwm_control_mode_write_allowed(4,2));motor_register_write_report_t r=motor_register_initial_report();r.status=MOTOR_WRITE_OK;
 pwm_control_end_mode_write(&r);assert(pwm_control_service()&&phase()==YPWM_CANCELED&&!pwm_control_can_arm());
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 assert(request(YPWM_CANCEL,321,8)==0&&pwm_control_take_mode_result(&seq,&result)&&result==0);
 reset();stage(321,8);assert(pwm_control_begin_mode_write(&q));r.status=MOTOR_WRITE_IO_ERROR;pwm_control_end_mode_write(&r);
 assert(pwm_control_service()&&phase()==YPWM_STAGED&&!pwm_control_can_arm());assert(!pwm_control_take_mode_result(&seq,&result));
 assert(request(YPWM_CANCEL,321,8)==0&&pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 reset();stage(321,8);mode_fail_id=2;assert(request(YPWM_CANCEL,321,8)==0);
 assert(pwm_control_service()&&phase()==YPWM_FAILED&&!pwm_control_can_arm());
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);
 assert(request(YPWM_PREPARE,322,8)==YPWM_ROLLBACK_FAILED);
 assert(reason()==YPWM_REASON_ROLLBACK_FAILED);
 /* Same-epoch CANCEL retries restoration; link/release cannot erase context. */
 assert(request(YPWM_CANCEL,322,8)==YPWM_STALE_EPOCH);
 pwm_control_connection_reset();pwm_control_stop(YPWM_REASON_LINK);
 assert(request(YPWM_CANCEL,321,8)==0&&pwm_control_service()&&phase()==YPWM_FAILED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);
 mode_fail_id=0;
 assert(request(YPWM_CANCEL,321,8)==0&&pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0&&reason()==0&&!pwm_control_can_arm());
 assert(request(YPWM_PREPARE,322,8)==0);
 /* Initialization drops rollback/session RAM; mode readiness is reverified. */
 mode_fail_id=0;pwm_control_init();uint8_t fresh[YPWM_STATUS_LENGTH],old_detail[64],old_write[64];bool old_has;
 assert(pwm_control_status(fresh,true)&&fresh[6]==0&&fresh[7]==0&&ypwm_u32(fresh+8)==0&&ypwm_u16(fresh+4)==0);
 assert(!pwm_control_can_arm()&&!pwm_control_mode_diagnostics(old_detail,old_write,&old_has,false));
 assert(request(YPWM_PREPARE,323,8)==0);
 reset();stage(321,8);tick+=YPWM_PREPARATION_TIMEOUT_MS;assert(pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(!pwm_control_can_arm()&&reason()==YPWM_REASON_TTL);
 reset();stage(321,8);pwm_control_connection_reset();assert(pwm_control_service()&&phase()==YPWM_CANCELED&&!pwm_control_can_arm());
 reset();stage(321,8);assert(request(YPWM_COMMIT,321,8)==0);assert(pwm_control_service()&&phase()==YPWM_READY);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);assert(request(YPWM_CANCEL,321,8)==0);
 assert(pwm_control_service()&&phase()==YPWM_CANCELED);assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 reset();stage(321,8);assert(request(YPWM_COMMIT,321,8)==0);assert(pwm_control_service()&&phase()==YPWM_READY);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);pwm_control_connection_reset();
 assert(pwm_control_service()&&phase()==YPWM_CANCELED&&!pwm_control_can_arm());
 reset();stage(321,8);verify_modes_ok=false;assert(request(YPWM_COMMIT,321,8)==0);
 assert(pwm_control_service()&&phase()==YPWM_ROLLING_BACK);assert(pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);
 uint8_t detail[64],write[64];bool has;assert(pwm_control_mode_diagnostics(detail,write,&has,false));
 assert(detail[3]==YPWM_DIAG_MODE_VERIFY&&detail[7]==3&&!has);
 reset();assert(request(YPWM_PREPARE,321,8)==0);cancel_mode=true;
 assert(pwm_control_service()&&phase()==YPWM_ROLLING_BACK);assert(pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_INVALID_STATE);
}
static void persistent_boot_tests(void){
 uint16_t seq;uint8_t result;mode_setting_t pref;
 reset();ready(8);const int16_t first=sent[3];const unsigned writes_before=mode_memory_programs;
 for(unsigned i=0;i<5;++i){advance();motor_target_set_t g=goals(8);assert(pwm_control_submit_positions(&g)&&pwm_control_service());}
 assert(mode_memory_programs==writes_before);armed=false;pwm_control_init();assert(pwm_control_service()&&phase()==YPWM_READY);
 armed=true;pwm_control_on_armed();motor_target_set_t restarted=goals(8);
 assert(pwm_control_submit_positions(&restarted)&&pwm_control_service());assert(sent[3]==first); /* no restored I/profile/history */
 reset();stage(321,8);assert(!mode_store_load(&pref));assert(request(YPWM_COMMIT,321,8)==0);
 assert(pwm_control_service()&&phase()==YPWM_READY);assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 assert(mode_store_load(&pref)&&pref.backend==YPWM_PID&&pref.mask==8);
 /* A reboot restores selection only: zero PWM/OFF verified, no goals/torqueON. */
 const unsigned before_on=on_count;pwm_control_init();assert(!pwm_control_can_arm());assert(pwm_control_service());
 assert(phase()==YPWM_READY&&pwm_control_can_arm()&&!armed&&on_count==before_on);
 pwm_control_connection_reset();assert(phase()==YPWM_READY); /* initial HELLO */
 motor_target_set_t goal=goals(8);assert(!pwm_control_submit_positions(&goal));
 pwm_control_on_armed();pwm_control_connection_reset();stopped(YPWM_REASON_LINK);
 /* Actual motor-mode mismatch must not enable a restored PID backend. */
 mode_ok=false;pwm_control_init();assert(pwm_control_service()&&phase()==YPWM_FAILED&&!pwm_control_can_arm());
 reset();mode_memory_read_ok=false;pwm_control_init();assert(phase()==YPWM_FAILED&&!pwm_control_can_arm());
 assert(pwm_control_service()&&!pwm_control_can_arm());assert(request(YPWM_PREPARE,321,8)==YPWM_FLASH_READ_FAILED);
 uint8_t boot[YPWM_STATUS_LENGTH],detail[64],write[64];bool has;
 assert(pwm_control_status(boot,true)&&boot[6]==YPWM_FLASH_READ_FAILED&&boot[7]==YPWM_REASON_FLASH_READ);
 assert(ypwm_u32(boot+8)==0&&ypwm_u16(boot+4)==0);
 assert(!pwm_control_mode_diagnostics(detail,write,&has,false));
 pwm_control_connection_reset();assert(request(YPWM_PREPARE,322,8)==YPWM_FLASH_READ_FAILED);
 /* A real initialization, with readable Flash, drops old RAM latches/epochs. */
 mode_memory_read_ok=true;pwm_control_init();assert(pwm_control_status(boot,true));
 assert(boot[6]==0&&boot[7]==0&&ypwm_u32(boot+8)==0&&ypwm_u16(boot+4)==0);
 assert(request(YPWM_PREPARE,323,8)==0);
 /* A read failure during admitted PREPARE retains its own error after rollback. */
 reset();mode_memory_read_ok=false;assert(request(YPWM_PREPARE,324,8)==0);
 assert(pwm_control_service()&&phase()==YPWM_ROLLING_BACK);mode_memory_read_ok=true;
 assert(pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_FLASH_READ_FAILED);
 assert(request(YPWM_PREPARE,325,8)==YPWM_FLASH_READ_FAILED);
 reset();mode_memory_read_ok=false;assert(request(YPWM_PREPARE,321,8)==0&&pwm_control_service());
 assert(phase()==YPWM_ROLLING_BACK&&pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_FLASH_READ_FAILED&&!mode_memory_programs&&!pwm_control_can_arm());
 /* CANCEL before ARM restores the prior persistent selection too. */
 reset();pref=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};assert(mode_store_save(&pref));stage(321,8);
 assert(request(YPWM_COMMIT,321,8)==0&&pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));
 assert(request(YPWM_CANCEL,321,8)==0&&pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 assert(mode_store_load(&pref)&&pref.backend==YPWM_POSITION);
 /* COMMIT storage failure blocks ARM, with verified previous-preference rollback. */
 reset();pref=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};assert(mode_store_save(&pref));stage(321,8);mode_memory_budget=0;
 assert(request(YPWM_COMMIT,321,8)==0&&pwm_control_service());assert(phase()==YPWM_ROLLING_BACK&&!pwm_control_can_arm());
 assert(pwm_control_service()&&phase()==YPWM_CANCELED);assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);
 assert(mode_store_load(&pref)&&pref.backend==YPWM_POSITION);
 /* Failed Flash restore stays blocked until verified recovery. */
 reset();pref=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};assert(mode_store_save(&pref));stage(321,8);
 assert(request(YPWM_COMMIT,321,8)==0&&pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));mode_memory_budget=0;
 assert(request(YPWM_CANCEL,321,8)==0&&pwm_control_service());assert(phase()==YPWM_FAILED&&!pwm_control_can_arm());
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);assert(request(YPWM_PREPARE,322,8)==YPWM_ROLLBACK_FAILED);
 mode_memory_budget=-1;
 assert(request(YPWM_CANCEL,321,8)==0&&pwm_control_service()&&phase()==YPWM_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0&&!pwm_control_can_arm());
 assert(mode_store_load(&pref)&&pref.backend==YPWM_POSITION);
}
/* Real-device observation: writing PWM0 may energize a previously OFF motor. */
static void zero_then_explicit_off_tests(void){
 uint8_t b[YPWM_STATUS_LENGTH],d[64],w[64],result;uint16_t seq;bool has;
 reset();stage(801,YPWM_ALL_MASK);pwm_auto_enable=true;
 unsigned before_off=off_count;
 assert(request(YPWM_COMMIT,801,YPWM_ALL_MASK)==0&&pwm_control_service());
 assert(phase()==YPWM_READY&&!armed&&!pwm_control_can_arm()&&!servo_torque_on&&on_count==0);
 assert(off_count>=before_off+2&&pwm_control_status(b,true)&&!ypwm_u16(b+18));
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0&&pwm_control_can_arm());
 /* A persisted PWM selection is ready only after explicit OFF and readback. */
 const mode_setting_t pref={YPWM_PID,YPWM_ALL_MASK};
 reset();assert(mode_store_save(&pref));pwm_auto_enable=true;pwm_control_init();before_off=off_count;
 assert(pwm_control_service()&&phase()==YPWM_READY&&!armed&&!servo_torque_on&&on_count==0);
 assert(off_count>=before_off+2&&pwm_control_can_arm());
 /* A PWM TX error still attempts OFF; the original zero-TX failure is retained. */
 reset();stage(802,YPWM_ALL_MASK);write_ok=false;before_off=off_count;
 assert(request(YPWM_COMMIT,802,YPWM_ALL_MASK)==0&&pwm_control_service());
 assert(phase()==YPWM_ROLLING_BACK&&!pwm_control_can_arm()&&off_count>=before_off+2);
 assert(pwm_control_mode_diagnostics(d,w,&has,false)&&d[3]==YPWM_DIAG_ZERO_TX&&d[24]==0x2c);
 /* An explicit OFF TX error must block preparation readiness. */
 reset();stage(803,YPWM_ALL_MASK);pwm_auto_enable=fail_off_after_pwm=true;
 assert(request(YPWM_COMMIT,803,YPWM_ALL_MASK)==0&&pwm_control_service());
 assert(phase()==YPWM_ROLLING_BACK&&!pwm_control_can_arm());
 assert(pwm_control_mode_diagnostics(d,w,&has,false)&&d[3]==YPWM_DIAG_TORQUE_OFF_TX&&d[24]==0x28&&d[27]==YPWM_DIAG_IO_DEVICE);
 /* A readable ON value at the last OFF check also blocks completion. */
 reset();stage(804,YPWM_ALL_MASK);fail_off_call=verify_off_calls+2;
 assert(request(YPWM_COMMIT,804,YPWM_ALL_MASK)==0&&pwm_control_service());
 assert(phase()==YPWM_ROLLING_BACK&&!pwm_control_can_arm());
 assert(pwm_control_mode_diagnostics(d,w,&has,false)&&d[3]==YPWM_DIAG_FINAL_OFF_VERIFY&&d[24]==0x28);
 /* Boot must attempt OFF after PWM TX failure, and never enable ARM. */
 reset();assert(mode_store_save(&pref));write_ok=false;pwm_control_init();before_off=off_count;
 assert(pwm_control_service()&&phase()==YPWM_FAILED&&!pwm_control_can_arm()&&off_count>=before_off+2);
 reset();assert(mode_store_save(&pref));pwm_auto_enable=fail_off_after_pwm=true;pwm_control_init();
 assert(pwm_control_service()&&phase()==YPWM_FAILED&&!pwm_control_can_arm());
}
static void direct_pwm_tests(void){
 int16_t raw[12]={0};uint16_t seq;uint8_t result;
 reset();assert(pwm_control_submit_raw(raw,50)==YPWM_PWM_MODE_REQUIRED);
 stage(600,YPWM_ALL_MASK);raw[0]=1;assert(pwm_control_submit_raw(raw,51)==YPWM_INVALID_STATE);
 raw[0]=0;const unsigned before_on=on_count;
 assert(pwm_control_submit_raw(raw,52)==0&&!pwm_control_take_raw_result(&seq,&result));
 assert(pwm_control_service()&&pwm_control_take_raw_result(&seq,&result)&&seq==52&&result==0);
 assert(on_count==before_on&&!pwm_control_can_arm()&&phase()==YPWM_STAGED);
 reset();stage(601,YPWM_ALL_MASK);mode_ok=false;
 assert(pwm_control_submit_raw(raw,57)==0);const unsigned no_tx=write_count;
 assert(pwm_control_service()&&pwm_control_take_raw_result(&seq,&result)&&result==YPWM_PWM_MODE_REQUIRED&&write_count==no_tx);
 reset();ready(YPWM_ALL_MASK);raw[0]=-800;raw[1]=713;raw[11]=-1;advance();
 assert(pwm_control_submit_raw(raw,53)==0&&pwm_control_service());
 assert(pwm_control_take_raw_result(&seq,&result)&&seq==53&&result==0);
 assert(memcmp(raw,sent,sizeof raw)==0); /* bypass P/I/D and reference profile */
 raw[0]=-801;assert(pwm_control_submit_raw(raw,54)==YPWM_INVALID_PAYLOAD);raw[0]=-800;
 /* A new position packet switches back to PID with fresh measured history. */
 motor_target_set_t g=goals(YPWM_ALL_MASK);advance();assert(pwm_control_submit_positions(&g));
 assert(pwm_control_service()&&sent[0]!=raw[0]);
 /* Raw output has the same target TTL and stop path. */
 advance();assert(pwm_control_submit_raw(raw,55)==0&&pwm_control_service());
 assert(pwm_control_take_raw_result(&seq,&result)&&result==0);tick+=YPWM_TARGET_TIMEOUT_MS;
 assert(pwm_control_service()&&phase()==YPWM_STOPPING);assert(pwm_control_service()&&phase()==YPWM_FAILED);
 reset();ready(YPWM_ALL_MASK);assert(pwm_control_submit_raw(raw,56)==0);
 pwm_control_stop(YPWM_REASON_RELEASE);
 assert(pwm_control_take_raw_result(&seq,&result)&&seq==56&&result==YPWM_INVALID_STATE);
 reset();assert(prepare(123,YPWM_ALL_MASK)==0&&pwm_control_service());
 assert(pwm_control_take_mode_result(&seq,&result));armed=true;cancel_on=true;
 assert(pwm_control_submit_raw(raw,58)==0&&pwm_control_service());
 assert(pwm_control_take_raw_result(&seq,&result)&&result==YPWM_INVALID_STATE&&phase()==YPWM_STOPPING);
 assert(off_count>0);assert(pwm_control_service()&&phase()==YPWM_FAILED);
 reset();ready(YPWM_ALL_MASK);advance();write_ok=false;
 assert(pwm_control_submit_raw(raw,59)==0&&pwm_control_service());
 assert(pwm_control_take_raw_result(&seq,&result)&&result==YPWM_MODE_FAILED&&reason()==YPWM_REASON_DRIVER);
}
static void calibrated_pid_tests(void){
 uint16_t seq;uint8_t result;const uint16_t offsets[]={3011,1328,1432};
 const int16_t wire[]={1085,3376,3480};
 for(unsigned id=1;id<=3;++id){
  reset();const uint8_t bytes[]={(uint8_t)offsets[id-1],(uint8_t)(offsets[id-1]>>8)};
  assert(calibration_cache_observe((uint8_t)id,31,bytes,2));
  const uint16_t mask=(uint16_t)(1U<<(id-1));
  assert(prepare(123,mask)==0&&pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));
  assert(pwm_control_can_arm());armed=true;feedback.motor[id-1].position=wire[id-1];
  motor_target_set_t g=goals(mask);g.motor[id-1].position=2048;
  assert(pwm_control_submit_positions(&g)&&pwm_control_service());
  assert(phase()==YPWM_ACTIVE&&sent[id-1]==0); /* calibrated current == target */
  assert(feedback.motor[id-1].position==wire[id-1]); /* raw snapshot preserved */
 }
 reset();assert(prepare(123,1)==0&&pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));
 calibration_memory_reset();calibration_cache_init();assert(!pwm_control_can_arm());
 armed=true;motor_target_set_t g=goals(1);const unsigned writes=write_count;
 assert(pwm_control_submit_positions(&g)&&pwm_control_service());
 assert(phase()==YPWM_STOPPING&&reason()==YPWM_REASON_CALIBRATION&&write_count==writes);
}
static void pid_trace_tests(void){
 uint8_t b[64],stats[28];reset();ready(YPWM_ALL_MASK);
 assert(pwm_control_pid_detail(b));FILE *fixture=fopen("pid_detail_fixture.bin","wb");assert(fixture);assert(fwrite(b,1,64,fixture)==64);const uint32_t first=ypwm_u32(b+2);
 assert(b[0]==7&&b[1]==1&&b[12]==1&&b[13]==2&&b[14]==20&&b[15]==0);
 for(unsigned j=0;j<2;++j){float reference,error,term;memcpy(&reference,b+16+j*24,4);
  memcpy(&error,b+20+j*24,4);memcpy(&term,b+24+j*24,4);
  assert(reference>2048&&error>0&&term>0&&b[38+j*24]&1);
  assert((int16_t)ypwm_u16(b+36+j*24)==sent[j]);}
 pwm_control_pid_detail_result(true);
 advance();assert(pwm_control_service());advance();assert(pwm_control_service()); /* pending replaced */
 for(unsigned pair=1;pair<6;++pair){assert(pwm_control_pid_detail(b));assert(b[12]==pair*2+1&&ypwm_u32(b+2)==first);assert(fwrite(b,1,64,fixture)==64);pwm_control_pid_detail_result(true);}
 assert(fclose(fixture)==0);
 assert(pwm_control_pid_detail(b)&&ypwm_u32(b+2)==feedback.sequence);pwm_control_pid_detail_result(false);
 assert(!pwm_control_pid_detail(b));tick+=1000;
 assert(pwm_control_pid_stats(stats)&&ypwm_u32(stats+8)==3&&ypwm_u32(stats+12)==1&&ypwm_u32(stats+16)==1&&ypwm_u32(stats+20)==1);
 reset();ready(1);assert(pwm_control_pid_detail(b));assert(b[38]&1);for(unsigned j=40;j<64;++j)assert(b[j]==0);
 for(unsigned pair=0;pair<6;++pair){assert(pwm_control_pid_detail(b));pwm_control_pid_detail_result(true);}
 advance();write_ok=false;assert(pwm_control_service());assert(!pwm_control_pid_detail(b)); /* no fake sent result */
}
int main(void){uint8_t b[YPWM_STATUS_LENGTH];uint16_t seq;uint8_t result;
 pid_trace_tests();calibrated_pid_tests();zero_then_explicit_off_tests();direct_pwm_tests();
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
 reset();ready(8);pwm_control_stop(YPWM_REASON_RELEASE);off_ok=false;
 assert(pwm_control_service()&&phase()==YPWM_STOPPING);assert(pwm_control_status(b,true)&&ypwm_u16(b+18)==YPWM_ALL_MASK);off_ok=true;stopped(YPWM_REASON_RELEASE);
 reset();assert(prepare(123,8)==0);assert(pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));armed=true;
 motor_target_set_t g=goals(8);assert(pwm_control_submit_positions(&g));cancel_on=true;assert(pwm_control_service());assert(sent[3]==0);stopped(YPWM_REASON_RELEASE);
 reset();ready(YPWM_ALL_MASK);for(unsigned i=0;i<12;++i)assert(sent[i]>0);
 for(unsigned i=0;i<80;++i){advance();g=goals(YPWM_ALL_MASK);for(unsigned j=0;j<12;++j)g.motor[j].position=2048;
  assert(pwm_control_submit_positions(&g));assert(pwm_control_service());assert(phase()==YPWM_ACTIVE);}
 assert(tick>1000); /* no obsolete one-second bench cutoff */
 reset();pwm_control_stop(YPWM_REASON_RELEASE);assert(pwm_control_service());assert(phase()==YPWM_FAILED&&write_count==0&&on_count==0);
 reset();assert(prepare(123,8)==0);assert(pwm_control_service());pwm_control_stop(YPWM_REASON_RELEASE);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_INVALID_STATE);stopped(YPWM_REASON_RELEASE);
 reset();uint8_t legacy[8]={2,1};assert(pwm_control_request_mode(legacy,sizeof legacy,42)==YPWM_INVALID_PAYLOAD);
 assert(prepare(123,YPWM_ALL_MASK)==0);assert(pwm_control_service());assert(pwm_control_take_mode_result(&seq,&result));armed=true;
 g=goals(YPWM_ALL_MASK);g.motor[0].speed=0;assert(!pwm_control_submit_positions(&g));
 transaction_tests();
 persistent_boot_tests();
 puts("F103 PID service: two-phase register transaction, rollback interlock, host-only current and remaining guards passed");return 0;
}
