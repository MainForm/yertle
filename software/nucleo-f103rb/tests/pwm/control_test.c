#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "pwm_control.h"
#include "motor_control.h"
#include "position_pid.h"
#include "sensor_task.h"
#include "host_protocol.h"
#include "mode_store.h"
#include "calibration_store.h"
void calibration_memory_reset(void);
static void seed_calibration(void){calibration_memory_reset();calibration_cache_init();
 const uint8_t zero[2]={0};for(unsigned id=1;id<=12;++id)assert(calibration_cache_observe((uint8_t)id,31,zero,2));}
#include "mode_memory.h"
static uint32_t tick;static unsigned critical,on_count,off_count,write_count,completed;
static bool pwm_auto_enable;static uint16_t servo_torque_on;
static bool stop_requested,armed,off_ok,mode_ok,feedback_ok,write_ok,cancel_mode,cancel_on;
static uint16_t on_mask;static int16_t sent[12];static motor_feedback_set_t feedback;
static uint8_t mode_fail_id;static unsigned verify_off_calls,fail_off_call;static bool verify_modes_ok;
uint16_t motor_device_register_lock_uncertain_mask(void){return mode_fail_id?(uint16_t)(1U<<(mode_fail_id-1U)):0U;}
void pwm_test_enter(void){++critical;}void pwm_test_exit(void){assert(critical);--critical;}
uint32_t xTaskGetTickCount(void){return tick;}
host_fsm_state_t host_protocol_get_state(void){return armed?HOST_FSM_ARMED:HOST_FSM_DISARMED;}
bool host_protocol_is_armed(void){return armed;}bool host_protocol_is_stop_requested(void){return stop_requested;}
void host_protocol_request_motor_stop(void){stop_requested=true;armed=false;}
void host_protocol_request_target_timeout(void){host_protocol_request_motor_stop();}
void host_protocol_latch_fault(host_fault_reason_t reason)
{ assert(reason==HOST_FAULT_SERVO); host_protocol_request_motor_stop(); }
motor_device_status_t motor_device_write_targets(const motor_target_set_t *targets)
{assert(!critical && targets);++write_count;return write_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
void host_protocol_complete_safe_stop(void){++completed;stop_requested=false;}
bool motor_device_validate_targets(const motor_target_set_t *g){if(!g)return false;for(unsigned i=0;i<12;++i)if(g->motor[i].position>4095||g->motor[i].speed>32767)return false;return true;}
motor_device_status_t motor_device_set_torque(bool on){assert(!critical&&!on);++off_count;if(off_ok)servo_torque_on=0;return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool on){assert(!critical);if(on){++on_count;on_mask=mask;if(cancel_on)motor_control_stop(YPWM_REASON_RELEASE);}else{++off_count;if(off_ok)servo_torque_on&=(uint16_t)~mask;}return off_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical);*unknown=off_ok?servo_torque_on:YPWM_ALL_MASK;return off_ok&&!servo_torque_on;}
bool motor_device_verify_modes(uint8_t backend,uint16_t mask,uint16_t *verified){assert(!critical);(void)backend;*verified=mode_ok?mask:0;return mode_ok;}
bool motor_device_verify_torque_off_detailed(uint16_t *unknown,motor_verify_report_t *r){
 bool ok=motor_device_verify_torque_off(unknown);*r=motor_verify_initial_report();
 if(++verify_off_calls==fail_off_call){ok=false;*unknown=16;}
 if(!ok){r->motor_id=5;r->address=0x28;r->expected=0;r->actual=1;r->io_status=0;}return ok;
}
bool motor_device_verify_modes_detailed(uint8_t backend,uint16_t mask,uint16_t *verified,motor_verify_report_t *r){
 bool ok=motor_device_verify_modes(backend,mask,verified)&&verify_modes_ok;*r=motor_verify_initial_report();
 if(!ok){*verified=(uint16_t)(mask&(mask-1U));r->motor_id=ypwm_motor_selected(mask,3U)?3U:4U;r->address=0x21;r->expected=2;r->actual=0;r->io_status=0;}return ok;
}
bool motor_device_snapshot_modes(uint16_t mask,motor_mode_snapshot_t *m,motor_verify_report_t *r){
 assert(!critical);memset(m,0,sizeof *m);*r=motor_verify_initial_report();
 if(cancel_mode)motor_control_connection_reset();
 m->valid_mask=mask;return true;
}
bool motor_device_restore_modes(const motor_mode_snapshot_t *m,bool (*allowed)(void),uint16_t *restored,uint8_t *id,motor_register_write_report_t *r){
 assert(!critical&&ypwm_mask(m->valid_mask)&&allowed());*r=motor_register_initial_report();
 *id=mode_fail_id;*restored=(uint16_t)(m->valid_mask&~motor_device_register_lock_uncertain_mask());return *restored==m->valid_mask;
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){assert(!critical&&mask);++write_count;if(write_ok){memcpy(sent,values,sizeof sent);if(pwm_auto_enable)servo_torque_on=mask;}return write_ok?MOTOR_DEVICE_OK:MOTOR_DEVICE_UART_ERROR;}
bool sensor_get_motor_feedback_snapshot(motor_feedback_set_t *f){assert(!critical);*f=feedback;return feedback_ok;}
static uint8_t phase(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[3];}
static uint8_t reason(void){uint8_t b[YPWM_STATUS_LENGTH];assert(pwm_control_status(b,true));return b[7];}
static void reset(void){
 seed_calibration();mode_memory_reset();pwm_auto_enable=false;servo_torque_on=0;
 tick=100;critical=on_count=off_count=write_count=completed=0;on_mask=0;stop_requested=armed=false;
 off_ok=mode_ok=feedback_ok=write_ok=true;cancel_mode=cancel_on=false;memset(sent,0,sizeof sent);
 mode_fail_id=0;verify_off_calls=fail_off_call=0;verify_modes_ok=true;
 memset(&feedback,0,sizeof feedback);feedback.valid_mask=YPWM_ALL_MASK;feedback.uptime_ms=tick;feedback.sequence=1;
 for(unsigned i=0;i<12;++i){feedback.motor[i].position=2048;feedback.motor[i].valid=true;feedback.motor[i].voltage=60;}
 motor_control_init();assert(!motor_control_can_arm());motor_control_service();assert(motor_control_position_ready());motor_control_service();
 assert(phase()==1U); /* Existing PC contract for position READY. */
}
static uint8_t request_backend(uint8_t backend, uint8_t op, uint32_t epoch, uint16_t mask)
{
 uint8_t b[YPWM_MODE_LENGTH]={YPWM_SCHEMA,op,backend};ypwm_put16(b+3,mask);ypwm_put32(b+5,epoch);
 return pwm_control_request_mode(b,sizeof b,42);
}
static uint8_t request(uint8_t op,uint32_t epoch,uint16_t mask){
 return request_backend(YPWM_PID, op, epoch, mask);
}
static void stage(uint32_t epoch,uint16_t mask){
 assert(request(YPWM_PREPARE,epoch,mask)==0);motor_control_service();assert(phase()==MOTOR_CONTROL_STAGED&&!motor_control_can_arm());
 uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result)&&seq==42&&result==0);
}
static uint8_t prepare(uint32_t epoch,uint16_t mask){
 if(pwm_control_staged_epoch()!=epoch){
  const uint8_t r=request(YPWM_PREPARE,epoch,mask);if(r)return r;
  motor_control_service();uint16_t seq;uint8_t result;
  assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 }
 return request(YPWM_COMMIT,epoch,mask);
}
static motor_target_set_t goals(uint16_t mask){motor_target_set_t g={0};for(unsigned i=0;i<12;++i){g.motor[i].position=(uint16_t)((mask&(1U<<i))?2200:2048);g.motor[i].speed=200;}return g;}
static void ready(uint16_t mask){assert(prepare(123,mask)==0);motor_control_service();assert(phase()==MOTOR_CONTROL_READY&&!motor_control_can_arm());
 uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result)&&seq==42&&result==0);armed=true;
 motor_target_set_t g=goals(mask);assert(motor_control_submit_targets(&g));motor_control_service();assert(phase()==MOTOR_CONTROL_ACTIVE&&on_count==1&&on_mask==mask);
}
static void advance(void){tick+=20;feedback.uptime_ms=tick;++feedback.sequence;}
static void stopped(uint8_t why){
 if(phase()!=MOTOR_CONTROL_STOPPING||reason()!=why)
  fprintf(stderr,"stop expected reason %u, actual phase %u reason %u\n",(unsigned)why,(unsigned)phase(),(unsigned)reason());
 assert(phase()==MOTOR_CONTROL_STOPPING&&reason()==why);
 for(unsigned attempt=0;attempt<30&&phase()==MOTOR_CONTROL_STOPPING;++attempt){motor_control_service();advance();}
 assert(phase()==MOTOR_CONTROL_READY&&completed==1&&!armed);}
static void current_is_host_owned(int16_t raw,bool all){
 reset();ready(8);
 if(all){for(unsigned i=0;i<12;++i)feedback.motor[i].current=raw;}
 else feedback.motor[3].current=raw;
 const unsigned writes=write_count;advance();motor_control_service();
 assert(phase()==MOTOR_CONTROL_ACTIVE&&armed&&!stop_requested&&write_count==writes+1);
 for(unsigned i=0;i<12;++i)assert(feedback.motor[i].current==((all||i==3)?raw:0));
 /* A PC release still stops PWM and verifies torqueOFF after high telemetry. */
 motor_control_stop(YPWM_REASON_RELEASE);
 stopped(YPWM_REASON_RELEASE);assert(sent[3]==0&&off_count>0);
}
static void transaction_tests(void){
 uint8_t result;uint16_t seq;
 reset();stage(321,8);assert(!motor_control_can_arm()&&!motor_control_accepts_targets());
 assert(!pwm_control_mode_write_allowed(0,2)&&!pwm_control_mode_write_allowed(13,2));
 assert(!pwm_control_mode_write_allowed(4,1)&&!pwm_control_mode_write_allowed(4,255));
 armed=true;assert(!pwm_control_mode_write_allowed(4,2));armed=false;
 stop_requested=true;assert(!pwm_control_mode_write_allowed(4,2));stop_requested=false;
 const uint32_t staged_tick=tick;tick+=YPWM_PREPARATION_TIMEOUT_MS-1;
 assert(pwm_control_mode_write_allowed(4,2));++tick;
 assert(!pwm_control_mode_write_allowed(4,2));tick=staged_tick;
 assert(!pwm_control_mode_write_allowed(4,0)&&pwm_control_mode_write_allowed(4,2)&&!pwm_control_mode_write_allowed(1,0)&&!pwm_control_mode_write_allowed(1,2));
 motor_register_write_request_t q={0};q.motor_id=4;q.address=0x21;q.length=1;q.options=1;q.data[0]=2;
 assert(pwm_control_begin_mode_write(&q));assert(!pwm_control_begin_mode_write(&q));
 assert(request(YPWM_COMMIT,321,8)==YPWM_INVALID_STATE);
 assert(request(YPWM_CANCEL,321,8)==0);motor_control_service(); assert(phase()==MOTOR_CONTROL_ROLLING_BACK);
 assert(!pwm_control_mode_write_allowed(4,2));motor_register_write_report_t r=motor_register_initial_report();r.status=MOTOR_WRITE_OK;
 pwm_control_end_mode_write(&r);motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED&&!motor_control_can_arm());
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 const unsigned rollback_off_count=off_count;
 assert(request(YPWM_CANCEL,321,8)==0);
 motor_control_service();
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 assert(phase()==YPWM_STATUS_PHASE_CANCELED&&off_count==rollback_off_count&&!motor_control_can_arm());
 assert(!pwm_control_mode_write_allowed(4,2));
 assert(request(YPWM_PREPARE,322,8)==0&&phase()==MOTOR_CONTROL_PREPARING);
 assert(request(YPWM_CANCEL,321,8)==YPWM_STALE_EPOCH);
 reset();stage(321,8);assert(pwm_control_begin_mode_write(&q));r.status=MOTOR_WRITE_IO_ERROR;pwm_control_end_mode_write(&r);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STAGED&&!motor_control_can_arm());assert(!pwm_control_take_mode_result(&seq,&result));
 assert(request(YPWM_CANCEL,321,8)==0); motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 reset();stage(321,8);mode_fail_id=4;assert(request(YPWM_CANCEL,321,8)==0);
 motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_FAILED&&!motor_control_can_arm());
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);
 assert(request(YPWM_PREPARE,322,8)==YPWM_ROLLBACK_FAILED);
 assert(reason()==YPWM_REASON_ROLLBACK_FAILED);
 /* Same-epoch CANCEL retries restoration; link/release cannot erase context. */
 assert(request(YPWM_CANCEL,322,8)==YPWM_STALE_EPOCH);
 motor_control_connection_reset();motor_control_stop(YPWM_REASON_LINK);
 assert(request(YPWM_CANCEL,321,8)==0); motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_FAILED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);
 mode_fail_id=0;
 assert(request(YPWM_CANCEL,321,8)==0); motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0&&reason()==0&&!motor_control_can_arm());
 assert(request(YPWM_PREPARE,322,8)==0);
 /* Initialization drops rollback/session RAM; mode readiness is reverified. */
 mode_fail_id=0;motor_control_init();uint8_t fresh[YPWM_STATUS_LENGTH],old_detail[YPWM_DIAG_LENGTH];
 assert(pwm_control_status(fresh,true)&&fresh[6]==0&&fresh[7]==0&&ypwm_u32(fresh+8)==0&&ypwm_u16(fresh+4)==0);
 assert(!motor_control_can_arm()&&!pwm_control_mode_diagnostics(old_detail,false));
 assert(request(YPWM_PREPARE,323,8)==YPWM_BUSY);
 motor_control_service();
 assert(request(YPWM_PREPARE,323,8)==0);
 reset();stage(321,8);tick+=YPWM_PREPARATION_TIMEOUT_MS;motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(!motor_control_can_arm()&&reason()==YPWM_REASON_TTL);
 reset();stage(321,8);motor_control_connection_reset();motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED&&!motor_control_can_arm());
 reset();stage(321,8);assert(request(YPWM_COMMIT,321,8)==0);motor_control_service(); assert(phase()==MOTOR_CONTROL_READY);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);assert(request(YPWM_CANCEL,321,8)==0);
 motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 reset();stage(321,8);assert(request(YPWM_COMMIT,321,8)==0);motor_control_service(); assert(phase()==MOTOR_CONTROL_READY);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0);motor_control_connection_reset();
 motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED&&!motor_control_can_arm());
 reset();stage(321,8);verify_modes_ok=false;assert(request(YPWM_COMMIT,321,8)==0);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_ROLLING_BACK);motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED&&reason()==YPWM_REASON_MODE);
 uint8_t detail[YPWM_DIAG_LENGTH];assert(pwm_control_mode_diagnostics(detail,false));
 assert(detail[3]==YPWM_DIAG_MODE_VERIFY&&detail[7]==4&&detail[30]==0);
 reset();assert(request(YPWM_PREPARE,321,8)==0);cancel_mode=true;
 motor_control_service(); assert(phase()==MOTOR_CONTROL_ROLLING_BACK);motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_INVALID_STATE);
}
static void persistent_boot_tests(void){
 uint16_t seq;uint8_t result;mode_setting_t pref;
 reset();ready(8);const int16_t first=sent[3];const unsigned writes_before=mode_memory_programs;
 for(unsigned i=0;i<5;++i){advance();motor_target_set_t g=goals(8);assert(motor_control_submit_targets(&g)); motor_control_service();}
 assert(mode_memory_programs==writes_before);armed=false;motor_control_init();motor_control_service(); assert(phase()==MOTOR_CONTROL_READY);
 armed=true;motor_control_on_armed();motor_target_set_t restarted=goals(8);
 assert(motor_control_submit_targets(&restarted)); motor_control_service();assert(sent[3]==first); /* no restored I/profile/history */
 reset();stage(321,8);assert(mode_store_read(&pref)==MODE_STORE_ABSENT);assert(request(YPWM_COMMIT,321,8)==0);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_READY);assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 assert(mode_store_read(&pref)==MODE_STORE_VALID&&pref.backend==YPWM_PID&&pref.mask==8);
 /* A reboot restores selection only: verifies OFF without emitting drive commands. */
 const unsigned before_on=on_count;motor_control_init();assert(!motor_control_can_arm());motor_control_service();
 assert(phase()==MOTOR_CONTROL_READY&&motor_control_can_arm()&&!armed&&on_count==before_on);
 motor_control_connection_reset();assert(phase()==MOTOR_CONTROL_READY); /* initial HELLO */
 motor_target_set_t goal=goals(8);assert(!motor_control_submit_targets(&goal));
 armed=true;motor_control_on_armed();motor_control_connection_reset();stopped(YPWM_REASON_LINK);
 /* Actual motor-mode mismatch must not enable a restored PID backend. */
 mode_ok=false;motor_control_init();motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_FAILED&&!motor_control_can_arm());
 assert(reason()==YPWM_REASON_MODE);
 reset();mode_memory_read_ok=false;motor_control_init();
 assert(phase()==MOTOR_CONTROL_BOOT&&!motor_control_can_arm());
 assert(request(YPWM_PREPARE,321,8)==YPWM_BUSY);
 motor_control_service();assert(phase()==YPWM_STATUS_PHASE_FAILED&&!motor_control_can_arm());
 assert(request(YPWM_PREPARE,321,8)==YPWM_FLASH_READ_FAILED);
 uint8_t boot[YPWM_STATUS_LENGTH],detail[YPWM_DIAG_LENGTH];
 assert(pwm_control_status(boot,true)&&boot[6]==YPWM_FLASH_READ_FAILED&&boot[7]==YPWM_REASON_FLASH_READ);
 assert(ypwm_u32(boot+8)==0&&ypwm_u16(boot+4)==0);
 assert(!pwm_control_mode_diagnostics(detail,false));
 motor_control_connection_reset();assert(request(YPWM_PREPARE,322,8)==YPWM_FLASH_READ_FAILED);
 /* A real initialization, with readable Flash, drops old RAM latches/epochs. */
 motor_control_init();assert(pwm_control_status(boot,true));
 assert(boot[3]==MOTOR_CONTROL_BOOT&&boot[6]==0&&boot[7]==0&&ypwm_u32(boot+8)==0&&ypwm_u16(boot+4)==0);
 mode_memory_read_ok=true;motor_control_service();
 assert(motor_control_position_ready()&&request(YPWM_PREPARE,323,8)==0);
 /* A read failure during admitted PREPARE retains its own error after rollback. */
 reset();mode_memory_read_ok=false;assert(request(YPWM_PREPARE,324,8)==0);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_ROLLING_BACK);mode_memory_read_ok=true;
 motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_FLASH_READ_FAILED);
 assert(request(YPWM_PREPARE,325,8)==YPWM_FLASH_READ_FAILED);
 reset();mode_memory_read_ok=false;assert(request(YPWM_PREPARE,321,8)==0); motor_control_service();
 assert(phase()==MOTOR_CONTROL_ROLLING_BACK); motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_FLASH_READ_FAILED&&!mode_memory_programs&&!motor_control_can_arm());
 /* CANCEL before ARM restores the prior persistent selection too. */
 reset();pref=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};assert(mode_store_save(&pref));stage(321,8);
 assert(request(YPWM_COMMIT,321,8)==0); motor_control_service();assert(pwm_control_take_mode_result(&seq,&result));
 assert(request(YPWM_CANCEL,321,8)==0); motor_control_service();assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 assert(mode_store_read(&pref)==MODE_STORE_VALID&&pref.backend==YPWM_POSITION);
 /* COMMIT storage failure blocks ARM, with verified previous-preference rollback. */
 reset();pref=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};assert(mode_store_save(&pref));stage(321,8);mode_memory_budget=0;
 assert(request(YPWM_COMMIT,321,8)==0); motor_control_service();assert(phase()==MOTOR_CONTROL_ROLLING_BACK&&!motor_control_can_arm());
 motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);
 assert(mode_store_read(&pref)==MODE_STORE_VALID&&pref.backend==YPWM_POSITION);
 /* Failed Flash restore stays blocked until verified recovery. */
 reset();pref=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};assert(mode_store_save(&pref));stage(321,8);
 assert(request(YPWM_COMMIT,321,8)==0); motor_control_service();assert(pwm_control_take_mode_result(&seq,&result));mode_memory_budget=0;
 assert(request(YPWM_CANCEL,321,8)==0); motor_control_service();assert(phase()==YPWM_STATUS_PHASE_FAILED&&!motor_control_can_arm());
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_MODE_FAILED);assert(request(YPWM_PREPARE,322,8)==YPWM_ROLLBACK_FAILED);
 mode_memory_budget=-1;
 assert(request(YPWM_CANCEL,321,8)==0); motor_control_service(); assert(phase()==YPWM_STATUS_PHASE_CANCELED);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==0&&!motor_control_can_arm());
 assert(mode_store_read(&pref)==MODE_STORE_VALID&&pref.backend==YPWM_POSITION);
}
static void boot_and_commit_verification_tests(void)
{
 uint8_t status[YPWM_STATUS_LENGTH], detail[YPWM_DIAG_LENGTH], result;
 uint16_t seq;

 for (uint8_t backend = YPWM_POSITION; backend <= YPWM_PID; ++backend) {
  const mode_setting_t pref = {backend, YPWM_ALL_MASK};
  const uint8_t ready_phase = backend == YPWM_POSITION ? 1U : MOTOR_CONTROL_READY;

  reset();
  assert(mode_store_save(&pref));
  pwm_auto_enable = true;
  servo_torque_on = YPWM_ALL_MASK;
  motor_control_init();
  unsigned before_off = off_count;
  unsigned before_read = verify_off_calls;
  motor_control_service();
  assert(phase() == ready_phase && motor_control_can_arm() && !armed);
  assert(off_count == before_off + 1U && verify_off_calls == before_read + 2U);
  assert(write_count == 0U && on_count == 0U && servo_torque_on == 0U);

  reset();
  assert(request_backend(backend, YPWM_PREPARE, 901, YPWM_ALL_MASK) == YPWM_OK);
  motor_control_service();
  assert(pwm_control_take_mode_result(&seq, &result) && result == YPWM_OK);
  pwm_auto_enable = true;
  before_off = off_count;
  before_read = verify_off_calls;
  assert(request_backend(backend, YPWM_COMMIT, 901, YPWM_ALL_MASK) == YPWM_OK);
  motor_control_service();
  assert(phase() == ready_phase && !armed && !motor_control_can_arm());
  assert(off_count == before_off + 1U && verify_off_calls == before_read + 2U);
  assert(write_count == 0U && on_count == 0U && servo_torque_on == 0U);
  assert(pwm_control_status(status, true) && !ypwm_u16(status + 18));
  assert(pwm_control_take_mode_result(&seq, &result) && result == YPWM_OK && motor_control_can_arm());

  reset();
  assert(mode_store_save(&pref));
  motor_control_init();
  fail_off_call = verify_off_calls + 2U;
  motor_control_service();
  assert(pwm_control_status(status, true) && status[3] == YPWM_STATUS_PHASE_FAILED);
  assert(status[6] == YPWM_MODE_FAILED && status[7] == YPWM_REASON_MODE);
  assert(!motor_control_can_arm() && write_count == 0U);
 }

 /* A final recheck cannot override an initial OFF transmission failure. */
 reset();stage(803,YPWM_ALL_MASK);off_ok=false;
 assert(request(YPWM_COMMIT,803,YPWM_ALL_MASK)==0);motor_control_service();
 assert(phase()==MOTOR_CONTROL_ROLLING_BACK&&!motor_control_can_arm());
 assert(pwm_control_mode_diagnostics(detail,false)&&detail[3]==YPWM_DIAG_TORQUE_OFF_TX&&detail[24]==0x28&&detail[27]==YPWM_DIAG_IO_DEVICE);

 /* COMMIT must not save a preference when the final recheck fails. */
 reset();stage(804,YPWM_ALL_MASK);fail_off_call=verify_off_calls+2U;
 assert(request(YPWM_COMMIT,804,YPWM_ALL_MASK)==0);motor_control_service();
 assert(phase()==MOTOR_CONTROL_ROLLING_BACK&&!motor_control_can_arm()&&mode_memory_programs==0U);
 assert(pwm_control_mode_diagnostics(detail,false)&&detail[3]==YPWM_DIAG_FINAL_OFF_VERIFY&&detail[24]==0x28);
}
static void direct_pwm_tests(void){
 int16_t raw[12]={0};uint16_t seq;uint8_t result;
 reset();assert(pwm_control_submit_raw(raw,50)==YPWM_PWM_MODE_REQUIRED);
 stage(600,YPWM_ALL_MASK);raw[0]=1;assert(pwm_control_submit_raw(raw,51)==YPWM_INVALID_STATE);
 raw[0]=0;const unsigned before_on=on_count;
 assert(pwm_control_submit_raw(raw,52)==0&&!pwm_control_take_raw_result(&seq,&result));
 motor_control_service(); assert(pwm_control_take_raw_result(&seq,&result)&&seq==52&&result==0);
 assert(on_count==before_on&&!motor_control_can_arm()&&phase()==MOTOR_CONTROL_STAGED);
 reset();stage(601,YPWM_ALL_MASK);mode_ok=false;
 assert(pwm_control_submit_raw(raw,57)==0);const unsigned no_tx=write_count;
 motor_control_service(); assert(pwm_control_take_raw_result(&seq,&result)&&result==YPWM_PWM_MODE_REQUIRED&&write_count==no_tx);
 reset();ready(YPWM_ALL_MASK);raw[0]=-777;raw[1]=713;raw[11]=-1;advance();
 assert(pwm_control_submit_raw(raw,53)==0); motor_control_service();
 assert(pwm_control_take_raw_result(&seq,&result)&&seq==53&&result==0);
 assert(memcmp(raw,sent,sizeof raw)==0); /* bypass P/I/D and reference profile */
 raw[0]=-1001;assert(pwm_control_submit_raw(raw,54)==YPWM_INVALID_PAYLOAD);raw[0]=-777;
 /* A new position packet switches back to PID with fresh measured history. */
 motor_target_set_t g=goals(YPWM_ALL_MASK);advance();assert(motor_control_submit_targets(&g));
 motor_control_service(); assert(sent[0]!=raw[0]);
 /* Raw output has the same target TTL and stop path. */
 advance();assert(pwm_control_submit_raw(raw,55)==0); motor_control_service();
 assert(pwm_control_take_raw_result(&seq,&result)&&result==0);tick+=MOTOR_CONTROL_TARGET_TIMEOUT_MS;
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);stopped(YPWM_REASON_TTL);
 reset();ready(YPWM_ALL_MASK);assert(pwm_control_submit_raw(raw,56)==0);
 motor_control_stop(YPWM_REASON_RELEASE);
 assert(pwm_control_take_raw_result(&seq,&result)&&seq==56&&result==YPWM_INVALID_STATE);
 reset();assert(prepare(123,YPWM_ALL_MASK)==0); motor_control_service();
 assert(pwm_control_take_mode_result(&seq,&result));armed=true;cancel_on=true;
 assert(pwm_control_submit_raw(raw,58)==0); motor_control_service();
 assert(pwm_control_take_raw_result(&seq,&result)&&result==YPWM_INVALID_STATE&&phase()==MOTOR_CONTROL_STOPPING);
 assert(off_count>0);stopped(YPWM_REASON_RELEASE);
 reset();ready(YPWM_ALL_MASK);advance();write_ok=false;
 for(unsigned i=0;i<3;++i){
  assert(pwm_control_submit_raw(raw,(uint16_t)(59U+i))==0);
  motor_control_service();
  assert(pwm_control_take_raw_result(&seq,&result)&&result==YPWM_MODE_FAILED);
  assert(phase()==(i<2U?MOTOR_CONTROL_ACTIVE:MOTOR_CONTROL_STOPPING));
 }
 assert(reason()==YPWM_REASON_DRIVER);
}
static void calibrated_pid_tests(void){
 uint16_t seq;uint8_t result;const uint16_t offsets[]={3011,1328,1432};
 const int16_t wire[]={1085,3376,3480};
 for(unsigned id=1;id<=3;++id){
  reset();const uint8_t bytes[]={(uint8_t)offsets[id-1],(uint8_t)(offsets[id-1]>>8)};
  assert(calibration_cache_observe((uint8_t)id,31,bytes,2));
  const uint16_t mask=(uint16_t)(1U<<(id-1));
  assert(prepare(123,mask)==0); motor_control_service();assert(pwm_control_take_mode_result(&seq,&result));
  assert(motor_control_can_arm());armed=true;feedback.motor[id-1].position=wire[id-1];
  motor_target_set_t g=goals(mask);g.motor[id-1].position=2048;
  assert(motor_control_submit_targets(&g)); motor_control_service();
  assert(phase()==MOTOR_CONTROL_ACTIVE&&sent[id-1]==0); /* calibrated current == target */
  assert(feedback.motor[id-1].position==wire[id-1]); /* raw snapshot preserved */
 }
 reset();assert(prepare(123,1)==0); motor_control_service();assert(pwm_control_take_mode_result(&seq,&result));
 calibration_memory_reset();calibration_cache_init();assert(!motor_control_can_arm());
 armed=true;motor_target_set_t g=goals(1);const unsigned writes=write_count;
 assert(motor_control_submit_targets(&g)); motor_control_service();
 assert(phase()==MOTOR_CONTROL_STOPPING&&reason()==YPWM_REASON_CALIBRATION&&write_count==writes);
}
static void motor_polarity_service_tests(void){
 const int8_t expected[12]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
 for(int direction=-1;direction<=1;direction+=2){
  reset();ready(YPWM_ALL_MASK);
  motor_target_set_t target=goals(YPWM_ALL_MASK);
  for(unsigned i=0;i<12;++i){target.motor[i].position=(uint16_t)(2048+direction*100);target.motor[i].speed=30000;}
  advance();assert(motor_control_submit_targets(&target)); motor_control_service();
  for(unsigned i=0;i<12;++i)assert(sent[i]*direction*expected[i]>0);
 }
}
static void pid_trace_tests(void){
 uint8_t b[64],stats[28];reset();ready(YPWM_ALL_MASK);
 assert(pwm_control_pid_detail(b));FILE *fixture=fopen("pid_detail_fixture.bin","wb");assert(fixture);assert(fwrite(b,1,64,fixture)==64);const uint32_t first=ypwm_u32(b+2);
 assert(b[0]==7&&b[1]==2&&b[12]==1&&b[13]==2&&ypwm_u32(b+14)==20);
 for(unsigned j=0;j<2;++j){float reference,error,term;memcpy(&reference,b+18+j*23,4);
  memcpy(&error,b+22+j*23,4);memcpy(&term,b+26+j*23,4);
  assert(reference>2048&&error>0&&term>0&&b[40+j*23]&1);
  assert((int16_t)ypwm_u16(b+38+j*23)==sent[j]);
  assert(b[40+j*23]&32); /* IDs1/2 reversed, P/I/D remain pre-polarity */
  assert(sent[j]<0);}
 pwm_control_pid_detail_result(true);
 advance();motor_control_service();advance();motor_control_service(); /* pending replaced */
 for(unsigned pair=1;pair<6;++pair){assert(pwm_control_pid_detail(b));assert(b[12]==pair*2+1&&ypwm_u32(b+2)==first);assert(fwrite(b,1,64,fixture)==64);pwm_control_pid_detail_result(true);}
 assert(fclose(fixture)==0);
 assert(pwm_control_pid_detail(b)&&ypwm_u32(b+2)==feedback.sequence);pwm_control_pid_detail_result(false);
 assert(!pwm_control_pid_detail(b));tick+=1000;
 assert(pwm_control_pid_stats(stats)&&ypwm_u32(stats+8)==3&&ypwm_u32(stats+12)==1&&ypwm_u32(stats+16)==1&&ypwm_u32(stats+20)==1);
 reset();ready(1);assert(pwm_control_pid_detail(b));assert(b[40]&1);for(unsigned j=41;j<64;++j)assert(b[j]==0);
 for(unsigned pair=0;pair<6;++pair){assert(pwm_control_pid_detail(b));pwm_control_pid_detail_result(true);}
 advance();write_ok=false;motor_control_service();assert(!pwm_control_pid_detail(b)); /* no fake sent result */
}
static void return_first_tests(void){
 /* Simulate physical encoder progress, not only a goal/reference update. */
 reset();ready(YPWM_ALL_MASK);feedback.motor[0].position=2100;advance();motor_control_service();
 motor_control_stop(YPWM_REASON_LINK);const unsigned before_off=off_count;
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING&&off_count==before_off);
 const unsigned before_write=write_count;motor_control_service(); assert(write_count==before_write);
 assert(sent[0]>0); /* all-negative physical PWM polarity: move raw downward */
 for(unsigned a=0;a<8;++a){advance();feedback.motor[0].position-=4;motor_control_service();}
 feedback.motor[0].position=2048;
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_READY&&completed==1);
 assert(off_count>before_off&&servo_torque_on==0);
 uint8_t detail[20];assert(pwm_control_stop_detail(detail)&&detail[2]==2&&ypwm_u16(detail+16)==0&&ypwm_u16(detail+18)==0);
 /* New session must reset numerical history after the return has finished. */
 assert(prepare(124,YPWM_ALL_MASK)==0); motor_control_service();
 uint16_t seq;uint8_t result;assert(pwm_control_take_mode_result(&seq,&result)&&result==0);
 armed=true;motor_target_set_t g=goals(YPWM_ALL_MASK);advance();
 assert(motor_control_submit_targets(&g)); motor_control_service();
 uint8_t trace[64];assert(pwm_control_pid_detail(trace));assert(trace[40]&64U);

 /* Return path must permit 4.1V and fall back to OFF at 4.0V. */
 reset();ready(YPWM_ALL_MASK);feedback.motor[0].position=2100;feedback.motor[0].voltage=41;
 motor_control_stop(YPWM_REASON_RELEASE);motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 feedback.motor[0].voltage=40;advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_READY);
 assert(pwm_control_stop_detail(detail)&&detail[2]==5&&servo_torque_on==0);
 /* Failed OFF attempts must not send PWM0 again and re-enable the servo. */
 reset();ready(YPWM_ALL_MASK);pwm_auto_enable=true;off_ok=false;
 motor_control_stop(YPWM_REASON_RELEASE);
 const unsigned writes_at_stop=write_count;
 for(unsigned i=0;i<3;++i){advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);}
 const unsigned writes_at_off=write_count;
 assert(writes_at_off>writes_at_stop&&completed==0U);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING&&write_count==writes_at_off);
 off_ok=true;advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_READY&&completed==1U);
 assert(write_count==writes_at_off&&servo_torque_on==0U);
 /* A mechanically stalled motor and stale sensor both fall back to OFF. */
 reset();ready(YPWM_ALL_MASK);feedback.motor[0].position=2100;motor_control_stop(YPWM_REASON_RELEASE);
 for(unsigned a=0;a<30&&phase()==MOTOR_CONTROL_STOPPING;++a){advance();motor_control_service();}
 assert(phase()==MOTOR_CONTROL_READY&&completed==1);
 assert(pwm_control_stop_detail(detail)&&detail[2]==7);
 /* Read failures count once per 20ms, recover on a good sample. */
 reset();ready(YPWM_ALL_MASK);feedback.motor[0].position=2100;
 motor_control_stop(YPWM_REASON_SENSOR);const unsigned off_before=off_count;feedback_ok=false;
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING&&off_count==off_before);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 feedback_ok=true;advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 assert(sent[0]>0&&sent[0]<=45); /* first 4 raw reference step */
 feedback_ok=false;advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_READY&&servo_torque_on==0);
 assert(pwm_control_stop_detail(detail)&&detail[2]==4);
 /* Incomplete samples also need three failed opportunities. */
 reset();ready(YPWM_ALL_MASK);feedback.motor[0].position=2100;motor_control_stop(YPWM_REASON_RELEASE);
 feedback.valid_mask=0;motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_READY&&servo_torque_on==0);
 /* A frozen sequence is eventually a failure. */
 reset();ready(YPWM_ALL_MASK);feedback.motor[0].position=2100;motor_control_stop(YPWM_REASON_RELEASE);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 for(unsigned i=0;i<2;++i){tick+=20;motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);}
 tick+=20;motor_control_service(); assert(phase()==MOTOR_CONTROL_READY&&servo_torque_on==0);
 /* Nonzero calibration must be removed before checking the raw2048 center. */
 reset();ready(YPWM_ALL_MASK);const uint8_t offset[2]={100,0};
 assert(calibration_cache_observe(1,31,offset,2));feedback.motor[0].position=2148;
 motor_control_stop(YPWM_REASON_RELEASE);stopped(YPWM_REASON_RELEASE);
}
static void speed_acc_do_not_shape_pwm(void){
 const uint16_t speeds[]={0,1,200,3000,32767};
 const uint8_t accs[]={0,10,100,255};int16_t expected[12]={0};
 for(unsigned v=0;v<sizeof speeds/sizeof speeds[0];++v)for(unsigned a=0;a<sizeof accs/sizeof accs[0];++a){
  reset();ready(YPWM_ALL_MASK);advance();motor_target_set_t g=goals(YPWM_ALL_MASK);
  for(unsigned i=0;i<12;++i){g.motor[i].position=2148;g.motor[i].speed=speeds[v];g.motor[i].acceleration=accs[a];}
  assert(motor_control_submit_targets(&g));motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
  if(!v&&!a)memcpy(expected,sent,sizeof sent);else assert(memcmp(expected,sent,sizeof sent)==0);
 }
}
static void removed_time_limits_and_retained_target_ttl(void){
 motor_target_set_t g;uint8_t b[64];
 /* Same-valued target packets keep the target TTL alive beyond500ms. */
 reset();ready(8);g=goals(8);
 for(unsigned j=0;j<60;++j){advance();assert(motor_control_submit_targets(&g)); motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);}
 /* Fresh feedback alone DOES NOT refresh accepted target time. */
 reset();ready(8);
 for(unsigned j=0;j<24;++j){advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);}
 advance();motor_control_service();stopped(YPWM_REASON_TTL);
 /* No service-gap/feedback-dt ceiling; retain current target packets. */
 const unsigned gaps[]={31,51,100,300,1000};
 for(unsigned j=0;j<sizeof gaps/sizeof gaps[0];++j){
  reset();ready(8);tick+=gaps[j];++feedback.sequence;feedback.uptime_ms=tick;g=goals(8);
  assert(motor_control_submit_targets(&g)); motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
  assert(pwm_control_pid_detail(b)&&ypwm_u32(b+14)==gaps[j]);
 }
 /* An old but otherwise complete, advancing sample is not an age stop. */
 reset();ready(8);tick+=100;++feedback.sequence;feedback.uptime_ms+=20;
 motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 /* Direct PWM also ignores old service/feedback age while targets refresh. */
 reset();ready(8);tick+=100;
 {int16_t raw[12]={0};uint16_t sequence;uint8_t result;raw[3]=100;
  assert(pwm_control_submit_raw(raw,77)==0); motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
  assert(pwm_control_take_raw_result(&sequence,&result)&&sequence==77&&result==0&&sent[3]==100);}
 /* Duplicate samples do not integrate, zero/backward dt stays invalid. */
 reset();ready(8);++feedback.sequence;tick+=20;
 motor_control_service();stopped(YPWM_REASON_SENSOR);
 reset();ready(8);++feedback.sequence;--feedback.uptime_ms;tick+=20;
 motor_control_service();stopped(YPWM_REASON_SENSOR);
 /* Return PID also accepts a300ms gap, still bounded by return progress. */
 reset();ready(YPWM_ALL_MASK);feedback.motor[0].position=2100;motor_control_stop(YPWM_REASON_RELEASE);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);tick+=300;++feedback.sequence;feedback.uptime_ms=tick;
 motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);
 /* Three consecutive hot sequences still trip even100ms apart. */
 reset();ready(8);feedback.motor[3].temperature=60;
 for(unsigned j=0;j<2;++j){tick+=100;++feedback.sequence;feedback.uptime_ms=tick;motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);}
 tick+=100;++feedback.sequence;feedback.uptime_ms=tick;motor_control_service();stopped(YPWM_REASON_THERMAL_VOLTAGE);
}
static void common_position_lifecycle_test(void)
{
 reset();
 armed=true;
 motor_control_on_armed();
 const motor_target_set_t target=goals(YPWM_ALL_MASK);
 assert(motor_control_submit_targets(&target));
 motor_control_service();
 assert(on_count==1U && !motor_control_position_ready() && phase()==1U);

 write_ok=false;
 for(unsigned i=0U;i<3U;++i){
  advance();
  motor_control_service();
  assert(phase()==(i<2U?1U:MOTOR_CONTROL_STOPPING));
 }
 assert(reason()==YPWM_REASON_DRIVER);
 write_ok=true;
 for(unsigned i=0U;i<4U;++i){advance();motor_control_service();}
 assert(motor_control_position_ready() && completed==1U && !armed);
 const unsigned writes_at_stop=write_count;
 armed=true;
 motor_control_on_armed();
 motor_control_service();
 assert(write_count==writes_at_stop && on_count==1U);

 feedback.motor[0].voltage=40U;
 assert(motor_control_submit_targets(&target));
 motor_control_service();
 assert(phase()==MOTOR_CONTROL_STOPPING && reason()==YPWM_REASON_THERMAL_VOLTAGE);
 assert(write_count==writes_at_stop && on_count==1U);
}

int main(void){
 common_position_lifecycle_test();uint8_t b[YPWM_STATUS_LENGTH];uint16_t seq;uint8_t result;
 removed_time_limits_and_retained_target_ttl();
 speed_acc_do_not_shape_pwm();
 return_first_tests();motor_polarity_service_tests();pid_trace_tests();calibrated_pid_tests();boot_and_commit_verification_tests();direct_pwm_tests();
 reset();ready(8);assert(sent[3]<0&&sent[0]==0);unsigned writes=write_count;tick+=10;motor_control_service(); assert(write_count==writes); /* duplicate sample */
 advance();motor_control_service();assert(sent[3]<0); /* updated sample */
 tick+=500;motor_control_service();stopped(YPWM_REASON_TTL);assert(prepare(123,8)==YPWM_STALE_EPOCH);
 reset();ready(8);feedback.motor[3].position=1000;advance();
 motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE&&!stop_requested);
 assert(sent[3]>=-YPWM_DUTY_LIMIT&&sent[3]<=YPWM_DUTY_LIMIT); /* large error does not stop */
 current_is_host_owned(307,false);current_is_host_owned(308,false);current_is_host_owned(309,false);
 current_is_host_owned(-308,false);current_is_host_owned(INT16_MIN,false);current_is_host_owned(INT16_MAX,false);
 current_is_host_owned(77,true);current_is_host_owned(-77,true); /* absolute sum924raw */
 reset();ready(8);feedback.valid_mask=0;advance();motor_control_service();stopped(YPWM_REASON_SENSOR);
 reset();ready(8);feedback.motor[3].current=308;feedback.motor[3].servo_error=1;advance();motor_control_service();stopped(YPWM_REASON_SENSOR);
 /* Same ID, three consecutive fresh complete snapshots. Duplicates do not count. */
 reset();ready(8);feedback.motor[3].temperature=60;
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 advance();motor_control_service();stopped(YPWM_REASON_THERMAL_VOLTAGE);
 /* A cold sample, missing sequence, or another motor breaks the streak. */
 reset();ready(8);feedback.motor[3].temperature=60;advance();motor_control_service();
 feedback.motor[3].temperature=59;advance();motor_control_service();
 feedback.motor[3].temperature=60;advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 advance();++feedback.sequence;motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 feedback.motor[3].temperature=20;feedback.motor[4].temperature=60;
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 advance();motor_control_service();stopped(YPWM_REASON_THERMAL_VOLTAGE);
 /* 4.1V remains active; 4.0V or below triggers shutdown. */
 reset();ready(8);feedback.motor[3].voltage=41;advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_ACTIVE);
 reset();ready(8);feedback.motor[3].voltage=40;advance();motor_control_service();stopped(YPWM_REASON_THERMAL_VOLTAGE);
 reset();ready(8);feedback.motor[3].voltage=39;advance();motor_control_service();stopped(YPWM_REASON_THERMAL_VOLTAGE);
 reset();ready(8);feedback.motor[3].voltage=75;advance();motor_control_service();stopped(YPWM_REASON_THERMAL_VOLTAGE);
 reset();ready(8);write_ok=false;
 for(unsigned i=0;i<3;++i){
  advance();motor_control_service();
  assert(phase()==(i<2U?MOTOR_CONTROL_ACTIVE:MOTOR_CONTROL_STOPPING));
 }
 write_ok=true;stopped(YPWM_REASON_DRIVER);
 reset();ready(8);motor_control_stop(YPWM_REASON_RELEASE);off_ok=false;
 for(unsigned a=0;a<4;++a){advance();motor_control_service(); assert(phase()==MOTOR_CONTROL_STOPPING);}assert(pwm_control_status(b,true)&&ypwm_u16(b+18)==YPWM_ALL_MASK);off_ok=true;stopped(YPWM_REASON_RELEASE);
 reset();assert(prepare(123,8)==0);motor_control_service();assert(pwm_control_take_mode_result(&seq,&result));armed=true;
 motor_target_set_t g=goals(8);assert(motor_control_submit_targets(&g));cancel_on=true;motor_control_service();stopped(YPWM_REASON_RELEASE);
 reset();ready(YPWM_ALL_MASK);for(unsigned i=0;i<12;++i)
  assert(sent[i]*position_pid_motor_polarity((uint8_t)(i+1U))>0);
 for(unsigned i=0;i<80;++i){advance();g=goals(YPWM_ALL_MASK);for(unsigned j=0;j<12;++j)g.motor[j].position=2048;
  assert(motor_control_submit_targets(&g));motor_control_service();assert(phase()==MOTOR_CONTROL_ACTIVE);}
 assert(tick>1000); /* no obsolete one-second bench cutoff */
 reset();motor_control_stop(YPWM_REASON_RELEASE);motor_control_service();assert(phase()==1U&&write_count==0&&on_count==0);
 reset();assert(prepare(123,8)==0);motor_control_service();motor_control_stop(YPWM_REASON_RELEASE);
 assert(pwm_control_take_mode_result(&seq,&result)&&result==YPWM_INVALID_STATE);stopped(YPWM_REASON_RELEASE);
 reset();uint8_t legacy[8]={2,1};assert(pwm_control_request_mode(legacy,sizeof legacy,42)==YPWM_INVALID_PAYLOAD);
 assert(prepare(123,YPWM_ALL_MASK)==0);motor_control_service();assert(pwm_control_take_mode_result(&seq,&result));armed=true;
 g=goals(YPWM_ALL_MASK);g.motor[0].speed=0;assert(motor_control_submit_targets(&g)); /* ignored in PWM backend */
 transaction_tests();
 persistent_boot_tests();
 puts("F103 PID service: two-phase register transaction, rollback interlock, host-only current and remaining guards passed");return 0;
}

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *m, imu_feedback_t *i, pmw3901_feedback_t *f){memset(i,0,sizeof *i);memset(f,0,sizeof *f);return sensor_get_motor_feedback_snapshot(m);}
