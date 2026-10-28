#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mode_memory.h"
#include "mode_store.h"
#include "calibration_store.h"
void calibration_memory_reset(void);
static void seed_calibration(void){calibration_memory_reset();calibration_cache_init();
 const uint8_t zero[2]={0};for(unsigned id=1;id<=12;++id)assert(calibration_cache_observe((uint8_t)id,31,zero,2));}
#include "../../firmware/src/host_protocol.c"
static TickType_t tick=100;static unsigned critical,queues,on_count,native_writes;static int queue;
static int16_t duty[12];static uint16_t torque_mask;static uint8_t last_result,last_error;
static unsigned detail_frames,stats_frames;static bool boot_off_ok=true;
static unsigned tx_order,diag_order,write_diag_order,result_order;static uint8_t mode_fail_id,last_diag[64],servo_modes[12];
static motor_register_write_result_t pending_write;static bool write_ready;
void pwm_test_enter(void){++critical;}void pwm_test_exit(void){assert(critical);--critical;}
TickType_t xTaskGetTickCount(void){return tick;}void vTaskDelay(TickType_t t){tick+=t;}
void vTaskDelayUntil(TickType_t *p,TickType_t t){tick=*p+t;*p=tick;}
void *xQueueCreate(unsigned n,unsigned size){assert(n==1&&size);return &queue;}
BaseType_t xQueueOverwrite(void *q,const void *p){assert(q&&p);++queues;return pdPASS;}
BaseType_t xQueueReceive(void *q,void *p,TickType_t timeout){(void)q;(void)p;(void)timeout;return 0;}
BaseType_t xQueueReset(void *q){assert(q);return pdPASS;}
bool host_uart_write_bytes(const uint8_t *data,size_t n){uint8_t raw[YERTLE_PROTOCOL_MAX_RAW],type=0;uint16_t seq=0;size_t length=0;
 assert(cobs_decode(data+1,n-2,raw,&length)&&validate_frame(raw,length,&type,&seq));
 ++tx_order;
 if(type==MESSAGE_EVENT&&raw[7]==7){assert(raw[5]==64);++detail_frames;}
 if(type==MESSAGE_EVENT&&raw[7]==8){assert(raw[5]==28);++stats_frames;}
 if(type==MESSAGE_EVENT&&raw[7]==YPWM_DIAG_EVENT){assert(raw[5]==64);diag_order=tx_order;memcpy(last_diag,raw+7,64);}
 if(type==REGISTER_WRITE_DIAGNOSTICS_TYPE){write_diag_order=tx_order;assert(raw[5]==64);}
 if(type==MESSAGE_ACK||type==MESSAGE_NACK){last_result=type;last_error=raw[10];result_order=tx_order;}return true;
}
bool host_uart_try_read(uint8_t *b,bool *d){(void)b;*d=false;return false;}
bool sensor_take_position_offsets(motor_position_offsets_t *r){(void)r;return false;}
bool sensor_request_position_offsets(uint16_t s){(void)s;return false;}
bool sensor_take_motor_register(motor_register_result_t *r){(void)r;return false;}
bool sensor_request_motor_register(uint16_t s,uint8_t id,uint8_t a,uint8_t n){(void)s;(void)id;(void)a;(void)n;return false;}
bool sensor_take_motor_register_write(motor_register_write_result_t *r){if(!write_ready)return false;*r=pending_write;write_ready=false;return true;}
bool sensor_request_motor_register_write(uint16_t seq,const motor_register_write_request_t *q){
 if(!pwm_control_begin_mode_write(q))return false;
 pending_write.request_sequence=seq;pending_write.report=motor_register_initial_report();
 pending_write.report.status=q->motor_id==mode_fail_id?MOTOR_WRITE_VERIFY_MISMATCH:MOTOR_WRITE_OK;
 if(pending_write.report.status==MOTOR_WRITE_OK)servo_modes[q->motor_id-1]=q->data[0];
 pwm_control_end_mode_write(&pending_write.report);write_ready=true;return true;
}
uint16_t motor_device_register_lock_uncertain_mask(void){return 0;}
void uart_diagnostics_send(void *c,uart_diag_sender_t send){(void)c;(void)send;}
bool sensor_get_feedback_snapshot(motor_feedback_set_t *f,imu_feedback_t *imu){assert(!critical);memset(f,0,sizeof *f);memset(imu,0,sizeof *imu);
 f->sequence=tick/20+1;f->uptime_ms=tick;f->valid_mask=YPWM_ALL_MASK;for(unsigned i=0;i<12;++i){f->motor[i].valid=true;f->motor[i].position=2048;f->motor[i].voltage=60;}return true;}
bool motor_device_validate_targets(const motor_target_set_t *g){for(unsigned i=0;i<12;++i)if(g->motor[i].position>4095||g->motor[i].speed>32767)return false;return true;}
motor_device_status_t motor_device_write_targets(const motor_target_set_t *g){(void)g;++native_writes;return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_set_torque(bool on){assert(!critical&&!on);if(!boot_off_ok)return MOTOR_DEVICE_UART_ERROR;torque_mask=0;return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool on){assert(!critical);if(on){++on_count;torque_mask|=mask;}else torque_mask&=(uint16_t)~mask;return MOTOR_DEVICE_OK;}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical);*unknown=torque_mask;return !torque_mask;}
bool motor_device_verify_modes(uint8_t backend,uint16_t mask,uint16_t *verified){
 assert(!critical);*verified=0;for(unsigned i=0;i<12;++i)if(servo_modes[i]==((backend==YPWM_PID&&(mask&(1U<<i)))?2:0))*verified|=(uint16_t)(1U<<i);
 return *verified==YPWM_ALL_MASK;
}
bool motor_device_verify_modes_detailed(uint8_t backend,uint16_t mask,uint16_t *verified,motor_verify_report_t *r){*r=motor_verify_initial_report();return motor_device_verify_modes(backend,mask,verified);}
bool motor_device_verify_torque_off_detailed(uint16_t *unknown,motor_verify_report_t *r){*r=motor_verify_initial_report();return motor_device_verify_torque_off(unknown);}
bool motor_device_snapshot_modes(motor_mode_snapshot_t *m,motor_verify_report_t *r){
 assert(!critical);memcpy(m->mode,servo_modes,12);m->valid_mask=YPWM_ALL_MASK;*r=motor_verify_initial_report();return true;
}
bool motor_device_restore_modes(const motor_mode_snapshot_t *m,bool (*allowed)(void),uint16_t *restored,uint8_t *id,motor_register_write_report_t *r){
 assert(!critical&&allowed());memcpy(servo_modes,m->mode,12);*restored=YPWM_ALL_MASK;*id=0;*r=motor_register_initial_report();return true;
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){assert(!critical&&mask);memcpy(duty,values,sizeof duty);torque_mask|=mask;return MOTOR_DEVICE_OK;}
static void receive(protocol_context_t *c,uint8_t type,const uint8_t *p,uint16_t n){
 static uint16_t seq;uint8_t raw[YERTLE_PROTOCOL_MAX_RAW]={1,type};ypwm_put16(raw+3,++seq);raw[5]=(uint8_t)n;if(n)memcpy(raw+7,p,n);
 ypwm_put16(raw+7+n,crc16_ccitt(raw,7+n));handle_frame(c,raw,9+n);
}
static void configure(protocol_context_t *c,uint8_t backend,uint32_t epoch){
 uint8_t mode[9]={YPWM_SCHEMA,YPWM_PREPARE,backend};ypwm_put16(mode+3,YPWM_ALL_MASK);ypwm_put32(mode+5,epoch);
 receive(c,MESSAGE_SET_CONTROL_MODE,mode,9);assert(c->pwm_mode_pending);
 receive(c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(c);assert(c->state==HOST_FSM_DISARMED);
 assert(pwm_control_service());service_pwm(c);assert(!c->pwm_mode_pending&&!pwm_control_can_arm());
 for(uint8_t id=1;id<=12;++id){
  const uint8_t q[5]={id,0x21,1,1,backend==YPWM_PID?2:0};
  receive(c,MESSAGE_SET_MOTOR_REGISTER,q,5);assert(c->register_job_active);
  service_motor_register_write(c);assert(last_result==MESSAGE_ACK);
 }
 mode[1]=YPWM_COMMIT;receive(c,MESSAGE_SET_CONTROL_MODE,mode,9);assert(c->pwm_mode_pending);
 assert(pwm_control_service());service_pwm(c);assert(!c->pwm_mode_pending&&!c->lease_established&&torque_mask==0);
}
static void transition(protocol_context_t *c,uint8_t backend,uint32_t epoch){
 configure(c,backend,epoch);
 receive(c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(c);assert(c->state==HOST_FSM_ARMED);
}
int main(void){
 seed_calibration();mode_memory_reset();
 protocol_context_t c={0};c.host_connected=true;c.state=HOST_FSM_DISARMED;c.last_host_heartbeat_tick=tick;published_state=HOST_FSM_DISARMED;
 assert(actuator_task_init());assert(pwm_control_service());assert(pwm_control_native_ready());
 /* Slow DISARMED configuration may outlive5s, but never ARM. A silent
    configuration still disconnects/rolls back at its own bounded20s window. */
 uint8_t slow[9]={YPWM_SCHEMA,YPWM_PREPARE,YPWM_PID};ypwm_put16(slow+3,YPWM_ALL_MASK);ypwm_put32(slow+5,118);
 receive(&c,MESSAGE_SET_CONTROL_MODE,slow,9);assert(pwm_control_service());service_pwm(&c);
 tick+=6000;expire_host_link(&c);try_enter_control_ready(&c);
 assert(c.host_connected&&c.state==HOST_FSM_DISARMED&&!pwm_control_can_arm()&&on_count==0&&native_writes==0);
 tick+=YPWM_PREPARATION_LINK_TIMEOUT_MS;expire_host_link(&c);assert(!c.host_connected);
 service_safety_state(&c);assert(pwm_control_service());service_safety_state(&c);
 receive(&c,MESSAGE_HELLO,NULL,0);assert(c.host_connected);
 uint8_t legacy[8]={2,1};receive(&c,MESSAGE_SET_CONTROL_MODE,legacy,8);assert(last_result==MESSAGE_NACK&&!c.pwm_mode_pending);
 const uint8_t forbidden[5]={1,0x21,1,1,2};receive(&c,MESSAGE_SET_MOTOR_REGISTER,forbidden,5);assert(last_result==MESSAGE_NACK);
 /* Configure-only: read queries preserve the idle link for an arbitrary wait,
    but cannot establish a lease, ARM, turn torque on or send a native target. */
 configure(&c,YPWM_PID,120);
 for(unsigned i=0;i<12;++i){tick+=250;receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,NULL,0);
  try_enter_control_ready(&c);assert(c.host_connected&&!c.lease_established&&c.state==HOST_FSM_DISARMED);
  assert(pwm_control_service());assert(on_count==0&&native_writes==0&&pwm_control_can_arm());}
 /* Losing the monitoring link before H still restores the previous setting. */
 tick+=1001;expire_host_link(&c);assert(!c.host_connected);service_safety_state(&c);assert(pwm_control_service());service_safety_state(&c);
 for(unsigned i=0;i<12;++i)assert(servo_modes[i]==0);
 receive(&c,MESSAGE_HELLO,NULL,0);assert(c.host_connected);
 transition(&c,YPWM_PID,123);
 /* Even continuous read traffic cannot renew an established control lease. */
 const TickType_t heartbeat_tick=c.last_host_heartbeat_tick;
 tick+=1;receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,NULL,0);assert(c.last_host_heartbeat_tick==heartbeat_tick);
 assert(diag_order&&diag_order<result_order&&last_diag[3]==YPWM_DIAG_COMPLETE);
 uint8_t targets[60]={0};for(unsigned i=0;i<12;++i){ypwm_put16(targets+5*i,2200);ypwm_put16(targets+5*i+2,200);}
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK&&queues==0);
 assert(pwm_control_service());assert(on_count==1&&torque_mask==YPWM_ALL_MASK&&native_writes==0);
 for(unsigned i=0;i<12;++i){const int8_t expected[12]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
  assert(duty[i]*expected[i]>0&&duty[i]>=-YPWM_DUTY_LIMIT&&duty[i]<=YPWM_DUTY_LIMIT);}
 /* Removed 0x0F must be rejected; ordinary target/heartbeat watchdog stops PID. */
 receive(&c,0x0f,NULL,0);assert(last_result==MESSAGE_NACK);
 tick+=501;assert(pwm_control_service());service_safety_state(&c);assert(c.state==HOST_FSM_DISARMED);
 for(unsigned a=0;a<4;++a){tick+=20;assert(pwm_control_service());service_safety_state(&c);}
 assert(!host_protocol_is_stop_requested());
 for(unsigned i=0;i<12;++i)assert(duty[i]==0);
 /* A new connection after a completed runtime stop must revalidate the
    persisted selection without EEPROM writes or energizing the motor. */
 assert(!pwm_control_can_arm());
 const unsigned recover_programs=mode_memory_programs,recover_on=on_count;
 set_host_connected(&c,false);receive(&c,MESSAGE_HELLO,NULL,0);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED&&!pwm_control_can_arm());
 assert(pwm_control_service()&&!pwm_control_can_arm()); /* request -> unverified */
 assert(pwm_control_service()&&pwm_control_can_arm()); /* OFF/mode/PWM0/OFF */
 uint8_t resumed[YPWM_STATUS_LENGTH];assert(pwm_control_status(resumed,true));
 assert(resumed[7]==YPWM_REASON_NONE); /* PC beginConfiguredControl requires this */
 assert(!torque_mask&&on_count==recover_on&&mode_memory_programs==recover_programs);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED); /* no stale lease */
 receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_ARMED);
 assert(on_count==recover_on); /* no torque until an explicit new target */
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(pwm_control_service());
 assert(on_count==recover_on+1);
 tick+=501;assert(pwm_control_service());service_safety_state(&c);
 /* HELLO during STOPPING is remembered, but cannot ARM or skip OFF. */
 set_host_connected(&c,false);receive(&c,MESSAGE_HELLO,NULL,0);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED&&!pwm_control_can_arm());
 for(unsigned a=0;a<4;++a){tick+=20;assert(pwm_control_service());service_safety_state(&c);}
 assert(!host_protocol_is_stop_requested());
 /* Physical mode mismatch on reconnect must stay FAILED and torque OFF. */
 servo_modes[3]=0;
 assert(pwm_control_service()&&pwm_control_service());
 assert(!pwm_control_can_arm()&&!torque_mask&&on_count==recover_on+1);
 servo_modes[3]=2;
 transition(&c,YPWM_POSITION,124);assert(pwm_control_native_ready());
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK&&queues==1);
 published_state=HOST_FSM_DISARMED;c.state=HOST_FSM_DISARMED;mode_fail_id=2;
 uint8_t mode[9]={YPWM_SCHEMA,YPWM_PREPARE,YPWM_PID};ypwm_put16(mode+3,YPWM_ALL_MASK);ypwm_put32(mode+5,125);
 receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);assert(pwm_control_service());service_pwm(&c);
 const uint8_t q[5]={2,0x21,1,1,2};receive(&c,MESSAGE_SET_MOTOR_REGISTER,q,5);service_motor_register_write(&c);
 assert(last_result==MESSAGE_NACK&&last_error==15&&write_diag_order<result_order);
 uint8_t failure_detail[64],write_detail[64];bool has_write;
 assert(pwm_control_mode_diagnostics(failure_detail,write_detail,&has_write,false));
 assert(failure_detail[3]==YPWM_DIAG_MODE_TRANSACTION&&failure_detail[7]==2&&failure_detail[6]==YPWM_MODE_FAILED);
 assert(pwm_control_service());try_enter_control_ready(&c);assert(!pwm_control_can_arm()&&c.state==HOST_FSM_DISARMED);
 /* Transient 0x0D failures wait safely for the PC's explicit CANCEL/retry. */
 mode[1]=YPWM_CANCEL;receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);
 assert(pwm_control_service());service_pwm(&c);assert(!pwm_control_can_arm());
 assert(last_diag[3]==YPWM_DIAG_ROLLBACK&&last_diag[6]==0&&ypwm_u16(last_diag+16)==YPWM_ALL_MASK);
 /* Same-epoch CANCEL confirms already completed requested rollback. */
 mode[1]=YPWM_CANCEL;receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);service_pwm(&c);
 assert(last_result==MESSAGE_ACK);for(unsigned i=0;i<12;++i)assert(servo_modes[i]==0);
 /* Reboot with a stored PID preference: HELLO cannot ARM; fresh heartbeat can
    start a verified new session without a fresh mode/EEPROM transaction. */
 const mode_setting_t saved={YPWM_PID,YPWM_ALL_MASK};assert(mode_store_save(&saved));
 for(unsigned i=0;i<12;++i)servo_modes[i]=2;
 const unsigned before_program=mode_memory_programs,before_on=on_count;
 /* Boot ordering: actuator can run before host init; boot OFF must finish
    before verification grants READY and HELLO + heartbeat can arm. */
 memset(&c,0,sizeof c);c.state=HOST_FSM_BOOT;published_state=HOST_FSM_BOOT;
 pending_safety_action=HOST_SAFETY_ACTION_NONE;safe_stop_completed=requested_pwm_stop=false;
 pwm_control_init();assert(pwm_control_service()&&!pwm_control_can_arm());
 request_safe_stop();transition_state(&c,HOST_FSM_DISARMED,STATE_CAUSE_BOOT_COMPLETE,false);
 boot_off_ok=false;assert(pwm_control_service()&&!pwm_control_can_arm());service_safety_state(&c);
 assert(host_protocol_is_stop_requested()); /* no readiness on failed OFF */
 boot_off_ok=true;assert(pwm_control_service()&&!pwm_control_can_arm());service_safety_state(&c);
 assert(!host_protocol_is_stop_requested());assert(pwm_control_service()&&pwm_control_can_arm());
 receive(&c,MESSAGE_HELLO,NULL,0);receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);
 assert(c.state==HOST_FSM_ARMED); /* no F5 and no auto torque */
 memset(&c,0,sizeof c);c.state=HOST_FSM_DISARMED;published_state=HOST_FSM_DISARMED;
 pending_safety_action=HOST_SAFETY_ACTION_NONE;safe_stop_completed=requested_pwm_stop=false;
 pwm_control_init();receive(&c,MESSAGE_HELLO,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED);
 assert(pwm_control_service());receive(&c,MESSAGE_HELLO,NULL,0);try_enter_control_ready(&c);
 assert(c.state==HOST_FSM_DISARMED&&on_count==before_on&&mode_memory_programs==before_program);
 receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_ARMED&&on_count==before_on);
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK&&pwm_control_service());
 assert(on_count==before_on+1&&native_writes==0&&mode_memory_programs==before_program);
 /* Flash read failure at first boot has its own NACK, with no adopted epoch. */
 memset(&c,0,sizeof c);c.state=HOST_FSM_DISARMED;published_state=HOST_FSM_DISARMED;
 pending_safety_action=HOST_SAFETY_ACTION_NONE;safe_stop_completed=requested_pwm_stop=false;
 mode_memory_read_ok=false;pwm_control_init();receive(&c,MESSAGE_HELLO,NULL,0);
 mode[1]=YPWM_PREPARE;ypwm_put32(mode+5,456);
 receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);
 assert(last_result==MESSAGE_NACK&&last_error==YPWM_FLASH_READ_FAILED&&!c.pwm_mode_pending);
 uint8_t boot_status[YPWM_STATUS_LENGTH];assert(pwm_control_status(boot_status,true));
 assert(boot_status[6]==YPWM_FLASH_READ_FAILED&&boot_status[7]==YPWM_REASON_FLASH_READ&&ypwm_u32(boot_status+8)==0);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED&&!pwm_control_can_arm());
 mode_memory_read_ok=true;pwm_control_init();assert(pwm_control_status(boot_status,true)&&boot_status[6]==0&&ypwm_u32(boot_status+8)==0);
 /* Wrong backend rejects a complete PWM frame, even while DISARMED. */
 mode_memory_reset();pwm_control_init();c.host_connected=true;c.pwm_mode_pending=false;
 uint8_t raw_pwm[YPWM_DIRECT_LENGTH]={0};
 receive(&c,MESSAGE_SET_MOTOR_PWM,raw_pwm,sizeof raw_pwm);
 assert(last_result==MESSAGE_NACK&&last_error==YPWM_PWM_MODE_REQUIRED);
 receive(&c,MESSAGE_SET_MOTOR_PWM,raw_pwm,23);
 assert(last_result==MESSAGE_NACK&&last_error==ERROR_INVALID_LENGTH);
 mode_fail_id=0;configure(&c,YPWM_PID,789);receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);
 assert(c.state==HOST_FSM_ARMED);
 ypwm_put16(raw_pwm,(uint16_t)(int16_t)-900);ypwm_put16(raw_pwm+2,713);
 receive(&c,MESSAGE_SET_MOTOR_PWM,raw_pwm,sizeof raw_pwm);
 assert(pwm_control_service());service_pwm(&c);
 assert(last_result==MESSAGE_ACK&&last_error==0&&duty[0]==-900&&duty[1]==713);
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK&&pwm_control_service());
 assert(duty[0]!=-900); /* existing position->PID route resumes */
 /* Host pressure delays diagnostics; never pop a partial set before ACKs. */
 unsigned before_detail=detail_frames;c.result_count=1;service_pid_detail(&c);assert(detail_frames==before_detail);c.result_count=0;
 c.host_connected=false;service_pid_detail(&c);assert(detail_frames==before_detail);c.host_connected=true;
 for(unsigned pair=0;pair<6;++pair){service_pid_detail(&c);assert(detail_frames==before_detail+pair+1);}
 /* Continuous diagnostics still emit 1Hz producer/loss counters. */
 tick+=1000;service_pid_detail(&c);assert(stats_frames>0);
 puts("Host/task transaction, Flash isolation and direct PWM routing passed");return 0;
}
