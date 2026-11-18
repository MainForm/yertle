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
static TickType_t tick=100;static unsigned critical,on_count,native_writes;
static int16_t duty[12];static motor_target_set_t native_target;static uint16_t torque_mask;static uint8_t last_result,last_error;
static unsigned detail_frames,stats_frames;static bool boot_off_ok=true;
static uint8_t last_mode_status[YPWM_STATUS_LENGTH];
static uint8_t imu_tx_valid, imu_tx_status;
static bool imu_ready, imu_valid;
static unsigned tx_order,diag_order,write_diag_order,result_order;static uint8_t mode_fail_id,last_diag[64],servo_modes[12];

void pwm_test_enter(void){++critical;}void pwm_test_exit(void){assert(critical);--critical;}
TickType_t xTaskGetTickCount(void){return tick;}void vTaskDelay(TickType_t t){tick+=t;}
void vTaskDelayUntil(TickType_t *p,TickType_t t){tick=*p+t;*p=tick;}
bool host_uart_write_bytes(const uint8_t *data,size_t n){uint8_t raw[YERTLE_PROTOCOL_MAX_RAW],type=0;uint16_t seq=0;size_t length=0;
 assert(cobs_decode(data+1,n-2,raw,&length)&&validate_frame(raw,length,&type,&seq));
 ++tx_order;
 if(type==MESSAGE_EVENT&&raw[7]==7){assert(raw[5]==64);++detail_frames;}
 if(type==MESSAGE_EVENT&&raw[7]==8){assert(raw[5]==28);++stats_frames;}
 if(type==MESSAGE_EVENT&&raw[7]==YPWM_EVENT_CODE){assert(raw[5]==YPWM_STATUS_LENGTH);memcpy(last_mode_status,raw+7,sizeof last_mode_status);}
 if(type==MESSAGE_EVENT&&raw[7]==YPWM_DIAG_EVENT){assert(raw[5]==64);diag_order=tx_order;memcpy(last_diag,raw+7,64);}
 if(type==REGISTER_WRITE_DIAGNOSTICS_TYPE){write_diag_order=tx_order;assert(raw[5]==64);}
 if(type==MESSAGE_IMU_FEEDBACK){assert(raw[5]==24);imu_tx_valid=raw[15];imu_tx_status=raw[16];}
 if(type==MESSAGE_ACK||type==MESSAGE_NACK){last_result=type;last_error=raw[10];result_order=tx_order;}return true;
}
bool host_uart_try_read(uint8_t *b,bool *d){(void)b;*d=false;return false;}
motor_device_status_t motor_device_read_position_offset(uint8_t id, motor_position_offset_t *offset)
{ (void)id; (void)offset; assert(0); return MOTOR_DEVICE_BUSY; }
motor_register_status_t motor_device_read_register(uint8_t id, uint8_t address, uint8_t length, motor_register_data_t *result)
{ (void)id; (void)address; (void)length; (void)result; assert(0); return MOTOR_REGISTER_BUSY; }
motor_register_write_report_t motor_device_write_register(const motor_register_write_request_t *request, bool (*allowed)(void))
{ (void)request; (void)allowed; assert(0); return motor_register_initial_report(); }
motor_register_write_report_t motor_device_write_mode(uint8_t id,uint8_t mode,bool (*allowed)(void))
{
    assert(!critical && allowed());
    motor_register_write_report_t report = motor_register_initial_report();
    report.status = id == mode_fail_id ? MOTOR_WRITE_VERIFY_MISMATCH : MOTOR_WRITE_OK;
    if (report.status == MOTOR_WRITE_OK) servo_modes[id - 1U] = mode;
    return report;
}
uint16_t motor_device_register_lock_uncertain_mask(void){return 0;}
void uart_diagnostics_send(void *c,uart_diag_sender_t send){(void)c;(void)send;}
void uart_diagnostics_send_motor_buses(void *c, uart_diag_sender_t s,uint16_t q){(void)c;(void)s;(void)q;}
void pmw3901_spi_send_diagnostics(void *c,pmw_spi_diag_sender_t s,uint16_t q){(void)c;(void)s;(void)q;}
bool pmw3901_spi_auto_payload(uint8_t p[64]){(void)p;return false;}
void pmw3901_spi_auto_result(bool sent){(void)sent;}
bool sensor_get_motor_feedback_snapshot(motor_feedback_set_t *f){assert(!critical);memset(f,0,sizeof *f);
 f->sequence=tick/20+1;f->uptime_ms=tick;f->valid_mask=YPWM_ALL_MASK;for(unsigned i=0;i<12;++i){f->motor[i].valid=true;f->motor[i].position=2048;f->motor[i].voltage=60;}return true;}
bool motor_device_validate_targets(const motor_target_set_t *g){for(unsigned i=0;i<12;++i)if(g->motor[i].position>4095||g->motor[i].speed>32767)return false;return true;}
motor_device_status_t motor_device_write_targets(const motor_target_set_t *g){assert(!critical);native_target=*g;++native_writes;return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_set_torque(bool on){assert(!critical);if(on){torque_mask=YPWM_ALL_MASK;return MOTOR_DEVICE_OK;}if(!boot_off_ok)return MOTOR_DEVICE_UART_ERROR;torque_mask=0;return MOTOR_DEVICE_OK;}
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
 motor_control_service();service_pwm(c);assert(!c->pwm_mode_pending&&!motor_control_can_arm());
 for(uint8_t id=1;id<=12;++id){
  const uint8_t q[5]={id,0x21,1,1,backend==YPWM_PID?2:0};
  receive(c,MESSAGE_SET_MOTOR_REGISTER,q,5);assert(c->register_job_active);
  service_motor_register_write(c);assert(last_result==MESSAGE_ACK);
 }
 mode[1]=YPWM_COMMIT;receive(c,MESSAGE_SET_CONTROL_MODE,mode,9);assert(c->pwm_mode_pending);
 receive(c,MESSAGE_HEARTBEAT,NULL,0);assert(c->lease_established);
 motor_control_service();try_enter_control_ready(c);assert(c->state==HOST_FSM_DISARMED);
 service_pwm(c);assert(!c->pwm_mode_pending&&!c->lease_established&&torque_mask==0);
 try_enter_control_ready(c);assert(c->state==HOST_FSM_DISARMED);
}
static void transition(protocol_context_t *c,uint8_t backend,uint32_t epoch){
 configure(c,backend,epoch);
 receive(c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(c);assert(c->state==HOST_FSM_ARMED);
}
int main(void){
 seed_calibration();mode_memory_reset();
 protocol_context_t c={0};c.host_connected=true;c.state=HOST_FSM_DISARMED;c.last_host_heartbeat_tick=tick;published_state=HOST_FSM_DISARMED;
 motor_control_init();motor_control_service();assert(motor_control_position_ready());
 const uint8_t imu_status_request=0x01U, bad_sensor_flag=0x02U;
 receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&imu_status_request,1);
 assert(imu_tx_status==1U&&!imu_tx_valid);
 imu_ready=true;
 receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&imu_status_request,1);
 assert(imu_tx_status==2U&&!imu_tx_valid);
 imu_valid=true;
 receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&imu_status_request,1);
 assert(imu_tx_status==2U&&imu_tx_valid);
 receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,NULL,0);
 assert(imu_tx_status==0U&&imu_tx_valid);
 receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&bad_sensor_flag,1);
 assert(last_result==MESSAGE_NACK&&last_error==ERROR_INVALID_FLAGS);
 assert(!c.lease_established);
 /* Slow DISARMED configuration may outlive5s, but never ARM. A silent
    configuration still disconnects/rolls back at its own bounded20s window. */
 uint8_t slow[9]={YPWM_SCHEMA,YPWM_PREPARE,YPWM_PID};ypwm_put16(slow+3,YPWM_ALL_MASK);ypwm_put32(slow+5,118);
 receive(&c,MESSAGE_SET_CONTROL_MODE,slow,9);motor_control_service();service_pwm(&c);
 tick+=6000;expire_host_link(&c);try_enter_control_ready(&c);
 assert(c.host_connected&&c.state==HOST_FSM_DISARMED&&!motor_control_can_arm()&&on_count==0&&native_writes==0);
 tick+=YPWM_PREPARATION_LINK_TIMEOUT_MS;expire_host_link(&c);assert(!c.host_connected);
 service_safety_state(&c);motor_control_service();service_safety_state(&c);
 receive(&c,MESSAGE_HELLO,NULL,0);assert(c.host_connected);
 uint8_t legacy[8]={2,1};receive(&c,MESSAGE_SET_CONTROL_MODE,legacy,8);assert(last_result==MESSAGE_NACK&&!c.pwm_mode_pending);
 const uint8_t forbidden[5]={1,0x21,1,1,2};receive(&c,MESSAGE_SET_MOTOR_REGISTER,forbidden,5);assert(last_result==MESSAGE_NACK);
 /* Configure-only: read queries preserve the idle link for an arbitrary wait,
    but cannot establish a lease, ARM, turn torque on or send a native target. */
 configure(&c,YPWM_PID,120);
 for(unsigned i=0;i<12;++i){tick+=250;receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,NULL,0);
  try_enter_control_ready(&c);assert(c.host_connected&&!c.lease_established&&c.state==HOST_FSM_DISARMED);
  motor_control_service();assert(on_count==0&&native_writes==0&&motor_control_can_arm());}
 /* Losing the monitoring link before H still restores the previous setting. */
 tick+=1001;expire_host_link(&c);assert(!c.host_connected);service_safety_state(&c);motor_control_service();service_safety_state(&c);
 for(unsigned i=0;i<12;++i)assert(servo_modes[i]==0);
 receive(&c,MESSAGE_HELLO,NULL,0);assert(c.host_connected);
 transition(&c,YPWM_PID,123);
 /* Even continuous read traffic cannot renew an established control lease. */
 const TickType_t heartbeat_tick=c.last_host_heartbeat_tick;
 tick+=1;receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,NULL,0);assert(c.last_host_heartbeat_tick==heartbeat_tick);
 assert(diag_order&&diag_order<result_order&&last_diag[3]==YPWM_DIAG_COMPLETE);
 uint8_t targets[60]={0};for(unsigned i=0;i<12;++i){ypwm_put16(targets+5*i,2200);ypwm_put16(targets+5*i+2,200);}
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK&&on_count==0&&native_writes==0);
 motor_control_service();assert(on_count==1&&torque_mask==YPWM_ALL_MASK&&native_writes==0);
 for(unsigned i=0;i<12;++i){const int8_t expected[12]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
  assert(duty[i]*expected[i]>0&&duty[i]>=-YPWM_DUTY_LIMIT&&duty[i]<=YPWM_DUTY_LIMIT);}
 /* Malformed 0x0F is rejected; ordinary target/heartbeat watchdog stops PID. */
 receive(&c,0x0f,NULL,0);assert(last_result==MESSAGE_NACK);
 tick+=501;motor_control_service();service_safety_state(&c);assert(c.state==HOST_FSM_DISARMED);
 for(unsigned a=0;a<4;++a){tick+=20;motor_control_service();service_safety_state(&c);}
 assert(!host_protocol_is_stop_requested());
 for(unsigned i=0;i<12;++i)assert(duty[i]==0);
 /* A completed runtime stop already reverified the motor mode. */
 assert(motor_control_can_arm());
 const unsigned recover_programs=mode_memory_programs,recover_on=on_count;
 set_host_connected(&c,false);receive(&c,MESSAGE_HELLO,NULL,0);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED&&motor_control_can_arm());
 uint8_t resumed[YPWM_STATUS_LENGTH];assert(pwm_control_status(resumed,true));
 assert(resumed[7]==YPWM_REASON_NONE); /* PC beginConfiguredControl requires this */
 assert(!torque_mask&&on_count==recover_on&&mode_memory_programs==recover_programs);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED); /* no stale lease */
 receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_ARMED);
 assert(on_count==recover_on); /* no torque until an explicit new target */
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);motor_control_service();
 assert(on_count==recover_on+1);
 tick+=501;motor_control_service();service_safety_state(&c);
 /* HELLO during STOPPING cannot ARM or skip OFF. */
 set_host_connected(&c,false);receive(&c,MESSAGE_HELLO,NULL,0);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED&&!motor_control_can_arm());
 /* A physical mode mismatch during stop completion keeps torque OFF. */
 servo_modes[3]=0;
 for(unsigned a=0;a<4;++a){tick+=20;motor_control_service();service_safety_state(&c);}
 assert(!host_protocol_is_stop_requested());
 assert(!motor_control_can_arm()&&!torque_mask&&on_count==recover_on+1);
 servo_modes[3]=2;
 transition(&c,YPWM_POSITION,124);assert(motor_control_position_ready());
 uint8_t position_status[YPWM_STATUS_LENGTH];
 assert(pwm_control_status(position_status,true)&&position_status[2]==YPWM_POSITION&&position_status[3]==1U);
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);
 assert(last_result==MESSAGE_ACK&&native_writes==0);
 uint8_t newer[sizeof targets];memcpy(newer,targets,sizeof newer);
 ypwm_put16(newer,2100);ypwm_put16(newer+2,600);newer[4]=25;
 receive(&c,MESSAGE_SET_JOINT_TARGETS,newer,sizeof newer);
 assert(last_result==MESSAGE_ACK&&native_writes==0);
 motor_control_service();
 assert(native_writes==1&&native_target.motor[0].position==2100&&
        native_target.motor[0].speed==600&&native_target.motor[0].acceleration==25);
 published_state=HOST_FSM_DISARMED;c.state=HOST_FSM_DISARMED;
 for(unsigned a=0;a<4;++a){tick+=20;motor_control_service();service_safety_state(&c);}
 assert(!torque_mask&&motor_control_position_ready());
 const unsigned native_writes_at_stop=native_writes;
 receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_ARMED);
 motor_control_service();assert(native_writes==native_writes_at_stop&&!torque_mask); /* no stale goal after re-arm */
 published_state=HOST_FSM_DISARMED;c.state=HOST_FSM_DISARMED;mode_fail_id=2;
 uint8_t mode[9]={YPWM_SCHEMA,YPWM_PREPARE,YPWM_PID};ypwm_put16(mode+3,YPWM_ALL_MASK);ypwm_put32(mode+5,125);
 receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);motor_control_service();service_pwm(&c);
 const uint8_t q[5]={2,0x21,1,1,2};receive(&c,MESSAGE_SET_MOTOR_REGISTER,q,5);service_motor_register_write(&c);
 assert(last_result==MESSAGE_NACK&&last_error==15&&write_diag_order<result_order);
 uint8_t failure_detail[64],write_detail[64];bool has_write;
 assert(pwm_control_mode_diagnostics(failure_detail,write_detail,&has_write,false));
 assert(failure_detail[3]==YPWM_DIAG_MODE_TRANSACTION&&failure_detail[7]==2&&failure_detail[6]==YPWM_MODE_FAILED);
 motor_control_service();try_enter_control_ready(&c);assert(!motor_control_can_arm()&&c.state==HOST_FSM_DISARMED);
 /* Transient 0x0D failures wait safely for the PC's explicit CANCEL/retry. */
 mode[1]=YPWM_CANCEL;receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);
 motor_control_service();service_pwm(&c);assert(!motor_control_can_arm());
 assert(last_diag[3]==YPWM_DIAG_ROLLBACK&&last_diag[6]==0&&ypwm_u16(last_diag+16)==YPWM_ALL_MASK);
 /* Same-epoch CANCEL confirms already completed requested rollback. */
 mode[1]=YPWM_CANCEL;receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);
 tick+=YPWM_STATUS_PERIOD_MS;service_pwm(&c);
 assert(last_result==MESSAGE_ACK&&last_mode_status[3]==9U);
 assert(last_mode_status[6]==0&&ypwm_u32(last_mode_status+8)==ypwm_u32(mode+5));
 assert(ypwm_u16(last_mode_status+14)==YPWM_ALL_MASK&&ypwm_u16(last_mode_status+18)==0);
 for(unsigned i=0;i<12;++i)assert(servo_modes[i]==0);
 /* Reboot with a stored PID preference: HELLO cannot ARM; fresh heartbeat can
    start a verified new session without a fresh mode/EEPROM transaction. */
 const mode_setting_t saved={YPWM_PID,YPWM_ALL_MASK};assert(mode_store_save(&saved));
 for(unsigned i=0;i<12;++i)servo_modes[i]=2;
 const unsigned before_program=mode_memory_programs,before_on=on_count,before_native=native_writes;
 /* Boot ordering: actuator can run before host init; boot OFF must finish
    before verification grants READY and HELLO + heartbeat can arm. */
 memset(&c,0,sizeof c);c.state=HOST_FSM_BOOT;published_state=HOST_FSM_BOOT;
 pending_safety_action=HOST_SAFETY_ACTION_NONE;safe_stop_completed=requested_motor_stop=false;
 motor_control_init();motor_control_service(); assert(!motor_control_can_arm());
 uint8_t starting[YPWM_STATUS_LENGTH];assert(pwm_control_status(starting,true));
 assert(starting[3]==MOTOR_CONTROL_BOOT&&starting[2]==YPWM_PID);
 mode_memory_read_ok=false; /* The preference is read once on entering BOOT. */
 request_safe_stop();transition_state(&c,HOST_FSM_DISARMED,STATE_CAUSE_BOOT_COMPLETE,false);
 boot_off_ok=false;motor_control_service(); assert(!motor_control_can_arm());service_safety_state(&c);
 assert(host_protocol_is_stop_requested()); /* no readiness on failed OFF */
 boot_off_ok=true;motor_control_service(); assert(!motor_control_can_arm());service_safety_state(&c);
 assert(!host_protocol_is_stop_requested());
 motor_control_service(); assert(motor_control_can_arm());
 mode_memory_read_ok=true;
 receive(&c,MESSAGE_HELLO,NULL,0);receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);
 assert(c.state==HOST_FSM_ARMED); /* no F5 and no auto torque */
 memset(&c,0,sizeof c);c.state=HOST_FSM_DISARMED;published_state=HOST_FSM_DISARMED;
 pending_safety_action=HOST_SAFETY_ACTION_NONE;safe_stop_completed=requested_motor_stop=false;
 motor_control_init();receive(&c,MESSAGE_HELLO,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED);
 motor_control_service();receive(&c,MESSAGE_HELLO,NULL,0);try_enter_control_ready(&c);
 assert(c.state==HOST_FSM_DISARMED&&on_count==before_on&&mode_memory_programs==before_program);
 receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_ARMED&&on_count==before_on);
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK);motor_control_service();
 assert(on_count==before_on+1&&native_writes==before_native&&mode_memory_programs==before_program);
 /* Flash read failure at first boot has its own NACK, with no adopted epoch. */
 memset(&c,0,sizeof c);c.state=HOST_FSM_DISARMED;published_state=HOST_FSM_DISARMED;
 pending_safety_action=HOST_SAFETY_ACTION_NONE;safe_stop_completed=requested_motor_stop=false;
 mode_memory_read_ok=false;motor_control_init();receive(&c,MESSAGE_HELLO,NULL,0);
 mode[1]=YPWM_PREPARE;ypwm_put32(mode+5,456);
 motor_control_service();
 receive(&c,MESSAGE_SET_CONTROL_MODE,mode,9);
 assert(last_result==MESSAGE_NACK&&last_error==YPWM_FLASH_READ_FAILED&&!c.pwm_mode_pending);
 uint8_t boot_status[YPWM_STATUS_LENGTH];assert(pwm_control_status(boot_status,true));
 assert(boot_status[6]==YPWM_FLASH_READ_FAILED&&boot_status[7]==YPWM_REASON_FLASH_READ&&ypwm_u32(boot_status+8)==0);
 try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED&&!motor_control_can_arm());
 mode_memory_read_ok=true;motor_control_init();assert(pwm_control_status(boot_status,true)&&boot_status[6]==0&&ypwm_u32(boot_status+8)==0);
 /* Wrong backend rejects a complete PWM frame, even while DISARMED. */
 mode_memory_reset();motor_control_init();motor_control_service();c.host_connected=true;c.pwm_mode_pending=false;
 uint8_t raw_pwm[YPWM_DIRECT_LENGTH]={0};
 receive(&c,MESSAGE_SET_MOTOR_PWM,raw_pwm,sizeof raw_pwm);
 assert(last_result==MESSAGE_NACK&&last_error==YPWM_PWM_MODE_REQUIRED);
 receive(&c,MESSAGE_SET_MOTOR_PWM,raw_pwm,23);
 assert(last_result==MESSAGE_NACK&&last_error==ERROR_INVALID_LENGTH);
 mode_fail_id=0;configure(&c,YPWM_PID,789);receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);
 assert(c.state==HOST_FSM_ARMED);
 ypwm_put16(raw_pwm,(uint16_t)(int16_t)-777);ypwm_put16(raw_pwm+2,713);
 receive(&c,MESSAGE_SET_MOTOR_PWM,raw_pwm,sizeof raw_pwm);
 motor_control_service();service_pwm(&c);
 assert(last_result==MESSAGE_ACK&&last_error==0&&duty[0]==-777&&duty[1]==713);
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK);motor_control_service();
 assert(duty[0]!=-777); /* existing position->PID route resumes */
 /* Host pressure delays diagnostics; never pop a partial set before ACKs. */
 unsigned before_detail=detail_frames;c.result_count=1;service_pid_detail(&c);assert(detail_frames==before_detail);c.result_count=0;
 c.host_connected=false;service_pid_detail(&c);assert(detail_frames==before_detail);c.host_connected=true;
 for(unsigned pair=0;pair<6;++pair){service_pid_detail(&c);assert(detail_frames==before_detail+pair+1);}
 /* Continuous diagnostics still emit 1Hz producer/loss counters. */
 tick+=1000;service_pid_detail(&c);assert(stats_frames>0);
 puts("Host/task transaction, Flash isolation and direct PWM routing passed");return 0;
}

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *m, imu_feedback_t *i, pmw3901_feedback_t *f)
{memset(i,0,sizeof *i);i->ready=imu_ready;i->valid=imu_valid;memset(f,0,sizeof *f);return sensor_get_motor_feedback_snapshot(m);}
