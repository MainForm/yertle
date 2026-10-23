#include "pwm_control.h"
#include "motor_device.h"
#include "sensor_task.h"
#include "host_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
typedef struct {
 uint8_t backend,phase,reason,result;uint16_t mask,verified,unknown,torque,request;
 uint32_t epoch,retired_epoch,last_command,accepted_ms,start_ms,last_service,status_ms;
 uint16_t ttl,feedback_age;int16_t demand[12],sent[12],anchor;
 bool result_pending,arm_allowed,dirty,ever_command;
} pwm_context;
static pwm_context p;
static uint32_t milliseconds(void){return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);}
static pwm_context snapshot(void){pwm_context s;taskENTER_CRITICAL();s=p;taskEXIT_CRITICAL();return s;}
void pwm_control_init(void){taskENTER_CRITICAL();memset(&p,0,sizeof p);p.unknown=YPWM_ALL_MASK;p.phase=YPWM_UNVERIFIED;p.dirty=true;taskEXIT_CRITICAL();}
bool pwm_control_native_ready(void){const pwm_context s=snapshot();return s.backend==YPWM_POSITION&&s.phase==YPWM_NATIVE_READY;}
bool pwm_control_can_arm(void){const pwm_context s=snapshot();return pwm_control_native_ready()||
 (s.backend==YPWM_RAW&&s.phase==YPWM_READY&&s.arm_allowed);}
void pwm_control_stop(uint8_t reason){
 taskENTER_CRITICAL();
 if(p.phase!=YPWM_STOPPING){const bool preparing=p.phase==YPWM_PREPARING;
   p.phase=YPWM_STOPPING;p.reason=reason;p.arm_allowed=false;
   if(p.epoch)p.retired_epoch=p.epoch;
   p.epoch=0;p.dirty=true;
   /* Complete a canceled asynchronous request as failure, never leave the host busy. */
   if(preparing&&!p.result_pending){p.result=YPWM_INVALID_STATE;p.result_pending=true;}}
 taskEXIT_CRITICAL();host_protocol_request_pwm_stop();
}
void pwm_control_connection_reset(void){const pwm_context s=snapshot();
 if(s.phase==YPWM_PREPARING||(s.backend==YPWM_RAW&&
    (s.phase==YPWM_READY||s.phase==YPWM_ACTIVE||(s.phase==YPWM_FAULT&&s.unknown))))
   pwm_control_stop(YPWM_REASON_LINK);}
uint8_t pwm_control_request_mode(const uint8_t *bytes,size_t n,uint16_t sequence){
 ypwm_mode_request r;if(!ypwm_decode_mode(bytes,n,&r))return YPWM_INVALID_PAYLOAD;
 if(host_protocol_get_state()!=HOST_FSM_DISARMED||host_protocol_is_stop_requested())return YPWM_INVALID_STATE;
 taskENTER_CRITICAL();
 if(p.phase==YPWM_PREPARING||p.phase==YPWM_ACTIVE||p.phase==YPWM_STOPPING||p.result_pending){taskEXIT_CRITICAL();return YPWM_BUSY;}
 /* A replay cannot reset travel/session limits or repeat EEPROM writes. */
 if(r.epoch==p.epoch||r.epoch==p.retired_epoch){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
 p.backend=r.backend;p.mask=r.mask;p.epoch=r.epoch;p.request=sequence;p.phase=YPWM_PREPARING;
 p.verified=0;p.reason=0;p.result=0;p.last_command=0;p.arm_allowed=false;p.ever_command=false;
 memset(p.demand,0,sizeof p.demand);memset(p.sent,0,sizeof p.sent);p.dirty=true;
 taskEXIT_CRITICAL();return YPWM_OK;
}
uint8_t pwm_control_submit(const uint8_t *bytes,size_t n){
 ypwm_target r;if(!ypwm_decode_target(bytes,n,&r))return YPWM_INVALID_PAYLOAD;
 if(r.flags==YPWM_RELEASE){pwm_control_stop(YPWM_REASON_RELEASE);return YPWM_OK;}
 if(!host_protocol_is_armed()||host_protocol_is_stop_requested())return YPWM_INVALID_STATE;
 taskENTER_CRITICAL();
 if(p.backend!=YPWM_RAW||!p.arm_allowed||(p.phase!=YPWM_READY&&p.phase!=YPWM_ACTIVE)){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
 if(r.epoch!=p.epoch||r.mask!=p.mask){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
 if(r.command<=p.last_command){taskEXIT_CRITICAL();return YPWM_OLD_COMMAND;}
 p.last_command=r.command;p.ttl=r.ttl_ms;p.accepted_ms=milliseconds();p.ever_command=true;
 memcpy(p.demand,r.pwm,sizeof p.demand);p.dirty=true;taskEXIT_CRITICAL();return YPWM_OK;
}
bool pwm_control_take_mode_result(uint16_t *sequence,uint8_t *result){
 taskENTER_CRITICAL();const bool ready=p.result_pending;
 if(ready){*sequence=p.request;*result=p.result;p.result_pending=false;
   p.arm_allowed=p.result==YPWM_OK&&p.backend==YPWM_RAW&&p.phase==YPWM_READY;}
 taskEXIT_CRITICAL();return ready;
}
bool pwm_control_status(uint8_t b[YPWM_STATUS_LENGTH],bool force){
 const uint32_t now=milliseconds();taskENTER_CRITICAL();
 if(!force&&(!p.dirty||now-p.status_ms<YPWM_STATUS_PERIOD_MS)){taskEXIT_CRITICAL();return false;}
 const pwm_context s=p;p.dirty=false;p.status_ms=now;taskEXIT_CRITICAL();
 memset(b,0,YPWM_STATUS_LENGTH);b[0]=YPWM_EVENT_CODE;b[1]=YPWM_SCHEMA;b[2]=s.backend;b[3]=s.phase;
 ypwm_put16(b+4,s.request);b[6]=s.result;b[7]=s.reason;ypwm_put32(b+8,s.epoch);
 ypwm_put16(b+12,s.mask);ypwm_put16(b+14,s.verified);ypwm_put16(b+16,s.torque);ypwm_put16(b+18,s.unknown);
 ypwm_put32(b+20,s.last_command);ypwm_put16(b+24,YPWM_DUTY_LIMIT);ypwm_put16(b+26,YPWM_TTL_MAX_MS);
 ypwm_put16(b+28,YPWM_TRAVEL_LIMIT);ypwm_put16(b+30,s.feedback_age);
 for(unsigned i=0;i<12;++i)if(s.mask&(1U<<i))ypwm_put16(b+32,(uint16_t)s.sent[i]);
 ypwm_put32(b+34,now);return true;
}
static bool mode_allowed(void){const pwm_context s=snapshot();return s.phase==YPWM_PREPARING&&
 host_protocol_get_state()==HOST_FSM_DISARMED&&!host_protocol_is_stop_requested();}
static unsigned selected(uint16_t mask){unsigned i=0;while(i<12&&!(mask&(1U<<i)))++i;return i;}
static int32_t magnitude(int16_t v){return v<0?-(int32_t)v:(int32_t)v;}
static void finish_mode(uint32_t epoch,uint8_t error,uint16_t verified,uint16_t unknown){
 taskENTER_CRITICAL();
 if(p.phase==YPWM_PREPARING&&p.epoch==epoch){
   p.result=error;p.verified=verified;p.unknown=unknown;p.torque=0;p.result_pending=true;p.dirty=true;
   p.phase=error?YPWM_FAULT:(p.backend==YPWM_RAW?YPWM_READY:YPWM_NATIVE_READY);
 }
 taskEXIT_CRITICAL();
}
bool pwm_control_service(void){
 pwm_context s=snapshot();const uint32_t now=milliseconds();
 if(s.phase==YPWM_NATIVE_READY)return false;
 if(s.phase==YPWM_UNVERIFIED){
   /* Never assume EEPROM position mode after an MCU-only reset. No mode writes. */
   uint16_t unknown=YPWM_ALL_MASK,verified=0;
   const bool off=motor_device_set_torque(false)==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
   const bool native=off&&motor_device_verify_modes(YPWM_POSITION,YPWM_ALL_MASK,&verified);
   taskENTER_CRITICAL();if(p.phase==YPWM_UNVERIFIED){p.unknown=unknown;p.verified=verified;
     p.phase=native?YPWM_NATIVE_READY:YPWM_FAULT;p.dirty=true;}taskEXIT_CRITICAL();return true;
 }
 if(s.phase==YPWM_PREPARING){
   uint16_t unknown=YPWM_ALL_MASK,verified=0;uint8_t error=YPWM_OK;
   if(motor_device_set_torque(false)!=MOTOR_DEVICE_OK||!motor_device_verify_torque_off(&unknown))error=YPWM_MODE_FAILED;
   for(uint8_t id=1;id<=12&&!error;++id){
     const uint8_t mode=(s.backend==YPWM_RAW&&(s.mask&(1U<<(id-1U))))?2U:0U;
     const motor_register_write_report_t r=motor_device_write_mode(id,mode,mode_allowed);
     if(r.status!=MOTOR_WRITE_OK)error=YPWM_MODE_FAILED;
   }
   if(!error&&!motor_device_verify_modes(s.backend,s.mask,&verified))error=YPWM_MODE_FAILED;
   if(!error&&s.backend==YPWM_RAW){const int16_t zero[12]={0};if(motor_device_write_pwm(s.mask,zero)!=MOTOR_DEVICE_OK)error=YPWM_MODE_FAILED;}
   if(!motor_device_verify_torque_off(&unknown))error=YPWM_MODE_FAILED;
   finish_mode(s.epoch,error,verified,unknown);return true;
 }
 if((s.phase==YPWM_READY||s.phase==YPWM_ACTIVE)&&
   (host_protocol_get_state()==HOST_FSM_FAULT||host_protocol_is_stop_requested()))pwm_control_stop(YPWM_REASON_LINK);
 s=snapshot();
 if(s.phase==YPWM_STOPPING){
   const int16_t zero[12]={0};
   /* OFF is attempted even if zero TX fails. Never enable or send position2048. */
   (void)motor_device_write_pwm(YPWM_ALL_MASK,zero);
   const motor_device_status_t tx=motor_device_torque_mask(YPWM_ALL_MASK,false);
   uint16_t unknown=YPWM_ALL_MASK;const bool off=tx==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
   taskENTER_CRITICAL();p.unknown=unknown;if(off){p.torque=0;memset(p.sent,0,sizeof p.sent);}p.dirty=true;taskEXIT_CRITICAL();
   if(off&&!host_protocol_is_armed()){
     taskENTER_CRITICAL();p.phase=YPWM_FAULT;p.ever_command=false;taskEXIT_CRITICAL();
     host_protocol_complete_safe_stop();
   }return true;
 }
 if(s.phase==YPWM_FAULT&&s.unknown){
   /* A failed boot/prepare cannot strand an unconfirmed powered servo. */
   (void)motor_device_torque_mask(YPWM_ALL_MASK,false);
   uint16_t unknown=YPWM_ALL_MASK;(void)motor_device_verify_torque_off(&unknown);
   taskENTER_CRITICAL();p.unknown=unknown;p.dirty=true;taskEXIT_CRITICAL();
   if(!unknown&&!host_protocol_is_armed())host_protocol_complete_safe_stop();
   return true;
 }
 if(s.phase==YPWM_FAULT&&!s.unknown&&!host_protocol_is_armed()&&host_protocol_is_stop_requested())
   host_protocol_complete_safe_stop();
 if(s.phase!=YPWM_READY&&s.phase!=YPWM_ACTIVE)return true;
 if(!s.ever_command)return true;
 if(now-s.accepted_ms>=s.ttl){pwm_control_stop(YPWM_REASON_TTL);return true;}
 if(s.phase==YPWM_ACTIVE&&now-s.start_ms>=YPWM_SESSION_MAX_MS){pwm_control_stop(YPWM_REASON_SESSION);return true;}
 if(s.phase==YPWM_ACTIVE&&now-s.last_service>30U){pwm_control_stop(YPWM_REASON_DEADLINE);return true;}
 motor_feedback_set_t f;imu_feedback_t imu;
 if(!sensor_get_feedback_snapshot(&f,&imu)||f.valid_mask!=YPWM_ALL_MASK||now-f.uptime_ms>YPWM_FEEDBACK_MAX_MS){pwm_control_stop(YPWM_REASON_SENSOR);return true;}
 int32_t sum=0;const unsigned axis=selected(s.mask);
 for(unsigned i=0;i<12;++i){const motor_feedback_t *m=&f.motor[i];
   if(!m->valid||m->servo_error||m->position<0||m->position>4095){pwm_control_stop(YPWM_REASON_SENSOR);return true;}
   const int32_t current=magnitude(m->current);sum+=current;
   if(current>=308){host_protocol_latch_unsafe(HOST_UNSAFE_CURRENT_LIMIT);pwm_control_stop(YPWM_REASON_CURRENT);return true;}
   if(m->temperature>=60||m->voltage<50||m->voltage>74){pwm_control_stop(YPWM_REASON_THERMAL_VOLTAGE);return true;}
 }
 if(sum>=924){host_protocol_latch_unsafe(HOST_UNSAFE_CURRENT_LIMIT);pwm_control_stop(YPWM_REASON_CURRENT);return true;}
 if(axis>=12){pwm_control_stop(YPWM_REASON_MODE);return true;}
 if(magnitude(f.motor[axis].speed)>500){pwm_control_stop(YPWM_REASON_SPEED);return true;}
 const int16_t anchor=s.phase==YPWM_READY?f.motor[axis].position:s.anchor;
 if(magnitude((int16_t)(f.motor[axis].position-anchor))>(int32_t)YPWM_TRAVEL_LIMIT){pwm_control_stop(YPWM_REASON_TRAVEL);return true;}
 int16_t out[12]={0};int32_t delta=(int32_t)s.demand[axis]-s.sent[axis];
 if(delta>YPWM_SLEW_PER_TICK)delta=YPWM_SLEW_PER_TICK;
 if(delta< -YPWM_SLEW_PER_TICK)delta= -YPWM_SLEW_PER_TICK;
 out[axis]=(int16_t)(s.sent[axis]+delta);
 /* Recheck cancellation immediately before writes. A later stop is handled next tick. */
 if(snapshot().phase!=s.phase||host_protocol_is_stop_requested())return true;
 if(motor_device_write_pwm(s.mask,out)!=MOTOR_DEVICE_OK){pwm_control_stop(YPWM_REASON_DRIVER);return true;}
 if(snapshot().phase!=s.phase||host_protocol_is_stop_requested())return true;
 if(s.phase==YPWM_READY){
   taskENTER_CRITICAL();p.unknown|=s.mask;taskEXIT_CRITICAL();
   if(motor_device_torque_mask(s.mask,true)!=MOTOR_DEVICE_OK){pwm_control_stop(YPWM_REASON_DRIVER);return true;}
   if(snapshot().phase!=s.phase||host_protocol_is_stop_requested()){
     const int16_t zero[12]={0};(void)motor_device_write_pwm(s.mask,zero);
     (void)motor_device_torque_mask(YPWM_ALL_MASK,false);return true;
   }
 }
 taskENTER_CRITICAL();if(p.phase==s.phase&&p.epoch==s.epoch){
   if(s.phase==YPWM_READY){p.start_ms=now;p.anchor=anchor;}p.phase=YPWM_ACTIVE;p.last_service=now;
   p.torque=s.mask;p.unknown=s.mask;p.feedback_age=(uint16_t)(now-f.uptime_ms);memcpy(p.sent,out,sizeof out);p.dirty=true;
 }taskEXIT_CRITICAL();return true;
}
