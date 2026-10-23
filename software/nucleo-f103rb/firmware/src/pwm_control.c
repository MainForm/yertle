#include "pwm_control.h"
#include "position_pid.h"
#include "sensor_task.h"
#include "host_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
typedef struct {
    uint8_t backend,phase,reason,result;
    uint16_t mask,verified,unknown,torque,request,feedback_age;
    uint32_t epoch,retired_epoch,command_count,accepted_ms,last_service,status_ms;
    motor_target_set_t goal;
    int16_t sent[12];
    bool result_pending,arm_allowed,dirty,ever_command;
} control_context;
static control_context p;
/* These numerical states belong exclusively to the actuator task. */
static position_pid_t axes[12];
static uint32_t last_sample,last_feedback_ms;
static uint32_t milliseconds(void){return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);}
static control_context snapshot(void){control_context s;taskENTER_CRITICAL();s=p;taskEXIT_CRITICAL();return s;}
void pwm_control_init(void){taskENTER_CRITICAL();memset(&p,0,sizeof p);p.unknown=YPWM_ALL_MASK;
    p.phase=YPWM_UNVERIFIED;p.dirty=true;taskEXIT_CRITICAL();}
bool pwm_control_native_ready(void){const control_context s=snapshot();return s.backend==YPWM_POSITION&&s.phase==YPWM_NATIVE_READY;}
bool pwm_control_accepts_positions(void){const control_context s=snapshot();return
    (s.backend==YPWM_POSITION&&s.phase==YPWM_NATIVE_READY)||
    (s.backend==YPWM_PID&&s.arm_allowed&&(s.phase==YPWM_READY||s.phase==YPWM_ACTIVE));}
bool pwm_control_can_arm(void){const control_context s=snapshot();return
    (s.backend==YPWM_POSITION&&s.phase==YPWM_NATIVE_READY)||
    (s.backend==YPWM_PID&&s.phase==YPWM_READY&&s.arm_allowed);}
void pwm_control_stop(uint8_t reason){
    taskENTER_CRITICAL();
    if(p.phase!=YPWM_STOPPING){const bool preparing=p.phase==YPWM_PREPARING;
        p.phase=YPWM_STOPPING;p.reason=reason;p.arm_allowed=false;
        if(p.epoch)p.retired_epoch=p.epoch;
        p.epoch=0;p.dirty=true;
        if(preparing||p.result_pending){p.result=YPWM_INVALID_STATE;p.result_pending=true;}}
    taskEXIT_CRITICAL();host_protocol_request_pwm_stop();
}
void pwm_control_connection_reset(void){const control_context s=snapshot();
    if(s.phase==YPWM_PREPARING||(s.backend==YPWM_PID&&
        (s.phase==YPWM_READY||s.phase==YPWM_ACTIVE||(s.phase==YPWM_FAULT&&s.unknown))))
        pwm_control_stop(YPWM_REASON_LINK);
}
uint8_t pwm_control_request_mode(const uint8_t *bytes,size_t n,uint16_t sequence){
    ypwm_mode_request r;if(!ypwm_decode_mode(bytes,n,&r))return YPWM_INVALID_PAYLOAD;
    if(host_protocol_get_state()!=HOST_FSM_DISARMED||host_protocol_is_stop_requested())return YPWM_INVALID_STATE;
    taskENTER_CRITICAL();
    if(p.phase==YPWM_PREPARING||p.phase==YPWM_ACTIVE||p.phase==YPWM_STOPPING||p.result_pending){taskEXIT_CRITICAL();return YPWM_BUSY;}
    if(r.epoch==p.epoch||r.epoch==p.retired_epoch){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
    p.backend=r.backend;p.mask=r.mask;p.epoch=r.epoch;p.request=sequence;p.phase=YPWM_PREPARING;
    p.verified=0;p.reason=0;p.result=0;p.command_count=0;p.arm_allowed=false;p.ever_command=false;
    memset(&p.goal,0,sizeof p.goal);memset(p.sent,0,sizeof p.sent);p.dirty=true;
    taskEXIT_CRITICAL();return YPWM_OK;
}
uint8_t pwm_control_release(const uint8_t *bytes,size_t n){
    if(!ypwm_decode_release(bytes,n))return YPWM_INVALID_PAYLOAD;
    pwm_control_stop(YPWM_REASON_RELEASE);return YPWM_OK;
}
bool pwm_control_submit_positions(const motor_target_set_t *goal){
    if(!goal||!motor_device_validate_targets(goal)||!host_protocol_is_armed()||host_protocol_is_stop_requested())return false;
    taskENTER_CRITICAL();
    if(p.backend!=YPWM_PID||!p.arm_allowed||(p.phase!=YPWM_READY&&p.phase!=YPWM_ACTIVE)){taskEXIT_CRITICAL();return false;}
    for(unsigned i=0;i<12;++i)if((p.mask&(1U<<i))&&!goal->motor[i].speed){taskEXIT_CRITICAL();return false;}
    /* New position goals do not reset the integrator or derivative history. */
    p.goal=*goal;p.accepted_ms=milliseconds();++p.command_count;p.ever_command=true;p.dirty=true;
    taskEXIT_CRITICAL();return true;
}
bool pwm_control_take_mode_result(uint16_t *sequence,uint8_t *result){
    taskENTER_CRITICAL();const bool ready=p.result_pending;
    if(ready){*sequence=p.request;*result=p.result;p.result_pending=false;
        p.arm_allowed=p.result==YPWM_OK&&p.backend==YPWM_PID&&p.phase==YPWM_READY;}
    taskEXIT_CRITICAL();return ready;
}
bool pwm_control_status(uint8_t b[YPWM_STATUS_LENGTH],bool force){
    const uint32_t now=milliseconds();taskENTER_CRITICAL();
    if(!force&&(!p.dirty||now-p.status_ms<YPWM_STATUS_PERIOD_MS)){taskEXIT_CRITICAL();return false;}
    const control_context s=p;p.dirty=false;p.status_ms=now;taskEXIT_CRITICAL();
    memset(b,0,YPWM_STATUS_LENGTH);b[0]=YPWM_EVENT_CODE;b[1]=YPWM_SCHEMA;b[2]=s.backend;b[3]=s.phase;
    ypwm_put16(b+4,s.request);b[6]=s.result;b[7]=s.reason;ypwm_put32(b+8,s.epoch);
    ypwm_put16(b+12,s.mask);ypwm_put16(b+14,s.verified);ypwm_put16(b+16,s.torque);ypwm_put16(b+18,s.unknown);
    ypwm_put32(b+20,s.command_count);ypwm_put16(b+24,YPWM_DUTY_LIMIT);ypwm_put16(b+26,YPWM_TARGET_TIMEOUT_MS);
    ypwm_put16(b+28,YPWM_FOLLOWING_ERROR_LIMIT);ypwm_put16(b+30,s.feedback_age);
    for(unsigned i=0;i<12;++i)if(s.mask&(1U<<i)){ypwm_put16(b+32,(uint16_t)s.sent[i]);break;}
    ypwm_put32(b+34,now);return true;
}
static bool mode_allowed(void){const control_context s=snapshot();return s.phase==YPWM_PREPARING&&
    host_protocol_get_state()==HOST_FSM_DISARMED&&!host_protocol_is_stop_requested();}
static void prepare_mode(const control_context *s){
    uint16_t unknown=YPWM_ALL_MASK,verified=0;uint8_t error=YPWM_OK;
    if(motor_device_set_torque(false)!=MOTOR_DEVICE_OK||!motor_device_verify_torque_off(&unknown))error=YPWM_MODE_FAILED;
    for(uint8_t id=1;id<=12&&!error;++id){
        const uint8_t mode=(s->backend==YPWM_PID&&(s->mask&(1U<<(id-1U))))?2U:0U;
        const motor_register_write_report_t r=motor_device_write_mode(id,mode,mode_allowed);
        if(r.status!=MOTOR_WRITE_OK)error=YPWM_MODE_FAILED;
    }
    if(!error&&!motor_device_verify_modes(s->backend,s->mask,&verified))error=YPWM_MODE_FAILED;
    if(!error&&s->backend==YPWM_PID){const int16_t zero[12]={0};
        if(motor_device_write_pwm(s->mask,zero)!=MOTOR_DEVICE_OK)error=YPWM_MODE_FAILED;}
    if(!motor_device_verify_torque_off(&unknown))error=YPWM_MODE_FAILED;
    memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;
    taskENTER_CRITICAL();
    if(p.phase==YPWM_PREPARING&&p.epoch==s->epoch){
        p.result=error;p.verified=verified;p.unknown=unknown;p.torque=0;p.result_pending=true;p.dirty=true;
        p.phase=error?YPWM_FAULT:(p.backend==YPWM_PID?YPWM_READY:YPWM_NATIVE_READY);
    }
    taskEXIT_CRITICAL();
}
static void service_stop(const control_context *s){
    const int16_t zero[12]={0};bool zero_sent=false;memset(axes,0,sizeof axes);
    /* 0x2c must never be written to native/unverified axes. */
    if(s->backend==YPWM_PID&&ypwm_mask(s->mask)&&(s->verified&s->mask)==s->mask)
        zero_sent=motor_device_write_pwm(s->mask,zero)==MOTOR_DEVICE_OK;
    const motor_device_status_t tx=motor_device_torque_mask(YPWM_ALL_MASK,false);
    uint16_t unknown=YPWM_ALL_MASK;const bool off=tx==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
    taskENTER_CRITICAL();p.unknown=unknown;if(off){p.torque=0;if(zero_sent)memset(p.sent,0,sizeof p.sent);}p.dirty=true;
    if(off&&!host_protocol_is_armed()){p.phase=YPWM_FAULT;p.ever_command=false;}
    taskEXIT_CRITICAL();if(off&&!host_protocol_is_armed())host_protocol_complete_safe_stop();
}
static bool feedback_safe(const motor_feedback_set_t *f,uint32_t now){
    if(f->valid_mask!=YPWM_ALL_MASK||!f->sequence||now-f->uptime_ms>YPWM_FEEDBACK_MAX_MS){pwm_control_stop(YPWM_REASON_SENSOR);return false;}
    /* Current thresholds belong to the PC CLI. Preserve current telemetry and
       servo-reported fault checks, but do not latch a local current trip. */
    for(unsigned i=0;i<12;++i){const motor_feedback_t *m=&f->motor[i];
        if(!m->valid||m->servo_error||m->position<0||m->position>4095){pwm_control_stop(YPWM_REASON_SENSOR);return false;}
        if(m->temperature>=60||m->voltage<50||m->voltage>74){pwm_control_stop(YPWM_REASON_THERMAL_VOLTAGE);return false;}
    }
    return true;
}
static void service_pid(const control_context *s,uint32_t now){
    if(!s->ever_command)return;
    if(now-s->accepted_ms>=YPWM_TARGET_TIMEOUT_MS){pwm_control_stop(YPWM_REASON_TTL);return;}
    if(s->phase==YPWM_ACTIVE&&now-s->last_service>30U){pwm_control_stop(YPWM_REASON_DEADLINE);return;}
    taskENTER_CRITICAL();p.last_service=now;taskEXIT_CRITICAL();
    motor_feedback_set_t f;imu_feedback_t imu;
    if(!sensor_get_feedback_snapshot(&f,&imu)){pwm_control_stop(YPWM_REASON_SENSOR);return;}
    if(!feedback_safe(&f,now))return;
    /* Never integrate/differentiate the same measurement more than once. */
    if(s->phase==YPWM_ACTIVE&&f.sequence==last_sample)return;
    const uint32_t dt_ms=s->phase==YPWM_READY?20U:f.uptime_ms-last_feedback_ms;
    if(!dt_ms||dt_ms>50U){pwm_control_stop(YPWM_REASON_SENSOR);return;}
    const float dt=(float)dt_ms*0.001f;
    const position_pid_config_t config=position_pid_default_config();int16_t out[12]={0};
    for(unsigned i=0;i<12;++i)if(s->mask&(1U<<i)){
        if(!position_pid_step(&axes[i],&config,s->goal.motor[i].position,s->goal.motor[i].speed,
            s->goal.motor[i].acceleration,f.motor[i].position,dt,&out[i])){pwm_control_stop(YPWM_REASON_TRAVEL);return;}
    }
    if(snapshot().phase!=s->phase||host_protocol_is_stop_requested())return;
    if(motor_device_write_pwm(s->mask,out)!=MOTOR_DEVICE_OK){pwm_control_stop(YPWM_REASON_DRIVER);return;}
    if(snapshot().phase!=s->phase||host_protocol_is_stop_requested())return;
    if(s->phase==YPWM_READY){
        taskENTER_CRITICAL();p.unknown|=s->mask;taskEXIT_CRITICAL();
        if(motor_device_torque_mask(s->mask,true)!=MOTOR_DEVICE_OK){pwm_control_stop(YPWM_REASON_DRIVER);return;}
        if(snapshot().phase!=s->phase||host_protocol_is_stop_requested()){
            const int16_t zero[12]={0};(void)motor_device_write_pwm(s->mask,zero);
            (void)motor_device_torque_mask(YPWM_ALL_MASK,false);return;
        }
    }
    last_sample=f.sequence;last_feedback_ms=f.uptime_ms;
    taskENTER_CRITICAL();if(p.phase==s->phase&&p.epoch==s->epoch){p.phase=YPWM_ACTIVE;p.torque=s->mask;p.unknown=s->mask;
        p.feedback_age=(uint16_t)(now-f.uptime_ms);memcpy(p.sent,out,sizeof out);p.dirty=true;}taskEXIT_CRITICAL();
}
bool pwm_control_service(void){
    control_context s=snapshot();const uint32_t now=milliseconds();
    if(s.phase==YPWM_NATIVE_READY)return false;
    if(s.phase==YPWM_UNVERIFIED){
        uint16_t unknown=YPWM_ALL_MASK,verified=0;
        const bool off=motor_device_set_torque(false)==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
        const bool native=off&&motor_device_verify_modes(YPWM_POSITION,YPWM_ALL_MASK,&verified);
        taskENTER_CRITICAL();if(p.phase==YPWM_UNVERIFIED){p.unknown=unknown;p.verified=verified;
            p.phase=native?YPWM_NATIVE_READY:YPWM_FAULT;p.dirty=true;}taskEXIT_CRITICAL();return true;
    }
    if(s.phase==YPWM_PREPARING){prepare_mode(&s);return true;}
    if((s.phase==YPWM_READY||s.phase==YPWM_ACTIVE)&&
        (host_protocol_get_state()==HOST_FSM_FAULT||host_protocol_is_stop_requested()||
         (s.phase==YPWM_ACTIVE&&!host_protocol_is_armed())))pwm_control_stop(YPWM_REASON_LINK);
    s=snapshot();
    if(s.phase==YPWM_STOPPING){service_stop(&s);return true;}
    if(s.phase==YPWM_FAULT&&s.unknown){
        (void)motor_device_torque_mask(YPWM_ALL_MASK,false);
        uint16_t unknown=YPWM_ALL_MASK;(void)motor_device_verify_torque_off(&unknown);
        taskENTER_CRITICAL();p.unknown=unknown;p.dirty=true;taskEXIT_CRITICAL();
        if(!unknown&&!host_protocol_is_armed())host_protocol_complete_safe_stop();
        return true;
    }
    if(s.phase==YPWM_FAULT&&!s.unknown&&!host_protocol_is_armed()&&host_protocol_is_stop_requested())host_protocol_complete_safe_stop();
    if(s.phase==YPWM_READY||s.phase==YPWM_ACTIVE)service_pid(&s,now);
    return true;
}
