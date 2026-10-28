#include "pwm_control.h"
#include "position_pid.h"
#include "mode_store.h"
#include "calibration_store.h"
#include "sensor_task.h"
#include "host_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "register_write_diagnostics.h"
typedef struct {
    uint8_t backend,phase,reason,result,operation,mode_write_id;
    uint16_t mask,verified,unknown,torque,request,feedback_age;
    uint32_t epoch,retired_epoch,command_count,accepted_ms,status_ms,started_ms;
    motor_target_set_t goal;
    int16_t sent[12];
    int16_t raw[12];uint16_t raw_request;uint8_t raw_result;
    bool direct,raw_pending,raw_result_pending,raw_preview,pid_restart,return_eligible;
    bool result_pending,arm_allowed,dirty,ever_command,mode_busy,transaction_pending,snapshot_valid,rollback_failed;
    bool preference_touched,boot_restored,flash_read_failed;
    bool reconnect_stop_recoverable,reconnect_recovery_requested;
} control_context;
static control_context p;
static motor_mode_snapshot_t previous_modes;
static mode_setting_t previous_setting;
/* Kept out of snapshot() to avoid enlarging every actuator stack frame. */
static uint8_t mode_detail[YPWM_DIAG_LENGTH],mode_write_detail[64],rollback_detail[YPWM_DIAG_LENGTH];
static bool mode_detail_valid,mode_detail_pending,mode_detail_has_write,rollback_detail_pending;
/* These numerical states belong exclusively to the actuator task. */
static position_pid_t axes[12];
static uint32_t last_sample,last_feedback_ms;
/* Actuator-owned: one snapshot cannot count as several hot samples. */
static uint32_t thermal_sequence;
static uint8_t hot_samples[12];
/* Actuator-owned shutdown. Repeated host/fault requests must not restart it. */
static struct {
    bool initialized,off_only;
    uint8_t settled,outcome;
    uint16_t max_error;
    uint32_t started,last_sequence,last_stamp,last_progress[12];
    uint16_t best_error[12];
} returning;
/* EVENT9/v1: stage1 returning,2 centered,3 ineligible,4 feedback,
 * 5 voltage,6 repeated-hot,7 stalled,8 timeout,9 PID,10 motor TX.
 * Existing EVENT envelope, no new command message or motor transaction. */
static uint8_t stop_detail[20];static bool stop_detail_pending;
static uint32_t stop_detail_ms;
static void publish_stop(const control_context *s,uint32_t now,bool force){
    if(!force&&now-stop_detail_ms<200U)return;
    taskENTER_CRITICAL();stop_detail_ms=now;
    stop_detail[0]=9;stop_detail[1]=1;stop_detail[2]=returning.outcome;stop_detail[3]=s->reason;
    ypwm_put32(stop_detail+4,now);ypwm_put32(stop_detail+8,now-returning.started);
    ypwm_put16(stop_detail+12,returning.max_error);stop_detail[14]=returning.settled;stop_detail[15]=0;
    ypwm_put16(stop_detail+16,s->torque);ypwm_put16(stop_detail+18,s->unknown);stop_detail_pending=true;
    taskEXIT_CRITICAL();
}
bool pwm_control_stop_detail(uint8_t b[20]){
    taskENTER_CRITICAL();const bool available=stop_detail_pending;
    if(available){memcpy(b,stop_detail,20);stop_detail_pending=false;}
    taskEXIT_CRITICAL();return available;
}
static bool temperature_trip(const motor_feedback_set_t *f){
    if(f->sequence==thermal_sequence)return false;
    if(!thermal_sequence||f->sequence-thermal_sequence!=1U)memset(hot_samples,0,sizeof hot_samples);
    thermal_sequence=f->sequence;
    bool trip=false;
    for(unsigned i=0;i<12;++i){
        if(f->motor[i].temperature<60U)hot_samples[i]=0;
        else if(hot_samples[i]<3U)++hot_samples[i];
        if(hot_samples[i]>=3U)trip=true;
    }
    return trip;
}
static uint32_t milliseconds(void){return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);}
/* Two bounded snapshots: actuator publishes latest, host drains a frozen set.
 * No allocation, motor queries, queue wait or host UART from the actuator.
 * Slow host consumes newest pending set; counted coalescing cannot stop PID. */
typedef struct { uint8_t frame[6][YPWM_PID_DETAIL_LENGTH]; } pid_trace_set;
static pid_trace_set trace_latest,trace_active;
static bool trace_pending,trace_draining;
static uint8_t trace_chunk;
static uint32_t trace_published,trace_coalesced,trace_completed,trace_tx_failed,trace_stats_ms;
static void trace_float(uint8_t *b,float value){uint32_t word;memcpy(&word,&value,4);ypwm_put32(b,word);}
/* Called under the existing successful-output commit critical section. */
static void publish_pid_trace(const control_context *s,const motor_feedback_set_t *f,uint32_t now,uint32_t dt){
    if(trace_pending)++trace_coalesced;
    for(unsigned pair=0;pair<6;++pair){uint8_t *b=trace_latest.frame[pair];memset(b,0,64);
        b[0]=YPWM_PID_DETAIL_EVENT;b[1]=YPWM_PID_DETAIL_SCHEMA;ypwm_put32(b+2,f->sequence);
        ypwm_put32(b+6,now);ypwm_put16(b+10,(uint16_t)s->goal.sequence);
        b[12]=(uint8_t)(pair*2U+1U);b[13]=2;ypwm_put32(b+14,dt);
        for(unsigned j=0;j<2;++j){const unsigned i=pair*2U+j;if(!(s->mask&(1U<<i)))continue;
            uint8_t *r=b+18+j*23;const position_pid_t *a=&axes[i];
            trace_float(r,a->reference);trace_float(r+4,a->trace_error);trace_float(r+8,a->trace_p);
            trace_float(r+12,a->integral);trace_float(r+16,a->trace_d);
            ypwm_put16(r+20,(uint16_t)p.sent[i]);r[22]=a->trace_flags;
        }
    }
    ++trace_published;trace_pending=true;
}
bool pwm_control_pid_detail(uint8_t b[YPWM_PID_DETAIL_LENGTH]){
    taskENTER_CRITICAL();
    if(!trace_draining&&trace_pending){trace_active=trace_latest;trace_pending=false;trace_draining=true;trace_chunk=0;}
    const bool available=trace_draining;
    if(available)memcpy(b,trace_active.frame[trace_chunk],64);
    taskEXIT_CRITICAL();return available;
}
void pwm_control_pid_detail_result(bool sent){
    taskENTER_CRITICAL();if(trace_draining){
        if(!sent){++trace_tx_failed;trace_draining=false;} /* discard incomplete set, never retry stale logs */
        else if(++trace_chunk==6){++trace_completed;trace_draining=false;}
    }taskEXIT_CRITICAL();
}
bool pwm_control_pid_stats(uint8_t b[YPWM_PID_STATS_LENGTH]){
    const uint32_t now=milliseconds();taskENTER_CRITICAL();
    if(!trace_published||now-trace_stats_ms<1000U){taskEXIT_CRITICAL();return false;}
    trace_stats_ms=now;b[0]=YPWM_PID_STATS_EVENT;b[1]=1;b[2]=b[3]=0; /* stats layout remains version1 */
    ypwm_put32(b+4,now);ypwm_put32(b+8,trace_published);ypwm_put32(b+12,trace_coalesced);
    ypwm_put32(b+16,trace_completed);ypwm_put32(b+20,trace_tx_failed);ypwm_put32(b+24,last_sample);
    taskEXIT_CRITICAL();return true;
}

static control_context snapshot(void){control_context s;taskENTER_CRITICAL();s=p;taskEXIT_CRITICAL();return s;}
/* Output guards must not copy the goal/PWM arrays onto nested task stacks. */
static bool pid_current(uint8_t phase){
    taskENTER_CRITICAL();const bool ok=p.phase==phase&&!p.direct;taskEXIT_CRITICAL();return ok;
}
static bool raw_current(uint8_t phase,uint16_t request){
    taskENTER_CRITICAL();const bool ok=p.phase==phase&&p.raw_pending&&p.raw_request==request;taskEXIT_CRITICAL();return ok;
}
static bool phase_current(uint8_t phase){
    taskENTER_CRITICAL();const bool ok=p.phase==phase;taskEXIT_CRITICAL();return ok;
}
void pwm_control_init(void){
    mode_setting_t setting={YPWM_POSITION,YPWM_ALL_MASK};const mode_store_result_t stored=mode_store_read(&setting);
    memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;
    thermal_sequence=0;memset(hot_samples,0,sizeof hot_samples);
    memset(&returning,0,sizeof returning);stop_detail_pending=false;stop_detail_ms=0;
    trace_pending=trace_draining=false;trace_chunk=0;
    trace_published=trace_coalesced=trace_completed=trace_tx_failed=trace_stats_ms=0;
    taskENTER_CRITICAL();memset(&p,0,sizeof p);p.unknown=YPWM_ALL_MASK;p.backend=setting.backend;p.mask=setting.mask;
    p.boot_restored=stored==MODE_STORE_VALID;p.phase=stored==MODE_STORE_READ_ERROR?YPWM_FAILED:YPWM_UNVERIFIED;
    p.flash_read_failed=stored==MODE_STORE_READ_ERROR;
    if(p.flash_read_failed){p.result=YPWM_FLASH_READ_FAILED;p.reason=YPWM_REASON_FLASH_READ;}
    p.dirty=true;
    mode_detail_valid=mode_detail_pending=mode_detail_has_write=rollback_detail_pending=false;taskEXIT_CRITICAL();
}
void pwm_control_on_armed(void){
    taskENTER_CRITICAL();if(p.phase==YPWM_READY||p.phase==YPWM_NATIVE_READY){p.snapshot_valid=false;p.boot_restored=false;}taskEXIT_CRITICAL();
}
bool pwm_control_native_ready(void){const control_context s=snapshot();return s.backend==YPWM_POSITION&&s.phase==YPWM_NATIVE_READY;}
bool pwm_control_accepts_positions(void){const control_context s=snapshot();return
    (s.backend==YPWM_POSITION&&s.phase==YPWM_NATIVE_READY)||
    (s.backend==YPWM_PID&&s.arm_allowed&&(s.phase==YPWM_READY||s.phase==YPWM_ACTIVE));}
static bool calibration_ready(uint16_t mask){
    uint16_t raw;int16_t steps;
    for(unsigned i=0;i<12;++i)if((mask&(1U<<i))&&!calibration_cache_get((uint8_t)(i+1U),&raw,&steps))return false;
    return true;
}
bool pwm_control_can_arm(void){const control_context s=snapshot();return
    (s.backend==YPWM_POSITION&&s.phase==YPWM_NATIVE_READY)||
    (s.backend==YPWM_PID&&s.phase==YPWM_READY&&s.arm_allowed&&calibration_ready(s.mask));}
static bool transitioning(uint8_t phase){return phase==YPWM_PREPARING||phase==YPWM_STAGED||phase==YPWM_ROLLING_BACK;}
void pwm_control_stop(uint8_t reason){
    taskENTER_CRITICAL();
    if(p.raw_pending){p.raw_pending=false;p.raw_result_pending=true;p.raw_result=YPWM_INVALID_STATE;}
    if(p.rollback_failed){
        /* OFF/release must retain the original epoch and recovery snapshot. */
        p.arm_allowed=false;p.dirty=true;
    }else if(transitioning(p.phase)){
        p.phase=YPWM_ROLLING_BACK;p.reason=reason;p.arm_allowed=false;p.result=YPWM_INVALID_STATE;p.dirty=true;
    }else if(p.phase!=YPWM_STOPPING){
        p.reconnect_stop_recoverable=p.backend==YPWM_PID&&
            (p.phase==YPWM_READY||p.phase==YPWM_ACTIVE)&&
            (p.verified&p.mask)==p.mask&&ypwm_mask(p.mask);
        p.reconnect_recovery_requested=false;
        p.phase=YPWM_STOPPING;p.reason=reason;p.arm_allowed=false;
        if(p.epoch)p.retired_epoch=p.epoch;
        p.epoch=0;p.dirty=true;
        if(p.result_pending){p.result=YPWM_INVALID_STATE;}
    }
    taskEXIT_CRITICAL();host_protocol_request_pwm_stop();
}
void pwm_control_reconnect_recover(void){
    taskENTER_CRITICAL();
    if((p.phase==YPWM_STOPPING||p.phase==YPWM_FAILED)&&
       p.reconnect_stop_recoverable&&!p.rollback_failed&&!p.flash_read_failed)
        p.reconnect_recovery_requested=true;
    taskEXIT_CRITICAL();
}
void pwm_control_connection_reset(void){const control_context s=snapshot();
    /* A restored preference is NOT a restored lease. An initial HELLO may
       establish a new session after boot OFF/mode verification, without any
       EEPROM writes. Once ARMED, the ordinary stop/reprepare rules apply. */
    if(s.boot_restored&&!s.epoch&&!s.ever_command&&
       (s.phase==YPWM_READY||s.phase==YPWM_NATIVE_READY))return;
    /* User-approved: retain rollback only until the first real ARMED state.
       This also covers a successfully committed mode whose ACK was lost. */
    if(s.snapshot_valid&&(s.phase==YPWM_READY||s.phase==YPWM_NATIVE_READY)&&!host_protocol_is_armed()){
        taskENTER_CRITICAL();if(p.epoch==s.epoch){p.phase=YPWM_ROLLING_BACK;p.arm_allowed=false;p.dirty=true;}taskEXIT_CRITICAL();
        pwm_control_stop(YPWM_REASON_LINK);return;
    }
    if(transitioning(s.phase)||(s.backend==YPWM_PID&&
        (s.phase==YPWM_READY||s.phase==YPWM_ACTIVE||(s.phase==YPWM_FAILED&&s.unknown))))
        pwm_control_stop(YPWM_REASON_LINK);
}
uint8_t pwm_control_request_mode(const uint8_t *bytes,size_t n,uint16_t sequence){
    ypwm_mode_request r;if(!ypwm_decode_mode(bytes,n,&r))return YPWM_INVALID_PAYLOAD;
    if(host_protocol_get_state()!=HOST_FSM_DISARMED)return YPWM_INVALID_STATE;
    taskENTER_CRITICAL();
    if(r.operation!=YPWM_PREPARE){
        if(r.epoch!=p.epoch||r.backend!=p.backend||r.mask!=p.mask){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
        if(p.result_pending){taskEXIT_CRITICAL();return YPWM_BUSY;}
        if(r.operation==YPWM_CANCEL){
            if(!transitioning(p.phase)&&p.phase!=YPWM_CANCELED&&
               !(p.rollback_failed&&p.phase==YPWM_FAILED)&&
               !(p.snapshot_valid&&(p.phase==YPWM_READY||p.phase==YPWM_NATIVE_READY))){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
            p.request=sequence;p.operation=YPWM_CANCEL;p.transaction_pending=true;p.result=YPWM_OK;p.arm_allowed=false;
            if(p.phase==YPWM_CANCELED){p.result_pending=true;p.transaction_pending=false;}
            else p.phase=YPWM_ROLLING_BACK;
        }else{
            if(p.phase!=YPWM_STAGED||p.mode_busy||host_protocol_is_stop_requested()){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
            p.request=sequence;p.operation=r.operation;p.phase=YPWM_PREPARING;p.transaction_pending=true;
        }
        p.dirty=true;taskEXIT_CRITICAL();return YPWM_OK;
    }
    if(p.flash_read_failed){taskEXIT_CRITICAL();return YPWM_FLASH_READ_FAILED;}
    if(p.rollback_failed){taskEXIT_CRITICAL();return YPWM_ROLLBACK_FAILED;}
    if(host_protocol_is_stop_requested()||transitioning(p.phase)||p.phase==YPWM_ACTIVE||p.phase==YPWM_STOPPING||
       p.result_pending||p.mode_busy||p.raw_pending||p.raw_result_pending){taskEXIT_CRITICAL();return YPWM_BUSY;}
    if(r.epoch==p.epoch||r.epoch==p.retired_epoch){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
    p.backend=r.backend;p.mask=r.mask;p.epoch=r.epoch;p.request=sequence;p.phase=YPWM_PREPARING;p.operation=YPWM_PREPARE;
    p.verified=0;p.reason=0;p.result=0;p.command_count=0;p.arm_allowed=false;p.ever_command=false;p.return_eligible=false;
    p.started_ms=milliseconds();p.snapshot_valid=false;p.transaction_pending=true;p.preference_touched=false;p.boot_restored=false;
    p.direct=p.raw_preview=p.pid_restart=false;
    memset(&p.goal,0,sizeof p.goal);memset(p.sent,0,sizeof p.sent);p.dirty=true;
    mode_detail_valid=mode_detail_pending=mode_detail_has_write=rollback_detail_pending=false;
    taskEXIT_CRITICAL();return YPWM_OK;
}
uint32_t pwm_control_staged_epoch(void){const control_context s=snapshot();return s.phase==YPWM_STAGED?s.epoch:0;}
bool pwm_control_preparation_pending(void){const control_context s=snapshot();return s.phase==YPWM_PREPARING||s.phase==YPWM_STAGED;}
bool pwm_control_mode_write_allowed(uint8_t id,uint8_t mode){
    if(id<1||id>12)return false;
    /* Called with the sensor's register transaction still on the stack.
     * Copy only the permission fields, never the full goal/control context.
     * Keep the same atomic state observation and all existing safety gates. */
    taskENTER_CRITICAL();
    const bool staged=p.phase==YPWM_STAGED&&p.snapshot_valid&&
        mode==((p.backend==YPWM_PID&&(p.mask&(1U<<(id-1))))?2:0);
    const uint32_t started_ms=p.started_ms;
    taskEXIT_CRITICAL();
    return staged&&
        host_protocol_get_state()==HOST_FSM_DISARMED&&!host_protocol_is_stop_requested()&&
        milliseconds()-started_ms<YPWM_PREPARATION_TIMEOUT_MS;
}
bool pwm_control_begin_mode_write(const motor_register_write_request_t *q){
    if(motor_register_validate_mode_write(q)!=MOTOR_WRITE_OK||!pwm_control_mode_write_allowed(q->motor_id,q->data[0]))return false;
    taskENTER_CRITICAL();const bool ok=p.phase==YPWM_STAGED&&!p.mode_busy;
    if(ok){p.mode_busy=true;p.mode_write_id=q->motor_id;}
    taskEXIT_CRITICAL();return ok;
}
static void diagnostic_register_report(uint8_t *d,const motor_register_write_report_t *r){
    d[30]=1;d[31]=(uint8_t)r->status;d[32]=(uint8_t)r->primary_status;d[33]=(uint8_t)r->failure_stage;
    d[34]=r->initial_value;d[35]=r->verified_value;d[36]=r->last_read_status;d[37]=r->last_write_status;
    d[38]=r->last_read_address;d[39]=r->last_write_address;d[42]=r->lock_expected;d[43]=r->lock_actual;
    d[44]=r->lock_read_status;d[45]=r->lock_write_status;
    d[46]=(r->target_write_attempted?1:0)|(r->lock_restore_attempted?2:0)|
        (r->lock_restore_verified?4:0)|(r->lock_deadline_exhausted?8:0);
    d[47]=r->lock_attempts;ypwm_put32(d+48,r->lock_elapsed_ms);ypwm_put32(d+52,r->elapsed_ms);
    d[60]=r->last_read_value;d[61]=r->io_detail;d[62]=r->mode_attempts;d[63]=r->mode_attempt_limit;
}
void pwm_control_end_mode_write(const motor_register_write_report_t *r){
    taskENTER_CRITICAL();p.mode_busy=false;
    if(r->status!=MOTOR_WRITE_OK&&transitioning(p.phase)){
        /* Preserve the failed external 0x0D, not the earlier successful PREPARE. */
        mode_detail[0]=YPWM_DIAG_EVENT;mode_detail[1]=YPWM_DIAG_VERSION;mode_detail[2]=p.backend;
        mode_detail[3]=YPWM_DIAG_MODE_TRANSACTION;mode_detail[6]=YPWM_MODE_FAILED;
        mode_detail[7]=p.mode_write_id;mode_detail[24]=0x21;
        mode_detail[25]=p.mode_write_id&&p.backend==YPWM_PID&&(p.mask&(1U<<(p.mode_write_id-1)))?2:0;
        mode_detail[26]=r->verified_value;mode_detail[27]=YPWM_DIAG_IO_REGISTER;
        mode_detail[28]=r->failure_stage==MOTOR_WRITE_STAGE_TARGET?r->last_write_status:r->last_read_status;
        mode_detail[29]=r->servo_error;diagnostic_register_report(mode_detail,r);
        ypwm_put32(mode_detail+20,milliseconds()-p.started_ms);ypwm_put32(mode_detail+56,milliseconds());
        mode_detail_valid=mode_detail_pending=true;mode_detail_has_write=false;
        const bool host_may_retry=p.phase==YPWM_STAGED&&p.snapshot_valid&&
            (r->status==MOTOR_WRITE_IO_ERROR||r->status==MOTOR_WRITE_VERIFY_MISMATCH)&&
            !r->servo_error&&!r->cleanup_servo_error&&!motor_device_register_lock_uncertain_mask()&&
            (!r->target_write_attempted||r->lock_restore_verified)&&
            host_protocol_get_state()==HOST_FSM_DISARMED&&!host_protocol_is_stop_requested()&&
            milliseconds()-p.started_ms<YPWM_PREPARATION_TIMEOUT_MS;
        /* STAGED never permits ARM. Safe transient failures wait for the PC's
           next explicit request/CANCEL; silence and the existing TTL rollback.
           Unsafe/unknown cleanup still rolls back immediately, without PC help. */
        if(!host_may_retry){p.phase=YPWM_ROLLING_BACK;p.result=YPWM_MODE_FAILED;p.reason=YPWM_REASON_MODE;}
        p.arm_allowed=false;p.dirty=true;
    }else if(r->status==MOTOR_WRITE_OK&&p.mode_write_id){
        const uint16_t ok=ypwm_u16(mode_detail+14)|(uint16_t)(1U<<(p.mode_write_id-1));ypwm_put16(mode_detail+14,ok);
    }
    taskEXIT_CRITICAL();
}
bool pwm_control_submit_positions(const motor_target_set_t *goal){
    if(!goal||!motor_device_validate_targets(goal)||!host_protocol_is_armed()||host_protocol_is_stop_requested())return false;
    taskENTER_CRITICAL();
    if(p.backend!=YPWM_PID||!p.arm_allowed||p.raw_pending||p.raw_result_pending||(p.phase!=YPWM_READY&&p.phase!=YPWM_ACTIVE)){taskEXIT_CRITICAL();return false;}
    /* New position goals do not reset the integrator or derivative history. */
    if(p.direct)p.pid_restart=true;
    p.direct=false;
    p.goal=*goal;p.accepted_ms=milliseconds();++p.command_count;p.ever_command=true;p.dirty=true;
    taskEXIT_CRITICAL();return true;
}
uint8_t pwm_control_submit_raw(const int16_t duty[12],uint16_t sequence){
    if(!duty)return YPWM_INVALID_PAYLOAD;
    const bool armed=host_protocol_is_armed();bool zero=true;
    taskENTER_CRITICAL();
    if(p.backend!=YPWM_PID){taskEXIT_CRITICAL();return YPWM_PWM_MODE_REQUIRED;}
    for(unsigned i=0;i<12;++i){
        if(duty[i]<-YPWM_DUTY_LIMIT||duty[i]>YPWM_DUTY_LIMIT||(!(p.mask&(1U<<i))&&duty[i])){
            taskEXIT_CRITICAL();return YPWM_INVALID_PAYLOAD;}
        if(duty[i])zero=false;
    }
    if(p.raw_pending||p.raw_result_pending||p.mode_busy){taskEXIT_CRITICAL();return YPWM_BUSY;}
    const bool preview=!armed&&zero&&host_protocol_get_state()==HOST_FSM_DISARMED&&
        (p.phase==YPWM_READY||p.phase==YPWM_STAGED);
    if(host_protocol_is_stop_requested()||(!preview&&(!armed||!p.arm_allowed||
        (p.phase!=YPWM_READY&&p.phase!=YPWM_ACTIVE)))){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
    memcpy(p.raw,duty,sizeof p.raw);p.raw_request=sequence;p.raw_pending=true;p.raw_preview=preview;
    if(preview)p.arm_allowed=false; /* PWM0 is not proof that torque stayed OFF. */
    else{p.direct=true;p.ever_command=true;p.accepted_ms=milliseconds();}
    p.dirty=true;taskEXIT_CRITICAL();return YPWM_OK;
}
bool pwm_control_take_raw_result(uint16_t *sequence,uint8_t *result){
    taskENTER_CRITICAL();const bool ready=p.raw_result_pending;
    if(ready){*sequence=p.raw_request;*result=p.raw_result;p.raw_result_pending=false;}
    taskEXIT_CRITICAL();return ready;
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
static bool rollback_allowed(void){return snapshot().phase==YPWM_ROLLING_BACK;}
bool pwm_control_mode_diagnostics(uint8_t detail[YPWM_DIAG_LENGTH],uint8_t write[64],bool *has_write,bool force){
    taskENTER_CRITICAL();
    if(rollback_detail_pending&&!mode_detail_pending){memcpy(detail,rollback_detail,YPWM_DIAG_LENGTH);
        *has_write=false;rollback_detail_pending=false;taskEXIT_CRITICAL();return true;}
    const bool ready=mode_detail_valid&&(force||mode_detail_pending);
    if(ready){memcpy(detail,mode_detail,sizeof mode_detail);*has_write=mode_detail_has_write;
        if(*has_write){memcpy(write,mode_write_detail,sizeof mode_write_detail);}
        mode_detail_pending=false;}
    taskEXIT_CRITICAL();return ready;
}
static void diagnostic_failure(uint8_t d[YPWM_DIAG_LENGTH],uint8_t stage,const motor_verify_report_t *r,
    uint8_t domain,uint8_t status){
    if(d[3])return; /* Preserve the first failure even if final OFF verification fails. */
    d[3]=stage;d[24]=r->address;d[25]=r->expected;d[26]=r->actual;
    d[7]=r->motor_id;d[27]=domain;d[28]=status;d[29]=r->servo_error;
}
static void prepare_mode(const control_context *s){
    uint16_t unknown=YPWM_ALL_MASK,verified=0;uint8_t error=YPWM_OK;
    const uint32_t started=milliseconds();uint8_t detail[YPWM_DIAG_LENGTH]={0},write_detail[64]={0};
    detail[0]=YPWM_DIAG_EVENT;detail[1]=YPWM_DIAG_VERSION;detail[2]=s->backend;
    ypwm_put16(detail+4,s->request);ypwm_put32(detail+8,s->epoch);ypwm_put16(detail+12,s->mask);
    memset(detail+24,255,6);detail[27]=YPWM_DIAG_IO_NONE;detail[29]=0;
    memset(detail+31,255,9);memset(detail+42,255,4);
    detail[60]=255;
    motor_verify_report_t vr=motor_verify_initial_report();vr.address=0x28;vr.expected=0;
    const motor_device_status_t off_tx=motor_device_set_torque(false);
    if(off_tx!=MOTOR_DEVICE_OK){error=YPWM_MODE_FAILED;
        diagnostic_failure(detail,YPWM_DIAG_TORQUE_OFF_TX,&vr,YPWM_DIAG_IO_DEVICE,(uint8_t)off_tx);}
    else if(!motor_device_verify_torque_off_detailed(&unknown,&vr)){error=YPWM_MODE_FAILED;
        diagnostic_failure(detail,YPWM_DIAG_TORQUE_OFF_VERIFY,&vr,YPWM_DIAG_IO_REGISTER,vr.io_status);}
    ypwm_put16(detail+18,unknown);
    /* PREPARE only snapshots. Normal register 0x21 writes belong to PC 0x0D. */
    if(!error&&s->operation==YPWM_PREPARE){
        if(!motor_device_snapshot_modes(&previous_modes,&vr)){
            error=YPWM_MODE_FAILED;diagnostic_failure(detail,YPWM_DIAG_MODE_VERIFY,&vr,YPWM_DIAG_IO_REGISTER,vr.io_status);
        }else{
            taskENTER_CRITICAL();if(p.epoch==s->epoch)p.snapshot_valid=true;taskEXIT_CRITICAL();
            previous_setting=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};
            if(mode_store_read(&previous_setting)==MODE_STORE_READ_ERROR){
                error=YPWM_FLASH_READ_FAILED;vr=motor_verify_initial_report();
                diagnostic_failure(detail,YPWM_DIAG_MODE_TRANSACTION,&vr,YPWM_DIAG_IO_NONE,255);
            }
        }
    }
    if(!error&&s->operation==YPWM_COMMIT&&!motor_device_verify_modes_detailed(s->backend,s->mask,&verified,&vr)){error=YPWM_MODE_FAILED;
        diagnostic_failure(detail,YPWM_DIAG_MODE_VERIFY,&vr,YPWM_DIAG_IO_REGISTER,vr.io_status);}
    if(!error&&s->operation==YPWM_COMMIT&&s->backend==YPWM_PID){const int16_t zero[12]={0};
        const motor_device_status_t tx=motor_device_write_pwm(s->mask,zero);
        if(tx!=MOTOR_DEVICE_OK){error=YPWM_MODE_FAILED;vr=motor_verify_initial_report();vr.address=0x2c;vr.expected=0;
            diagnostic_failure(detail,YPWM_DIAG_ZERO_TX,&vr,YPWM_DIAG_IO_DEVICE,(uint8_t)tx);}
        /* PWM0 can enable torque. Always explicitly switch OFF after attempting
         * it, including a possibly partial PWM TX, then independently read OFF. */
        const motor_device_status_t final_tx=motor_device_torque_mask(YPWM_ALL_MASK,false);
        if(final_tx!=MOTOR_DEVICE_OK&&!error){error=YPWM_MODE_FAILED;
            vr=motor_verify_initial_report();vr.address=0x28;vr.expected=0;
            diagnostic_failure(detail,YPWM_DIAG_TORQUE_OFF_TX,&vr,YPWM_DIAG_IO_DEVICE,(uint8_t)final_tx);}
    }
    if(!motor_device_verify_torque_off_detailed(&unknown,&vr)){
        if(!error){diagnostic_failure(detail,YPWM_DIAG_FINAL_OFF_VERIFY,&vr,YPWM_DIAG_IO_REGISTER,vr.io_status);ypwm_put16(detail+18,unknown);}
        if(!error)error=YPWM_MODE_FAILED;
    }
    if(!error&&s->operation==YPWM_COMMIT){
        const control_context current=snapshot();
        if(current.phase!=YPWM_PREPARING||current.epoch!=s->epoch)error=YPWM_INVALID_STATE;
        else{
            const mode_setting_t setting={s->backend,s->mask};
            taskENTER_CRITICAL();p.preference_touched=true;taskEXIT_CRITICAL();
            if(!mode_store_save(&setting)){
                error=YPWM_MODE_FAILED;vr=motor_verify_initial_report();
                diagnostic_failure(detail,YPWM_DIAG_MODE_TRANSACTION,&vr,YPWM_DIAG_IO_NONE,255);
            }
        }
    }
    if(!error){detail[3]=YPWM_DIAG_COMPLETE;ypwm_put16(detail+18,unknown);}
    ypwm_put16(detail+16,verified);ypwm_put16(detail+40,motor_device_register_lock_uncertain_mask());
    ypwm_put32(detail+20,milliseconds()-started);ypwm_put32(detail+56,milliseconds());detail[6]=error;
    memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;
    thermal_sequence=0;memset(hot_samples,0,sizeof hot_samples);
    taskENTER_CRITICAL();
    if(p.phase==YPWM_PREPARING&&p.epoch==s->epoch){
        p.result=error;p.verified=verified;p.unknown=unknown;p.torque=0;p.dirty=true;
        if(error==YPWM_FLASH_READ_FAILED){p.flash_read_failed=true;p.reason=YPWM_REASON_FLASH_READ;}
        if(error){p.phase=YPWM_ROLLING_BACK;p.arm_allowed=false;}
        else{
            p.result_pending=true;p.transaction_pending=false;
            p.phase=s->operation==YPWM_PREPARE?YPWM_STAGED:(p.backend==YPWM_PID?YPWM_READY:YPWM_NATIVE_READY);
        }
    }
    else {detail[6]=YPWM_INVALID_STATE;if(!error)detail[3]=YPWM_DIAG_CANCELED;}
    memcpy(mode_detail,detail,sizeof detail);memcpy(mode_write_detail,write_detail,sizeof write_detail);
    mode_detail_has_write=detail[30]!=0;mode_detail_valid=mode_detail_pending=true;
    taskEXIT_CRITICAL();
}
static void rollback_mode(const control_context *s){
    /* A queued/in-flight sensor job must retire before the snapshot is restored.
       Its callback rejects any late target write after STAGED is invalidated. */
    if(s->mode_busy)return;
    uint16_t unknown=YPWM_ALL_MASK,restored=0;uint8_t failed=0;
    motor_register_write_report_t failure=motor_register_initial_report();
    const bool off=motor_device_set_torque(false)==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
    bool ok=off;
    if(off&&s->snapshot_valid)ok=motor_device_restore_modes(&previous_modes,rollback_allowed,&restored,&failed,&failure);
    if(off&&!s->snapshot_valid)restored=YPWM_ALL_MASK;
    const bool final_off=motor_device_torque_mask(YPWM_ALL_MASK,false)==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
    ok=ok&&final_off&&!motor_device_register_lock_uncertain_mask();
    /* Undo the F103 preference as well as physical modes for pre-ARM CANCEL. */
    if(ok&&s->preference_touched)ok=mode_store_save(&previous_setting);
    taskENTER_CRITICAL();
    p.unknown=unknown;p.torque=0;p.verified=restored;p.arm_allowed=false;p.ever_command=false;p.rollback_failed=!ok;
    p.phase=ok?YPWM_CANCELED:YPWM_FAILED;p.dirty=true;
    if(!ok&&!p.flash_read_failed)p.reason=YPWM_REASON_ROLLBACK_FAILED;
    if(p.transaction_pending){if(!ok)p.result=YPWM_MODE_FAILED;p.result_pending=true;p.transaction_pending=false;}
    /* Retain both snapshots until every restoration proof succeeds. */
    if(ok){p.snapshot_valid=false;p.preference_touched=false;
        if(s->rollback_failed&&!p.flash_read_failed)p.reason=YPWM_REASON_NONE;}
    p.boot_restored=false;
    /* Keep original first-failure detail; status exposes rollback verification mask. */
    if(mode_detail_valid){
        memcpy(rollback_detail,mode_detail,YPWM_DIAG_LENGTH);rollback_detail[1]=YPWM_DIAG_VERSION;
        rollback_detail[3]=YPWM_DIAG_ROLLBACK;ypwm_put16(rollback_detail+4,s->request);
        rollback_detail[6]=ok?0:YPWM_MODE_FAILED;rollback_detail[7]=failed;
        ypwm_put16(rollback_detail+16,restored);ypwm_put16(rollback_detail+18,unknown);
        ypwm_put16(rollback_detail+40,motor_device_register_lock_uncertain_mask());
        rollback_detail[24]=failed?0x21:255;rollback_detail[25]=failed?previous_modes.mode[failed-1]:255;
        rollback_detail[26]=failed?failure.verified_value:255;
        rollback_detail[27]=failed?YPWM_DIAG_IO_REGISTER:YPWM_DIAG_IO_NONE;
        rollback_detail[28]=failed?(failure.failure_stage==MOTOR_WRITE_STAGE_TARGET?failure.last_write_status:failure.last_read_status):255;
        rollback_detail[29]=failure.servo_error;rollback_detail[30]=failed?1:0;
        if(failed)diagnostic_register_report(rollback_detail,&failure);
        else{memset(rollback_detail+31,255,9);memset(rollback_detail+42,255,4);rollback_detail[46]=rollback_detail[47]=0;
            rollback_detail[61]=rollback_detail[62]=rollback_detail[63]=0;}
        ypwm_put32(rollback_detail+56,milliseconds());rollback_detail_pending=true;
    }
    taskEXIT_CRITICAL();
    if(final_off&&!host_protocol_is_armed())host_protocol_complete_safe_stop();
}
/* Return uses the already configured backend/calibration, never mode writes,
 * a host lease, or new torque-ON. Boot/unarmed/unknown axes fail the eligibility
 * check and proceed to OFF; this must never arm an idle robot to center it. */
static void service_stop(const control_context *s){
    const uint32_t now=milliseconds();
    if(!returning.initialized){
        memset(&returning,0,sizeof returning);returning.initialized=true;returning.started=now;
        returning.off_only=s->backend!=YPWM_PID||!s->ever_command||!s->return_eligible||
            !ypwm_mask(s->mask)||(s->verified&s->mask)!=s->mask||!calibration_ready(s->mask);
        returning.outcome=returning.off_only?3U:1U;
        publish_stop(s,now,true);
        memset(axes,0,sizeof axes);
        thermal_sequence=0;memset(hot_samples,0,sizeof hot_samples);
        for(unsigned i=0;i<12;++i){returning.best_error[i]=UINT16_MAX;returning.last_progress[i]=now;}
    }
    if(!returning.off_only){
        motor_feedback_set_t f;imu_feedback_t imu;
        bool valid=sensor_get_feedback_snapshot(&f,&imu)&&f.sequence&&
            f.valid_mask==YPWM_ALL_MASK;
        if(now-returning.started>=15000U||!valid){returning.off_only=true;returning.outcome=valid?8U:4U;}
        else if(f.sequence!=returning.last_sequence){
            const uint32_t dt_ms=returning.last_sequence?f.uptime_ms-returning.last_stamp:20U;
            int16_t out[12]={0};bool centered=true;
            if(!dt_ms||(dt_ms&0x80000000UL)){returning.off_only=true;returning.outcome=4;}
            returning.max_error=0;
            for(unsigned i=0;i<12&&!returning.off_only;++i){
                const motor_feedback_t *m=&f.motor[i];int16_t measured=m->position;
                if(!m->valid||m->servo_error||m->position<0||m->position>4095||m->voltage<50||m->voltage>74||
                   ((s->mask&(1U<<i))&&!calibration_cache_position((uint8_t)(i+1U),m->position,&measured))){
                    returning.off_only=true;returning.outcome=(m->voltage<50||m->voltage>74)?5U:4U;break;
                }
                const uint16_t error=(uint16_t)(measured>2048?measured-2048:2048-measured);
                if(error>returning.max_error)returning.max_error=error;
                if(error>16U)centered=false;
                if(error<=16U||returning.best_error[i]==UINT16_MAX||error+12U<=returning.best_error[i]){
                    returning.best_error[i]=error;returning.last_progress[i]=now;
                }
                /* Native axes in a partial PWM bench mask must already be at
                 * center: never send PWM or a native packet to the wrong mode. */
                if(!(s->mask&(1U<<i))){if(error>16U){returning.off_only=true;returning.outcome=3;}continue;}
                position_pid_config_t config=position_pid_default_config();
                config.polarity=position_pid_motor_polarity((uint8_t)(i+1U));
                if(!position_pid_step(&axes[i],&config,2048,measured,(float)dt_ms*.001f,&out[i])){
                    returning.off_only=true;returning.outcome=9;}
            }
            returning.last_sequence=f.sequence;returning.last_stamp=f.uptime_ms;
            if(!returning.off_only&&temperature_trip(&f)){returning.off_only=true;returning.outcome=6;}
            for(unsigned i=0;i<12;++i)if(!returning.off_only&&now-returning.last_progress[i]>=500U){returning.off_only=true;returning.outcome=7;}
            returning.settled=centered?(uint8_t)(returning.settled+1U):0U;
            if(!returning.off_only&&returning.settled>=3U){returning.off_only=true;returning.outcome=2;}
            if(!returning.off_only&&phase_current(YPWM_STOPPING)){
                if(motor_device_write_pwm(s->mask,out)==MOTOR_DEVICE_OK){
                    taskENTER_CRITICAL();memcpy(p.sent,out,sizeof out);p.dirty=true;taskEXIT_CRITICAL();
                    publish_stop(s,now,false);return;
                }
                returning.off_only=true;returning.outcome=10;
            }
        }else return; /* No duplicate integration; return timeout/progress bounds remain. */
    }
    const int16_t zero[12]={0};bool zero_sent=false;
    /* PWM0 and torqueOFF are separate writes; PWM0 alone can enable torque. */
    if(s->backend==YPWM_PID&&ypwm_mask(s->mask)&&(s->verified&s->mask)==s->mask)
        zero_sent=motor_device_write_pwm(s->mask,zero)==MOTOR_DEVICE_OK;
    const motor_device_status_t tx=motor_device_torque_mask(YPWM_ALL_MASK,false);
    uint16_t unknown=YPWM_ALL_MASK;const bool off=tx==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
    /* A later H/new mode session must not inherit return integrator/reference. */
    if(off){memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;}
    taskENTER_CRITICAL();p.unknown=unknown;if(off){p.torque=0;if(zero_sent)memset(p.sent,0,sizeof p.sent);}p.dirty=true;
    if(off&&!host_protocol_is_armed()){p.phase=YPWM_FAILED;p.ever_command=false;p.return_eligible=false;}
    taskEXIT_CRITICAL();const control_context status=snapshot();publish_stop(&status,now,true);
    if(off&&!host_protocol_is_armed())host_protocol_complete_safe_stop();
}
static bool feedback_safe(const motor_feedback_set_t *f){
    if(f->valid_mask!=YPWM_ALL_MASK||!f->sequence){pwm_control_stop(YPWM_REASON_SENSOR);return false;}
    /* Current thresholds belong to the PC CLI. Preserve current telemetry and
       servo-reported fault checks, but do not latch a local current trip. */
    for(unsigned i=0;i<12;++i){const motor_feedback_t *m=&f->motor[i];
        if(!m->valid||m->servo_error||m->position<0||m->position>4095){pwm_control_stop(YPWM_REASON_SENSOR);return false;}
        if(m->voltage<50||m->voltage>74){pwm_control_stop(YPWM_REASON_THERMAL_VOLTAGE);return false;}
    }
    if(temperature_trip(f)){pwm_control_stop(YPWM_REASON_THERMAL_VOLTAGE);return false;}
    return true;
}
static void service_pid(const control_context *s,uint32_t now){
    if(!s->ever_command)return;
    if(now-s->accepted_ms>=YPWM_TARGET_TIMEOUT_MS){pwm_control_stop(YPWM_REASON_TTL);return;}
    motor_feedback_set_t f;imu_feedback_t imu;
    if(!sensor_get_feedback_snapshot(&f,&imu)){pwm_control_stop(YPWM_REASON_SENSOR);return;}
    if(!feedback_safe(&f))return;
    /* Never integrate/differentiate the same measurement more than once. */
    if(!s->pid_restart&&s->phase==YPWM_ACTIVE&&f.sequence==last_sample)return;
    if(s->pid_restart){memset(axes,0,sizeof axes);taskENTER_CRITICAL();p.pid_restart=false;taskEXIT_CRITICAL();}
    const uint32_t dt_ms=s->phase==YPWM_READY||s->pid_restart?20U:f.uptime_ms-last_feedback_ms;
    if(!dt_ms||(dt_ms&0x80000000UL)){pwm_control_stop(YPWM_REASON_SENSOR);return;}
    const float dt=(float)dt_ms*0.001f;
    int16_t out[12]={0};
    for(unsigned i=0;i<12;++i)if(s->mask&(1U<<i)){
        int16_t measured;
        if(!calibration_cache_position((uint8_t)(i+1U),f.motor[i].position,&measured)){
            pwm_control_stop(YPWM_REASON_CALIBRATION);return;
        }
        position_pid_config_t config=position_pid_default_config();
        config.polarity=position_pid_motor_polarity((uint8_t)(i+1U));
        if(!position_pid_step(&axes[i],&config,s->goal.motor[i].position,measured,dt,&out[i])){pwm_control_stop(YPWM_REASON_TRAVEL);return;}
    }
    if(!pid_current(s->phase)||host_protocol_is_stop_requested())return;
    if(motor_device_write_pwm(s->mask,out)!=MOTOR_DEVICE_OK){pwm_control_stop(YPWM_REASON_DRIVER);return;}
    taskENTER_CRITICAL();p.return_eligible=true;taskEXIT_CRITICAL();
    if(!pid_current(s->phase)||host_protocol_is_stop_requested())return;
    if(s->phase==YPWM_READY){
        taskENTER_CRITICAL();p.unknown|=s->mask;taskEXIT_CRITICAL();
        if(motor_device_torque_mask(s->mask,true)!=MOTOR_DEVICE_OK){pwm_control_stop(YPWM_REASON_DRIVER);return;}
        if(!pid_current(s->phase)||host_protocol_is_stop_requested()){
            pwm_control_stop(YPWM_REASON_LINK);return;
        }
    }
    last_sample=f.sequence;last_feedback_ms=f.uptime_ms;
    taskENTER_CRITICAL();if(p.phase==s->phase&&p.epoch==s->epoch){p.phase=YPWM_ACTIVE;p.torque=s->mask;p.unknown=s->mask;
        p.feedback_age=(uint16_t)((now-f.uptime_ms)>UINT16_MAX?UINT16_MAX:(now-f.uptime_ms));memcpy(p.sent,out,sizeof out);p.dirty=true;
        publish_pid_trace(s,&f,milliseconds(),dt_ms);}taskEXIT_CRITICAL();
}
static void service_raw(const control_context *s,uint32_t now){
    if(!s->raw_preview){
        if(!host_protocol_is_armed()||host_protocol_is_stop_requested()){pwm_control_stop(YPWM_REASON_LINK);return;}
        if(now-s->accepted_ms>=YPWM_TARGET_TIMEOUT_MS){pwm_control_stop(YPWM_REASON_TTL);return;}
        motor_feedback_set_t f;imu_feedback_t imu;
        if(!sensor_get_feedback_snapshot(&f,&imu)){pwm_control_stop(YPWM_REASON_SENSOR);return;}
        if(!feedback_safe(&f))return;
    }
    if(!s->raw_pending)return;
    uint8_t result=YPWM_OK;
    if(s->raw_preview){uint16_t verified=0;
        if(host_protocol_get_state()!=HOST_FSM_DISARMED||host_protocol_is_stop_requested())result=YPWM_INVALID_STATE;
        else if(!motor_device_verify_modes(YPWM_PID,s->mask,&verified))result=YPWM_PWM_MODE_REQUIRED;
    }
    if(!raw_current(s->phase,s->raw_request)||host_protocol_is_stop_requested())result=YPWM_INVALID_STATE;
    if(!result){
        if(motor_device_write_pwm(s->mask,s->raw)!=MOTOR_DEVICE_OK)result=YPWM_MODE_FAILED;
        else if(!s->raw_preview){taskENTER_CRITICAL();p.return_eligible=true;taskEXIT_CRITICAL();}
    }
    if(!result&&(!raw_current(s->phase,s->raw_request)||host_protocol_is_stop_requested()||
       (!s->raw_preview&&!host_protocol_is_armed())))result=YPWM_INVALID_STATE;
    if(!result&&!s->raw_preview&&s->phase==YPWM_READY){
        if(motor_device_torque_mask(s->mask,true)!=MOTOR_DEVICE_OK)result=YPWM_MODE_FAILED;
    }
    if(!s->raw_preview&&(!raw_current(s->phase,s->raw_request)||host_protocol_is_stop_requested()||!host_protocol_is_armed())){
        pwm_control_stop(YPWM_REASON_LINK);result=YPWM_INVALID_STATE;
    }
    taskENTER_CRITICAL();
    if(p.raw_pending&&p.raw_request==s->raw_request){
        p.raw_pending=false;p.raw_result=result;p.raw_result_pending=true;
        if(!result){memcpy(p.sent,s->raw,sizeof p.sent);p.unknown|=s->mask;
            if(!s->raw_preview){p.phase=YPWM_ACTIVE;p.torque=s->mask;}else p.raw_preview=false;}
        p.dirty=true;
    }
    taskEXIT_CRITICAL();
    if(result&&!s->raw_preview&&!host_protocol_is_stop_requested()&&phase_current(s->phase))
        pwm_control_stop(YPWM_REASON_DRIVER);
}
bool pwm_control_service(void){
    control_context s=snapshot();const uint32_t now=milliseconds();
    if(s.phase!=YPWM_STOPPING)memset(&returning,0,sizeof returning);
    if(s.snapshot_valid&&(s.phase==YPWM_READY||s.phase==YPWM_NATIVE_READY)&&host_protocol_is_armed()){
        taskENTER_CRITICAL();p.snapshot_valid=false;taskEXIT_CRITICAL();
    }
    if(s.phase==YPWM_NATIVE_READY)return false;
    /* A fresh HELLO after a completed operational stop may request a new
     * validation. Keep rollback/flash failures latched and never arm or
     * command motion here. A HELLO during STOPPING waits for confirmed OFF. */
    if(s.phase==YPWM_FAILED&&s.reconnect_recovery_requested&&s.reconnect_stop_recoverable&&
       !s.rollback_failed&&!s.flash_read_failed&&!s.unknown&&
       host_protocol_get_state()==HOST_FSM_DISARMED&&!host_protocol_is_stop_requested()){
        taskENTER_CRITICAL();
        if(p.phase==YPWM_FAILED&&p.reconnect_recovery_requested){
            p.phase=YPWM_UNVERIFIED;p.boot_restored=true;p.arm_allowed=false;
            p.reconnect_recovery_requested=false;p.reconnect_stop_recoverable=false;p.dirty=true;
        }
        taskEXIT_CRITICAL();return true;
    }
    if(s.phase==YPWM_UNVERIFIED){
        /* The actuator can run before host init. Complete the boot torque-OFF
         * request first, then verify the persisted backend on a later pass. */
        if(host_protocol_get_state()==HOST_FSM_BOOT)return true;
        if(host_protocol_is_stop_requested()){
            uint16_t unknown=YPWM_ALL_MASK;
            const bool off=motor_device_set_torque(false)==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
            taskENTER_CRITICAL();if(p.phase==YPWM_UNVERIFIED){p.unknown=unknown;p.dirty=true;}taskEXIT_CRITICAL();
            if(off&&!host_protocol_is_armed())host_protocol_complete_safe_stop();
            return true;
        }
        uint16_t unknown=YPWM_ALL_MASK,verified=0;
        const bool off=motor_device_set_torque(false)==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
        bool ready=off&&motor_device_verify_modes(s.backend,s.mask,&verified);
        if(ready&&s.backend==YPWM_PID){const int16_t zero[12]={0};
            const motor_device_status_t zero_tx=motor_device_write_pwm(s.mask,zero);
            /* A persisted PWM backend needs the same zero/OFF/readback order as
             * COMMIT. OFF cleanup is attempted even if the PWM TX failed. */
            const motor_device_status_t final_tx=motor_device_torque_mask(YPWM_ALL_MASK,false);
            const bool final_off=motor_device_verify_torque_off(&unknown);
            ready=zero_tx==MOTOR_DEVICE_OK&&final_tx==MOTOR_DEVICE_OK&&final_off;
        }
        taskENTER_CRITICAL();if(p.phase==YPWM_UNVERIFIED){p.unknown=unknown;p.verified=verified;
            p.phase=ready?(s.backend==YPWM_PID?YPWM_READY:YPWM_NATIVE_READY):YPWM_FAILED;
            if(ready){p.reason=YPWM_REASON_NONE;p.result=YPWM_OK;}
            p.arm_allowed=ready&&s.backend==YPWM_PID;p.dirty=true;}taskEXIT_CRITICAL();return true;
    }
    if(transitioning(s.phase)&&now-s.started_ms>=YPWM_PREPARATION_TIMEOUT_MS&&s.phase!=YPWM_ROLLING_BACK){
        pwm_control_stop(YPWM_REASON_TTL);s=snapshot();
    }
    if(s.phase==YPWM_ROLLING_BACK){rollback_mode(&s);return true;}
    if(s.raw_pending&&s.raw_preview){service_raw(&s,now);return true;}
    if(s.phase==YPWM_STAGED||s.phase==YPWM_CANCELED)return true;
    if(s.phase==YPWM_PREPARING){prepare_mode(&s);return true;}
    if((s.phase==YPWM_READY||s.phase==YPWM_ACTIVE)&&
        (host_protocol_get_state()==HOST_FSM_FAULT||host_protocol_is_stop_requested()||
         (s.phase==YPWM_ACTIVE&&!host_protocol_is_armed())))pwm_control_stop(YPWM_REASON_LINK);
    s=snapshot();
    if(s.phase==YPWM_STOPPING){service_stop(&s);return true;}
    if(s.phase==YPWM_FAILED&&s.unknown){
        (void)motor_device_torque_mask(YPWM_ALL_MASK,false);
        uint16_t unknown=YPWM_ALL_MASK;(void)motor_device_verify_torque_off(&unknown);
        taskENTER_CRITICAL();p.unknown=unknown;p.dirty=true;taskEXIT_CRITICAL();
        if(!unknown&&!host_protocol_is_armed())host_protocol_complete_safe_stop();
        return true;
    }
    if(s.phase==YPWM_FAILED&&!s.unknown&&!host_protocol_is_armed()&&host_protocol_is_stop_requested())host_protocol_complete_safe_stop();
    if(s.phase==YPWM_READY||s.phase==YPWM_ACTIVE){if(s.direct)service_raw(&s,now);else service_pid(&s,now);}
    return true;
}
