#include "pwm_control.h"
#include "motor_control.h"
#include "motor_safe_stop.h"
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
    motor_target_set_t targets;
    uint32_t accepted_ms;
} motor_position_target_t;

enum mode_rollback_result {
    MODE_ROLLBACK_NONE = 0,
    MODE_ROLLBACK_CONFIRMED,
    MODE_ROLLBACK_FAILED
};

typedef struct {
    uint8_t backend,phase,reason,result,operation,mode_write_id,rollback_result;
    uint16_t mask,verified,unknown,torque,request,feedback_age;
    uint32_t epoch,retired_epoch,command_count,raw_accepted_ms,status_ms,started_ms;
    motor_position_target_t position_target;
    int16_t sent[12];
    int16_t raw[12];uint16_t raw_request;uint8_t raw_result;
    bool direct,raw_pending,raw_result_pending,raw_preview,pid_restart;
    bool result_pending,arm_allowed,dirty,ever_command,mode_busy,transaction_pending,snapshot_valid;
    bool preference_touched,flash_read_failed,boot_flash_read;
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
static uint8_t tx_failures;
/* Actuator-owned: one snapshot cannot count as several hot samples. */
static uint32_t thermal_sequence;
static uint8_t hot_samples[12];
/* Actuator-owned shutdown. Repeated host/fault requests must not restart it. */
static motor_safe_stop_t returning;
static int16_t return_reference[YERTLE_MOTOR_COUNT];
static bool return_reference_ready;
/* EVENT9/v1: stage1 returning,2 centered,3 ineligible,4 feedback,
 * 5 voltage,6 repeated-hot,7 stalled,8 timeout,9 PID,10 motor TX.
 * Existing EVENT envelope, no new command message or motor transaction. */
static uint8_t stop_detail[20];static bool stop_detail_pending;
static uint32_t stop_detail_ms;
static void publish_stop(uint8_t reason,uint16_t torque,uint16_t unknown,uint32_t now,bool force){
    if(!force&&now-stop_detail_ms<200U)return;
    taskENTER_CRITICAL();stop_detail_ms=now;
    stop_detail[0]=9;stop_detail[1]=1;stop_detail[2]=(uint8_t)returning.outcome;stop_detail[3]=reason;
    ypwm_put32(stop_detail+4,now);ypwm_put32(stop_detail+8,now-returning.started_ms);
    ypwm_put16(stop_detail+12,returning.max_error);stop_detail[14]=returning.settled;stop_detail[15]=0;
    ypwm_put16(stop_detail+16,torque);ypwm_put16(stop_detail+18,unknown);stop_detail_pending=true;
    taskEXIT_CRITICAL();
}
bool pwm_control_stop_detail(uint8_t b[20]){
    taskENTER_CRITICAL();const bool available=stop_detail_pending;
    if(available){memcpy(b,stop_detail,20);stop_detail_pending=false;}
    taskEXIT_CRITICAL();return available;
}
static bool temperature_trip(const motor_feedback_set_t *f){
    const bool new_sample = f->sequence != thermal_sequence;
    if (new_sample && (!thermal_sequence || f->sequence - thermal_sequence != 1U)) memset(hot_samples, 0, sizeof hot_samples);
    thermal_sequence = f->sequence;
    bool trip=false;
    for(unsigned i=0;i<12;++i){
        if(f->motor[i].temperature<60U)hot_samples[i]=0;
        else if(new_sample && hot_samples[i]<3U)++hot_samples[i];
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
        ypwm_put32(b+6,now);ypwm_put16(b+10,(uint16_t)s->position_target.targets.sequence);
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
static bool raw_current(uint8_t phase,uint16_t request){
    taskENTER_CRITICAL();const bool ok=p.phase==phase&&p.raw_pending&&p.raw_request==request;taskEXIT_CRITICAL();return ok;
}
static bool phase_current(uint8_t phase){
    taskENTER_CRITICAL();const bool ok=p.phase==phase;taskEXIT_CRITICAL();return ok;
}
void motor_control_init(void){
    memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;tx_failures=0U;
    thermal_sequence=0;memset(hot_samples,0,sizeof hot_samples);
    memset(&returning,0,sizeof returning);stop_detail_pending=false;stop_detail_ms=0;
    trace_pending=trace_draining=false;trace_chunk=0;
    trace_published=trace_coalesced=trace_completed=trace_tx_failed=trace_stats_ms=0;
    taskENTER_CRITICAL();
    memset(&p, 0, sizeof p);
    p.phase = MOTOR_CONTROL_BOOT;
    p.backend = YPWM_POSITION;
    p.mask = p.unknown = YPWM_ALL_MASK;
    p.dirty = true;
    mode_detail_valid=mode_detail_pending=mode_detail_has_write=rollback_detail_pending=false;taskEXIT_CRITICAL();
}
void motor_control_on_armed(void)
{
    taskENTER_CRITICAL();
    if (p.phase == MOTOR_CONTROL_READY) {
        p.ever_command = false;
        p.snapshot_valid = false;
    }
    taskEXIT_CRITICAL();
}

bool motor_control_position_ready(void)
{
    taskENTER_CRITICAL();
    const bool ready = p.backend == YPWM_POSITION && p.phase == MOTOR_CONTROL_READY;
    taskEXIT_CRITICAL();
    return ready;
}

/* Caller holds the critical section. */
static bool accepts_targets(void)
{
    return p.arm_allowed && (p.phase == MOTOR_CONTROL_READY || p.phase == MOTOR_CONTROL_ACTIVE);
}

bool motor_control_accepts_targets(void)
{
    taskENTER_CRITICAL();
    const bool accepted = accepts_targets();
    taskEXIT_CRITICAL();
    return accepted;
}

static bool calibration_ready(uint16_t mask){
    uint16_t raw;int16_t steps;
    for(unsigned i=0;i<12;++i)if((mask&(1U<<i))&&!calibration_cache_get((uint8_t)(i+1U),&raw,&steps))return false;
    return true;
}
bool motor_control_can_arm(void)
{
    const control_context s = snapshot();
    return s.phase == MOTOR_CONTROL_READY && s.arm_allowed && !s.unknown &&
        (s.backend == YPWM_POSITION || calibration_ready(s.mask));
}
static bool transitioning(uint8_t phase){return phase==MOTOR_CONTROL_PREPARING||phase==MOTOR_CONTROL_STAGED||phase==MOTOR_CONTROL_ROLLING_BACK;}
static void begin_stop(uint8_t reason)
{
    taskENTER_CRITICAL();
    if (p.raw_pending) {
        p.raw_pending = false;
        p.raw_result_pending = true;
        p.raw_result = YPWM_INVALID_STATE;
    }
    p.arm_allowed = false;
    if (p.rollback_result == MODE_ROLLBACK_FAILED) {
        /* Retain the transaction and its snapshot for another CANCEL. */
    } else if (transitioning(p.phase)) {
        p.phase = MOTOR_CONTROL_ROLLING_BACK;
        p.reason = reason;
        p.result = YPWM_INVALID_STATE;
    } else if (p.phase != MOTOR_CONTROL_STOPPING) {
        p.phase = MOTOR_CONTROL_STOPPING;
        p.reason = reason;
        if (p.epoch) p.retired_epoch = p.epoch;
        p.epoch = 0U;
        p.rollback_result = MODE_ROLLBACK_NONE;
        p.ever_command = false;
        p.direct = p.raw_preview = p.pid_restart = false;
        if (p.result_pending) p.result = YPWM_INVALID_STATE;
    }
    p.dirty = true;
    taskEXIT_CRITICAL();
}

void motor_control_stop(uint8_t reason)
{
    begin_stop(reason);
    host_protocol_request_motor_stop();
}

void motor_control_connection_reset(void)
{
    const control_context s = snapshot();
    if (s.snapshot_valid && s.phase == MOTOR_CONTROL_READY && !host_protocol_is_armed()) {
        taskENTER_CRITICAL();
        if (p.epoch == s.epoch && p.phase == s.phase) p.phase = MOTOR_CONTROL_ROLLING_BACK;
        taskEXIT_CRITICAL();
        motor_control_stop(YPWM_REASON_LINK);
    } else if (transitioning(s.phase) || s.phase == MOTOR_CONTROL_ACTIVE ||
               (s.phase == MOTOR_CONTROL_READY && (host_protocol_is_armed() || s.unknown))) {
        motor_control_stop(YPWM_REASON_LINK);
    }
}

uint8_t pwm_control_request_mode(const uint8_t *bytes,size_t n,uint16_t sequence){
    ypwm_mode_request r;if(!ypwm_decode_mode(bytes,n,&r))return YPWM_INVALID_PAYLOAD;
    if(host_protocol_get_state()!=HOST_FSM_DISARMED)return YPWM_INVALID_STATE;
    taskENTER_CRITICAL();
    if(p.phase==MOTOR_CONTROL_BOOT){taskEXIT_CRITICAL();return YPWM_BUSY;}
    if(r.operation!=YPWM_PREPARE){
        if(r.epoch!=p.epoch||r.backend!=p.backend||r.mask!=p.mask){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
        if(p.result_pending){taskEXIT_CRITICAL();return YPWM_BUSY;}
        if(r.operation==YPWM_CANCEL){
            const bool confirmed = p.rollback_result == MODE_ROLLBACK_CONFIRMED;
            const bool retry = p.phase == MOTOR_CONTROL_BLOCKED &&
                               p.rollback_result == MODE_ROLLBACK_FAILED;
            if(!transitioning(p.phase)&&!confirmed&&!retry&&
               !(p.snapshot_valid&&p.phase==MOTOR_CONTROL_READY)){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
            p.request=sequence;p.operation=YPWM_CANCEL;p.result=YPWM_OK;p.arm_allowed=false;
            p.transaction_pending = !confirmed;
            if (confirmed) p.result_pending = true;
            else p.phase = MOTOR_CONTROL_ROLLING_BACK;
        }else{
            if(p.phase!=MOTOR_CONTROL_STAGED||p.mode_busy||host_protocol_is_stop_requested()){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
            p.request=sequence;p.operation=r.operation;p.phase=MOTOR_CONTROL_PREPARING;p.transaction_pending=true;
        }
        p.dirty=true;taskEXIT_CRITICAL();return YPWM_OK;
    }
    if(p.flash_read_failed){taskEXIT_CRITICAL();return YPWM_FLASH_READ_FAILED;}
    if(p.rollback_result==MODE_ROLLBACK_FAILED){taskEXIT_CRITICAL();return YPWM_ROLLBACK_FAILED;}
    if(host_protocol_is_stop_requested()||transitioning(p.phase)||p.phase==MOTOR_CONTROL_ACTIVE||p.phase==MOTOR_CONTROL_STOPPING||
       p.result_pending||p.mode_busy||p.raw_pending||p.raw_result_pending){taskEXIT_CRITICAL();return YPWM_BUSY;}
    if(r.epoch==p.epoch||r.epoch==p.retired_epoch){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
    p.backend=r.backend;p.mask=r.mask;p.epoch=r.epoch;p.request=sequence;p.phase=MOTOR_CONTROL_PREPARING;p.operation=YPWM_PREPARE;
    p.verified=0;p.reason=0;p.result=0;p.command_count=0;p.arm_allowed=false;p.ever_command=false;
    p.started_ms=milliseconds();p.snapshot_valid=false;p.transaction_pending=true;p.preference_touched=false;
    p.rollback_result = MODE_ROLLBACK_NONE;
    p.direct=p.raw_preview=p.pid_restart=false;
    memset(&p.position_target,0,sizeof p.position_target);memset(p.sent,0,sizeof p.sent);p.dirty=true;
    mode_detail_valid=mode_detail_pending=mode_detail_has_write=rollback_detail_pending=false;
    taskEXIT_CRITICAL();return YPWM_OK;
}
uint32_t pwm_control_staged_epoch(void){const control_context s=snapshot();return s.phase==MOTOR_CONTROL_STAGED?s.epoch:0;}
bool pwm_control_preparation_pending(void){const control_context s=snapshot();return s.phase==MOTOR_CONTROL_PREPARING||s.phase==MOTOR_CONTROL_STAGED;}
static bool mode_write_window_open(uint32_t started_ms)
{
    return host_protocol_get_state() == HOST_FSM_DISARMED &&
           !host_protocol_is_stop_requested() &&
           milliseconds() - started_ms < YPWM_PREPARATION_TIMEOUT_MS;
}

bool pwm_control_mode_write_current(void)
{
    taskENTER_CRITICAL();
    const bool active = p.phase == MOTOR_CONTROL_STAGED && p.snapshot_valid && p.mode_busy;
    const uint32_t started_ms = p.started_ms;
    taskEXIT_CRITICAL();
    return active && mode_write_window_open(started_ms);
}

bool pwm_control_mode_write_allowed(uint8_t id,uint8_t mode){
    if(id<1||id>12)return false;
    /* Only copy the fields needed to validate the requested motor and mode. */
    taskENTER_CRITICAL();
    const bool staged=p.phase==MOTOR_CONTROL_STAGED&&p.snapshot_valid&&
        mode==((p.backend==YPWM_PID&&(p.mask&(1U<<(id-1))))?2:0);
    const uint32_t started_ms=p.started_ms;
    taskEXIT_CRITICAL();
    return staged && mode_write_window_open(started_ms);
}
bool pwm_control_begin_mode_write(const motor_register_write_request_t *q){
    if(motor_register_validate_mode_write(q)!=MOTOR_WRITE_OK||!pwm_control_mode_write_allowed(q->motor_id,q->data[0]))return false;
    taskENTER_CRITICAL();const bool ok=p.phase==MOTOR_CONTROL_STAGED&&!p.mode_busy;
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
        const bool host_may_retry=p.phase==MOTOR_CONTROL_STAGED&&p.snapshot_valid&&
            (r->status==MOTOR_WRITE_IO_ERROR||r->status==MOTOR_WRITE_VERIFY_MISMATCH)&&
            !r->servo_error&&!r->cleanup_servo_error&&!motor_device_register_lock_uncertain_mask()&&
            (!r->target_write_attempted||r->lock_restore_verified)&&
            host_protocol_get_state()==HOST_FSM_DISARMED&&!host_protocol_is_stop_requested()&&
            milliseconds()-p.started_ms<YPWM_PREPARATION_TIMEOUT_MS;
        /* STAGED never permits ARM. Safe transient failures wait for the PC's
           next explicit request/CANCEL; silence and the existing TTL rollback.
           Unsafe/unknown cleanup still rolls back immediately, without PC help. */
        if(!host_may_retry){p.phase=MOTOR_CONTROL_ROLLING_BACK;p.result=YPWM_MODE_FAILED;p.reason=YPWM_REASON_MODE;}
        p.arm_allowed=false;p.dirty=true;
    }else if(r->status==MOTOR_WRITE_OK&&p.mode_write_id){
        const uint16_t ok=ypwm_u16(mode_detail+14)|(uint16_t)(1U<<(p.mode_write_id-1));ypwm_put16(mode_detail+14,ok);
    }
    taskEXIT_CRITICAL();
}
bool motor_control_submit_targets(const motor_target_set_t *targets)
{
    if (!targets || !motor_device_validate_targets(targets)) return false;

    taskENTER_CRITICAL();
    if (!host_protocol_is_armed() || host_protocol_is_stop_requested() ||
        !accepts_targets() || p.raw_pending || p.raw_result_pending) {
        taskEXIT_CRITICAL();
        return false;
    }

    /* Only switching back from direct PWM resets the PID history. */
    if (p.direct) p.pid_restart = true;
    p.direct = false;
    p.position_target.targets = *targets;
    p.position_target.accepted_ms = milliseconds();
    p.ever_command = true;
    ++p.command_count;
    p.dirty = true;
    taskEXIT_CRITICAL();
    return true;
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
        (p.phase==MOTOR_CONTROL_READY||p.phase==MOTOR_CONTROL_STAGED);
    if(host_protocol_is_stop_requested()||(!preview&&(!armed||!p.arm_allowed||
        (p.phase!=MOTOR_CONTROL_READY&&p.phase!=MOTOR_CONTROL_ACTIVE)))){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
    memcpy(p.raw,duty,sizeof p.raw);p.raw_request=sequence;p.raw_pending=true;p.raw_preview=preview;
    if(preview)p.arm_allowed=false; /* PWM0 is not proof that torque stayed OFF. */
    else{p.direct=true;p.ever_command=true;p.raw_accepted_ms=milliseconds();}
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
        p.arm_allowed=p.result==YPWM_OK&&p.phase==MOTOR_CONTROL_READY;}
    taskEXIT_CRITICAL();return ready;
}
bool pwm_control_status(uint8_t b[YPWM_STATUS_LENGTH],bool force){
    const uint32_t now=milliseconds();taskENTER_CRITICAL();
    if(!force&&(!p.dirty||now-p.status_ms<YPWM_STATUS_PERIOD_MS)){taskEXIT_CRITICAL();return false;}
    const control_context s=p;p.dirty=false;p.status_ms=now;taskEXIT_CRITICAL();
    memset(b,0,YPWM_STATUS_LENGTH);b[0]=YPWM_EVENT_CODE;b[1]=YPWM_SCHEMA;b[2]=s.backend;
    b[3] = s.phase;
    if (s.phase == MOTOR_CONTROL_BLOCKED && s.rollback_result == MODE_ROLLBACK_CONFIRMED) {
        b[3] = YPWM_STATUS_PHASE_CANCELED;
    } else if (s.backend == YPWM_POSITION &&
               (s.phase == MOTOR_CONTROL_READY || s.phase == MOTOR_CONTROL_ACTIVE)) {
        b[3] = 1U;
    }
    ypwm_put16(b+4,s.request);b[6]=s.result;b[7]=s.reason;ypwm_put32(b+8,s.epoch);
    ypwm_put16(b+12,s.mask);ypwm_put16(b+14,s.verified);ypwm_put16(b+16,s.torque);ypwm_put16(b+18,s.unknown);
    ypwm_put32(b+20,s.command_count);ypwm_put16(b+24,YPWM_DUTY_LIMIT);ypwm_put16(b+26,MOTOR_CONTROL_TARGET_TIMEOUT_MS);
    ypwm_put16(b+28,YPWM_FOLLOWING_ERROR_LIMIT);ypwm_put16(b+30,s.feedback_age);
    for(unsigned i=0;i<12;++i)if(s.mask&(1U<<i)){ypwm_put16(b+32,(uint16_t)s.sent[i]);break;}
    ypwm_put32(b+34,now);return true;
}
static bool rollback_allowed(void){return snapshot().phase==MOTOR_CONTROL_ROLLING_BACK;}
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
        if(current.phase!=MOTOR_CONTROL_PREPARING||current.epoch!=s->epoch)error=YPWM_INVALID_STATE;
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
    memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;tx_failures=0U;
    thermal_sequence=0;memset(hot_samples,0,sizeof hot_samples);
    taskENTER_CRITICAL();
    if(p.phase==MOTOR_CONTROL_PREPARING&&p.epoch==s->epoch){
        p.result=error;p.verified=verified;p.unknown=unknown;p.torque=0;p.dirty=true;
        if(error==YPWM_FLASH_READ_FAILED){p.flash_read_failed=true;p.reason=YPWM_REASON_FLASH_READ;}
        if(error){p.phase=MOTOR_CONTROL_ROLLING_BACK;p.arm_allowed=false;}
        else{
            p.result_pending=true;p.transaction_pending=false;
            p.phase=s->operation==YPWM_PREPARE?MOTOR_CONTROL_STAGED:MOTOR_CONTROL_READY;
        }
    }
    else {detail[6]=YPWM_INVALID_STATE;if(!error)detail[3]=YPWM_DIAG_CANCELED;}
    memcpy(mode_detail,detail,sizeof detail);memcpy(mode_write_detail,write_detail,sizeof write_detail);
    mode_detail_has_write=detail[30]!=0;mode_detail_valid=mode_detail_pending=true;
    taskEXIT_CRITICAL();
}
static void rollback_mode(const control_context *s){
    /* An in-flight register write must retire before the snapshot is restored.
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
    p.unknown=unknown;p.torque=0;p.verified=restored;p.arm_allowed=false;p.ever_command=false;
    p.rollback_result = ok ? MODE_ROLLBACK_CONFIRMED : MODE_ROLLBACK_FAILED;
    p.phase = MOTOR_CONTROL_BLOCKED;
    p.dirty = true;
    if(!ok&&!p.flash_read_failed)p.reason=YPWM_REASON_ROLLBACK_FAILED;
    if(p.transaction_pending){if(!ok)p.result=YPWM_MODE_FAILED;p.result_pending=true;p.transaction_pending=false;}
    /* Retain both snapshots until every restoration proof succeeds. */
    if(ok){p.snapshot_valid=false;p.preference_touched=false;
        if(s->rollback_result==MODE_ROLLBACK_FAILED&&!p.flash_read_failed)p.reason=YPWM_REASON_NONE;}
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
/* The common stop state supplies feedback and checks progress. */
static motor_stop_outcome_t feedback_limits(const motor_feedback_set_t *feedback)
{
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        if (feedback->motor[i].voltage <= 40U || feedback->motor[i].voltage > 74U) {
            return MOTOR_STOP_VOLTAGE;
        }
    }
    return temperature_trip(feedback) ? MOTOR_STOP_TEMPERATURE : MOTOR_STOP_NONE;
}

static motor_stop_outcome_t prepare_stop_feedback(
    const void *mode, const motor_feedback_set_t *feedback, int16_t position[YERTLE_MOTOR_COUNT])
{
    const motor_stop_outcome_t limits = feedback_limits(feedback);
    if (limits != MOTOR_STOP_NONE) return limits;

    const control_context *s = mode;
    if (s->backend == YPWM_PID) {
        for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
            if ((s->mask & (1U << i)) &&
                !calibration_cache_position((uint8_t)(i + 1U), position[i], &position[i])) {
                return MOTOR_STOP_FEEDBACK;
            }
        }
    }
    return MOTOR_STOP_NONE;
}

static motor_stop_outcome_t send_position_return(
    const void *mode, const int16_t position[YERTLE_MOTOR_COUNT], uint32_t dt_ms)
{
    (void)mode;
    (void)position;
    (void)dt_ms;
    motor_target_set_t targets = { 0 };
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        targets.motor[i].position = MOTOR_STOP_CENTER;
        targets.motor[i].speed = 200U;
        targets.motor[i].acceleration = 10U;
    }
    return motor_device_write_targets(&targets) == MOTOR_DEVICE_OK ? MOTOR_STOP_NONE : MOTOR_STOP_TX;
}

static motor_stop_outcome_t send_pwm_return(
    const void *mode, const int16_t position[YERTLE_MOTOR_COUNT], uint32_t dt_ms)
{
    const control_context *s = mode;
    int16_t out[YERTLE_MOTOR_COUNT] = { 0 };
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        if (!(s->mask & (1U << i))) continue;
        int ref = return_reference_ready ? return_reference[i] : position[i];
        if (ref < MOTOR_STOP_CENTER) {
            ref = ref + 4 < MOTOR_STOP_CENTER ? ref + 4 : MOTOR_STOP_CENTER;
        } else if (ref > MOTOR_STOP_CENTER) {
            ref = ref - 4 > MOTOR_STOP_CENTER ? ref - 4 : MOTOR_STOP_CENTER;
        }
        if (ref > position[i] + 8) ref = position[i] + 8;
        if (ref < position[i] - 8) ref = position[i] - 8;
        return_reference[i] = (int16_t)ref;
        position_pid_config_t config = position_pid_default_config();
        config.polarity = position_pid_motor_polarity((uint8_t)(i + 1U));
        if (!position_pid_step(&axes[i], &config, (uint16_t)ref, position[i],
                               (float)dt_ms * 0.001f, &out[i])) return MOTOR_STOP_PID;
    }
    return_reference_ready = true;
    if (!phase_current(MOTOR_CONTROL_STOPPING) || motor_device_write_pwm(s->mask, out) != MOTOR_DEVICE_OK) {
        return MOTOR_STOP_TX;
    }
    taskENTER_CRITICAL();
    memcpy(p.sent, out, sizeof(out));
    p.dirty = true;
    taskEXIT_CRITICAL();
    return MOTOR_STOP_NONE;
}

static bool pause_pwm_return(const void *mode)
{
    const control_context *s = mode;
    const int16_t zero[YERTLE_MOTOR_COUNT] = { 0 };
    if (!phase_current(MOTOR_CONTROL_STOPPING) || motor_device_write_pwm(s->mask, zero) != MOTOR_DEVICE_OK) {
        return false;
    }
    taskENTER_CRITICAL();
    memset(p.sent, 0, sizeof(p.sent));
    p.dirty = true;
    taskEXIT_CRITICAL();
    return true;
}

static void zero_pwm_before_off(const void *mode)
{
    const control_context *s = mode;
    if (s->backend == YPWM_PID && ypwm_mask(s->mask) && (s->verified & s->mask) == s->mask) {
        (void)pause_pwm_return(s);
    }
}

static const motor_safe_stop_ops_t position_stop_ops = {
    .prepare_feedback = prepare_stop_feedback,
    .send_return = send_position_return
};

static const motor_safe_stop_ops_t pwm_stop_ops = {
    .prepare_feedback = prepare_stop_feedback,
    .send_return = send_pwm_return,
    .pause_return = pause_pwm_return,
    .before_off = zero_pwm_before_off
};

static void service_stop(const control_context *s, uint32_t now)
{
    const bool pwm = s->backend == YPWM_PID;
    const bool verified = ypwm_mask(s->mask) && (s->verified & s->mask) == s->mask;
    if (returning.outcome == MOTOR_STOP_NONE) {
        const bool eligible = verified && s->torque && (!pwm || calibration_ready(s->mask));
        motor_safe_stop_begin(&returning, now, eligible);
        return_reference_ready = false;
        publish_stop(s->reason, s->torque, s->unknown, now, true);
        memset(axes, 0, sizeof axes);
    }

    const motor_safe_stop_ops_t *ops = pwm ? &pwm_stop_ops : &position_stop_ops;
    motor_safe_stop_service(&returning, now, s->mask, ops, s);
    if (returning.outcome == MOTOR_STOP_RETURNING) {
        publish_stop(s->reason, s->torque, s->unknown, now, false);
        return;
    }

    const bool off = returning.off_confirmed;
    const bool complete = off && !host_protocol_is_armed();
    uint16_t reverified = 0U;
    const bool ready = complete && verified && s->rollback_result != MODE_ROLLBACK_FAILED && !s->flash_read_failed &&
        motor_device_verify_modes(s->backend, s->mask, &reverified);
    taskENTER_CRITICAL();
    if (complete) p.verified = reverified;
    p.unknown = returning.unknown_torque;
    if (off) p.torque = 0U;
    p.dirty = true;
    if (complete) {
        p.phase = ready ? MOTOR_CONTROL_READY : MOTOR_CONTROL_BLOCKED;
        p.arm_allowed = ready;
        if (ready) {
            p.reason = YPWM_REASON_NONE;
            p.result = YPWM_OK;
        } else if (p.rollback_result != MODE_ROLLBACK_FAILED && !p.flash_read_failed) {
            p.reason = YPWM_REASON_MODE;
            p.result = YPWM_MODE_FAILED;
        }
    }
    taskEXIT_CRITICAL();

    /* Preserve the completed stop reason in its diagnostic even after READY. */
    publish_stop(s->reason, off ? 0U : s->torque, returning.unknown_torque, now, true);
    if (complete) {
        last_sample = last_feedback_ms = 0U;
        tx_failures = 0U;
        host_protocol_complete_safe_stop();
    }
}

static bool output_is_current(const control_context *s)
{
    taskENTER_CRITICAL();
    const bool current = p.phase == s->phase && p.epoch == s->epoch &&
        p.direct == s->direct && (!s->direct || (p.raw_pending && p.raw_request == s->raw_request)) &&
        host_protocol_is_armed() && !host_protocol_is_stop_requested();
    taskEXIT_CRITICAL();
    return current;
}

static uint8_t send_drive_output(const control_context *s, const int16_t duty[YERTLE_MOTOR_COUNT])
{
    if (!output_is_current(s)) return YPWM_INVALID_STATE;
    if (s->phase == MOTOR_CONTROL_READY) {
        taskENTER_CRITICAL();
        p.unknown |= s->mask;
        taskEXIT_CRITICAL();
    }

    motor_device_status_t status = duty ? motor_device_write_pwm(s->mask, duty) :
        motor_device_write_targets(&s->position_target.targets);
    if (status == MOTOR_DEVICE_OK && s->phase == MOTOR_CONTROL_READY) {
        if (!output_is_current(s)) return YPWM_INVALID_STATE;
        status = motor_device_torque_mask(s->mask, true);
        if (status == MOTOR_DEVICE_OK) {
            taskENTER_CRITICAL();
            p.torque = s->mask;
            taskEXIT_CRITICAL();
        }
    }
    if (status != MOTOR_DEVICE_OK) return YPWM_MODE_FAILED;
    return output_is_current(s) ? YPWM_OK : YPWM_INVALID_STATE;
}

static void record_drive_result(uint8_t result)
{
    if (result == YPWM_OK) {
        tx_failures = 0U;
    } else if (result == YPWM_MODE_FAILED && host_protocol_is_armed() &&
               !host_protocol_is_stop_requested() && ++tx_failures >= MOTOR_CONTROL_TX_FAILURE_LIMIT) {
        begin_stop(YPWM_REASON_DRIVER);
        host_protocol_latch_fault(HOST_FAULT_SERVO);
    }
}

/* Called with the critical section held after a successful output. */
static bool mark_active(const control_context *s, const int16_t *duty)
{
    if (!output_is_current(s)) return false;
    p.phase = MOTOR_CONTROL_ACTIVE;
    p.torque = s->mask;
    p.unknown = s->mask;
    if (duty) memcpy(p.sent, duty, sizeof p.sent);
    p.dirty = true;
    return true;
}

static void service_pid(const control_context *s, const motor_feedback_set_t *feedback, uint32_t now)
{
    /* Sample each measurement once, including when a UART write fails. */
    if (!s->pid_restart && feedback->sequence == last_sample) return;
    if (s->pid_restart) {
        memset(axes, 0, sizeof axes);
        taskENTER_CRITICAL();
        p.pid_restart = false;
        taskEXIT_CRITICAL();
    }
    const uint32_t dt_ms = !last_sample || s->pid_restart ? 20U : feedback->uptime_ms - last_feedback_ms;
    if (!dt_ms || (dt_ms & 0x80000000UL)) {
        motor_control_stop(YPWM_REASON_SENSOR);
        return;
    }

    int16_t duty[YERTLE_MOTOR_COUNT] = { 0 };
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        if (!(s->mask & (1U << i))) continue;
        int16_t measured;
        if (!calibration_cache_position((uint8_t)(i + 1U), feedback->motor[i].position, &measured)) {
            motor_control_stop(YPWM_REASON_CALIBRATION);
            return;
        }
        position_pid_config_t config = position_pid_default_config();
        config.polarity = position_pid_motor_polarity((uint8_t)(i + 1U));
        if (!position_pid_step(&axes[i], &config, s->position_target.targets.motor[i].position,
                               measured, (float)dt_ms * 0.001f, &duty[i])) {
            motor_control_stop(YPWM_REASON_TRAVEL);
            return;
        }
    }

    last_sample = feedback->sequence;
    last_feedback_ms = feedback->uptime_ms;
    const uint8_t result = send_drive_output(s, duty);
    record_drive_result(result);
    if (result != YPWM_OK) return;

    taskENTER_CRITICAL();
    if (mark_active(s, duty)) {
        const uint32_t age = now - feedback->uptime_ms;
        p.feedback_age = (uint16_t)(age > UINT16_MAX ? UINT16_MAX : age);
        publish_pid_trace(s, feedback, milliseconds(), dt_ms);
    }
    taskEXIT_CRITICAL();
}

static void service_raw(const control_context *s)
{
    if (!s->raw_pending) return;
    uint8_t result;
    if (!s->raw_preview) {
        result = send_drive_output(s, s->raw);
    } else {
        uint16_t verified = 0U;
        if (host_protocol_get_state() != HOST_FSM_DISARMED || host_protocol_is_stop_requested()) {
            result = YPWM_INVALID_STATE;
        } else if (!motor_device_verify_modes(YPWM_PID, s->mask, &verified)) {
            result = YPWM_PWM_MODE_REQUIRED;
        } else if (!raw_current(s->phase, s->raw_request) || host_protocol_is_stop_requested()) {
            result = YPWM_INVALID_STATE;
        } else {
            taskENTER_CRITICAL();
            p.unknown |= s->mask; /* PWM 0 can enable torque, even on partial TX. */
            taskEXIT_CRITICAL();
            result = motor_device_write_pwm(s->mask, s->raw) == MOTOR_DEVICE_OK ? YPWM_OK : YPWM_MODE_FAILED;
        }
    }

    taskENTER_CRITICAL();
    if (p.raw_pending && p.raw_request == s->raw_request) {
        if (result == YPWM_OK) {
            if (!s->raw_preview) {
                if (!mark_active(s, s->raw)) result = YPWM_INVALID_STATE;
            } else {
                memcpy(p.sent, s->raw, sizeof p.sent);
                p.raw_preview = false;
            }
        }
        p.raw_pending = false;
        p.raw_result = result;
        p.raw_result_pending = true;
        p.dirty = true;
    }
    taskEXIT_CRITICAL();
    if (!s->raw_preview) record_drive_result(result);
}

static void service_drive(const control_context *s, uint32_t now)
{
    if (!s->ever_command || !host_protocol_is_armed()) return;
    const uint32_t accepted_ms = s->direct ? s->raw_accepted_ms : s->position_target.accepted_ms;
    if (now - accepted_ms >= MOTOR_CONTROL_TARGET_TIMEOUT_MS) {
        begin_stop(YPWM_REASON_TTL);
        host_protocol_request_target_timeout();
        return;
    }

    motor_feedback_set_t feedback;
    if (!sensor_get_motor_feedback_snapshot(&feedback) ||
        motor_feedback_status(&feedback) != MOTOR_FEEDBACK_VALID) {
        motor_control_stop(YPWM_REASON_SENSOR);
        return;
    }
    if (feedback_limits(&feedback) != MOTOR_STOP_NONE) {
        motor_control_stop(YPWM_REASON_THERMAL_VOLTAGE);
        return;
    }

    if (s->backend == YPWM_POSITION) {
        const uint8_t result = send_drive_output(s, NULL);
        record_drive_result(result);
        if (result != YPWM_OK) return;
        taskENTER_CRITICAL();
        (void)mark_active(s, NULL);
        taskEXIT_CRITICAL();
    } else if (s->direct) {
        service_raw(s);
    } else {
        service_pid(s, &feedback, now);
    }
}

static void service_motor_boot(const control_context *s)
{
    mode_setting_t setting = { s->backend, s->mask };
    if (!s->boot_flash_read) {
        const bool read_failed = mode_store_read(&setting) == MODE_STORE_READ_ERROR;
        taskENTER_CRITICAL();
        p.boot_flash_read = true;
        p.backend = setting.backend;
        p.mask = setting.mask;
        p.dirty = true;
        if (read_failed) {
            p.flash_read_failed = true;
            p.reason = YPWM_REASON_FLASH_READ;
            p.result = YPWM_FLASH_READ_FAILED;
            p.phase = MOTOR_CONTROL_BLOCKED;
        }
        taskEXIT_CRITICAL();
        if (read_failed) return;
    }

    /* The actuator can run before the host has published its startup OFF request. */
    if (host_protocol_get_state() == HOST_FSM_BOOT) return;

    uint16_t unknown = YPWM_ALL_MASK;
    if (host_protocol_is_stop_requested()) {
        const bool off = motor_device_set_torque(false) == MOTOR_DEVICE_OK &&
                         motor_device_verify_torque_off(&unknown);
        taskENTER_CRITICAL();
        p.unknown = unknown;
        p.dirty = true;
        taskEXIT_CRITICAL();
        if (off && !host_protocol_is_armed()) host_protocol_complete_safe_stop();
        return;
    }

    uint16_t verified = 0U;
    const bool off = motor_device_set_torque(false) == MOTOR_DEVICE_OK &&
                     motor_device_verify_torque_off(&unknown);
    bool ready = off && motor_device_verify_modes(setting.backend, setting.mask, &verified);
    if (ready && setting.backend == YPWM_PID) {
        const int16_t zero[YERTLE_MOTOR_COUNT] = { 0 };
        const motor_device_status_t zero_tx = motor_device_write_pwm(setting.mask, zero);
        /* Attempt OFF even if the preceding PWM transmission failed. */
        const motor_device_status_t final_tx = motor_device_torque_mask(YPWM_ALL_MASK, false);
        const bool final_off = motor_device_verify_torque_off(&unknown);
        ready = zero_tx == MOTOR_DEVICE_OK && final_tx == MOTOR_DEVICE_OK && final_off;
    }

    taskENTER_CRITICAL();
    p.unknown = unknown;
    p.verified = verified;
    p.phase = ready ? MOTOR_CONTROL_READY : MOTOR_CONTROL_BLOCKED;
    p.arm_allowed = ready;
    p.dirty = true;
    taskEXIT_CRITICAL();
}

void motor_control_service(void)
{
    control_context s=snapshot();const uint32_t now=milliseconds();
    if(s.phase!=MOTOR_CONTROL_STOPPING)returning.outcome=MOTOR_STOP_NONE;
    if (s.phase == MOTOR_CONTROL_BOOT) {
        service_motor_boot(&s);
        return;
    }
    if (transitioning(s.phase) && s.phase != MOTOR_CONTROL_ROLLING_BACK &&
        now - s.started_ms >= YPWM_PREPARATION_TIMEOUT_MS) {
        motor_control_stop(YPWM_REASON_TTL);
        s = snapshot();
    }
    if (s.phase == MOTOR_CONTROL_ROLLING_BACK) {
        rollback_mode(&s);
        return;
    }
    if (s.raw_pending && s.raw_preview) {
        service_raw(&s);
        return;
    }
    if (s.phase == MOTOR_CONTROL_PREPARING) {
        prepare_mode(&s);
        return;
    }
    if (s.phase == MOTOR_CONTROL_READY || s.phase == MOTOR_CONTROL_ACTIVE) {
        if (host_protocol_is_stop_requested() || host_protocol_get_state() == HOST_FSM_FAULT) {
            if (s.phase == MOTOR_CONTROL_READY && !s.unknown && !s.ever_command) {
                if (!host_protocol_is_armed()) host_protocol_complete_safe_stop();
                return;
            }
            begin_stop(YPWM_REASON_LINK);
        } else if (s.phase == MOTOR_CONTROL_ACTIVE && !host_protocol_is_armed()) {
            motor_control_stop(YPWM_REASON_LINK);
        } else {
            service_drive(&s, now);
            return;
        }
        s = snapshot();
    }
    if (s.phase == MOTOR_CONTROL_STOPPING) {
        service_stop(&s, now);
    } else if (s.phase == MOTOR_CONTROL_BLOCKED && s.unknown) {
        if (host_protocol_get_state() == HOST_FSM_BOOT) return;
        (void)motor_device_torque_mask(YPWM_ALL_MASK, false);
        uint16_t unknown = YPWM_ALL_MASK;
        (void)motor_device_verify_torque_off(&unknown);
        taskENTER_CRITICAL();
        p.unknown = unknown;
        p.dirty = true;
        taskEXIT_CRITICAL();
        if (!unknown && !host_protocol_is_armed()) host_protocol_complete_safe_stop();
    } else if (s.phase == MOTOR_CONTROL_BLOCKED &&
               !host_protocol_is_armed() && host_protocol_is_stop_requested()) {
        host_protocol_complete_safe_stop();
    }
}
