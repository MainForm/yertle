#include "pwm_control.h"
#include "position_pid.h"
#include "mode_store.h"
#include "sensor_task.h"
#include "host_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "register_write_diagnostics.h"
typedef struct {
    uint8_t backend,phase,reason,result,operation,mode_write_id;
    uint16_t mask,verified,unknown,torque,request,feedback_age;
    uint32_t epoch,retired_epoch,command_count,accepted_ms,last_service,status_ms,started_ms;
    motor_target_set_t goal;
    int16_t sent[12];
    bool result_pending,arm_allowed,dirty,ever_command,mode_busy,transaction_pending,snapshot_valid,rollback_failed;
    bool preference_touched,boot_restored,flash_read_failed;
} control_context;
static control_context p;
static motor_mode_snapshot_t previous_modes;
static mode_setting_t previous_setting;
/* Kept out of snapshot() to avoid enlarging every actuator stack frame. */
static uint8_t mode_detail[YPWM_DIAG_LENGTH],mode_write_detail[64],rollback_detail[YPWM_DIAG_LENGTH];
static bool mode_detail_valid,mode_detail_pending,mode_detail_has_write,rollback_detail_pending;
/* TEMPORARY BENCH FIRMWARE: three EVENT6/v1 records, no arming/position goals.
 * Data: event6,version1,step,backend,seq16,selected16,epoch32,mcu32,
 * preceding TX status,0,valid16,off16,on16,status[12],value[12],servo[12],elapsed32.
 * step1=beforePWM0,step2=afterPWM0(sequenceA),step3=afterexplicitOFF(sequenceB).
 * Records are queued in static storage and sent by the existing host task. */
static uint8_t sequence_audit[3][64],sequence_audit_count,sequence_audit_next;
/* These numerical states belong exclusively to the actuator task. */
static position_pid_t axes[12];
static uint32_t last_sample,last_feedback_ms;
static uint32_t milliseconds(void){return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);}
static control_context snapshot(void){control_context s;taskENTER_CRITICAL();s=p;taskEXIT_CRITICAL();return s;}
void pwm_control_init(void){
    sequence_audit_count=sequence_audit_next=0;
    mode_setting_t setting={YPWM_POSITION,YPWM_ALL_MASK};const mode_store_result_t stored=mode_store_read(&setting);
    memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;
    taskENTER_CRITICAL();memset(&p,0,sizeof p);p.unknown=YPWM_ALL_MASK;p.backend=setting.backend;p.mask=setting.mask;
    p.boot_restored=stored==MODE_STORE_VALID;p.phase=stored==MODE_STORE_READ_ERROR?YPWM_FAULT:YPWM_UNVERIFIED;
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
bool pwm_control_can_arm(void){return false;} /* BENCH ONLY: H/P cannot ARM. */
static bool transitioning(uint8_t phase){return phase==YPWM_PREPARING||phase==YPWM_STAGED||phase==YPWM_ROLLING_BACK;}
void pwm_control_stop(uint8_t reason){
    taskENTER_CRITICAL();
    if(transitioning(p.phase)){
        p.phase=YPWM_ROLLING_BACK;p.reason=reason;p.arm_allowed=false;p.result=YPWM_INVALID_STATE;p.dirty=true;
    }else if(p.phase!=YPWM_STOPPING){
        p.phase=YPWM_STOPPING;p.reason=reason;p.arm_allowed=false;
        if(p.epoch)p.retired_epoch=p.epoch;
        p.epoch=0;p.dirty=true;
        if(p.result_pending){p.result=YPWM_INVALID_STATE;}
    }
    taskEXIT_CRITICAL();host_protocol_request_pwm_stop();
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
        (s.phase==YPWM_READY||s.phase==YPWM_ACTIVE||(s.phase==YPWM_FAULT&&s.unknown))))
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
               !(p.snapshot_valid&&(p.phase==YPWM_READY||p.phase==YPWM_NATIVE_READY))){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
            p.request=sequence;p.operation=YPWM_CANCEL;p.transaction_pending=true;p.result=YPWM_OK;p.arm_allowed=false;
            if(p.phase==YPWM_CANCELED){p.result_pending=true;p.transaction_pending=false;}
            else p.phase=YPWM_ROLLING_BACK;
        }else{
            if(p.phase!=YPWM_STAGED||p.mode_busy||host_protocol_is_stop_requested()){taskEXIT_CRITICAL();return YPWM_INVALID_STATE;}
            p.request=sequence;p.operation=YPWM_COMMIT;p.phase=YPWM_PREPARING;p.transaction_pending=true;
        }
        p.dirty=true;taskEXIT_CRITICAL();return YPWM_OK;
    }
    if(p.flash_read_failed){taskEXIT_CRITICAL();return YPWM_FLASH_READ_FAILED;}
    if(p.rollback_failed){taskEXIT_CRITICAL();return YPWM_ROLLBACK_FAILED;}
    if(host_protocol_is_stop_requested()||transitioning(p.phase)||p.phase==YPWM_ACTIVE||p.phase==YPWM_STOPPING||
       p.result_pending||p.mode_busy){taskEXIT_CRITICAL();return YPWM_BUSY;}
    if(r.epoch==p.epoch||r.epoch==p.retired_epoch){taskEXIT_CRITICAL();return YPWM_STALE_EPOCH;}
    p.backend=r.backend;p.mask=r.mask;p.epoch=r.epoch;p.request=sequence;p.phase=YPWM_PREPARING;p.operation=YPWM_PREPARE;
    p.verified=0;p.reason=0;p.result=0;p.command_count=0;p.arm_allowed=false;p.ever_command=false;
    p.started_ms=milliseconds();p.snapshot_valid=false;p.transaction_pending=true;p.preference_touched=false;p.boot_restored=false;
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
    (void)goal;return false; /* BENCH ONLY: no goal or nonzero PWM path. */
}
uint8_t pwm_control_submit_raw(const int16_t duty[12],uint16_t sequence){
    (void)duty;(void)sequence;return YPWM_INVALID_STATE;
}
bool pwm_control_take_raw_result(uint16_t *sequence,uint8_t *result){
    (void)sequence;(void)result;return false;
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
    if(sequence_audit_next<sequence_audit_count){
        memcpy(detail,sequence_audit[sequence_audit_next++],64);
        *has_write=false;taskEXIT_CRITICAL();return true;
    }
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
static bool sequence_audit_capture(const control_context *s,uint8_t step,uint8_t tx_status){
    uint8_t *d=sequence_audit[step-1];memset(d,0,64);
    d[0]=6;d[1]=1;d[2]=step;d[3]=s->backend;
    ypwm_put16(d+4,s->request);ypwm_put16(d+6,s->mask);ypwm_put32(d+8,s->epoch);
    ypwm_put32(d+12,milliseconds());d[16]=tx_status;
    const uint32_t started=milliseconds();uint16_t valid=0,off=0,on=0;
    for(uint8_t id=1;id<=12;++id){
        motor_register_data_t r;
        const motor_register_status_t status=motor_device_read_register(id,0x28,1,&r);
        d[24+id-1]=(uint8_t)status;d[36+id-1]=255;d[48+id-1]=r.servo_error;
        if(status==MOTOR_REGISTER_OK&&!r.servo_error){
            const uint16_t bit=(uint16_t)(1U<<(id-1));valid|=bit;d[36+id-1]=r.data[0];
            if(r.data[0]==0)off|=bit;else if(r.data[0]==1)on|=bit;
        }
    }
    ypwm_put16(d+18,valid);ypwm_put16(d+20,off);ypwm_put16(d+22,on);
    ypwm_put32(d+60,milliseconds()-started);
    taskENTER_CRITICAL();sequence_audit_count=step;taskEXIT_CRITICAL();
    return valid==YPWM_ALL_MASK;
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
    if(!error&&s->operation==YPWM_COMMIT&&s->backend==YPWM_PID){
        taskENTER_CRITICAL();sequence_audit_count=sequence_audit_next=0;taskEXIT_CRITICAL();
        /* Establish A's common starting point using per-ID reads. */
        if(!sequence_audit_capture(s,1,(uint8_t)off_tx)||ypwm_u16(sequence_audit[0]+20)!=YPWM_ALL_MASK)
            error=YPWM_MODE_FAILED;
    }
    if(!error&&s->operation==YPWM_COMMIT&&s->backend==YPWM_PID){const int16_t zero[12]={0};
        const motor_device_status_t tx=motor_device_write_pwm(s->mask,zero);
        if(tx!=MOTOR_DEVICE_OK){error=YPWM_MODE_FAILED;vr=motor_verify_initial_report();vr.address=0x2c;vr.expected=0;
            diagnostic_failure(detail,YPWM_DIAG_ZERO_TX,&vr,YPWM_DIAG_IO_DEVICE,(uint8_t)tx);}
        const bool a_valid=sequence_audit_capture(s,2,(uint8_t)tx);
        /* B completes with an explicit OFF even if A's readback/TX failed.
         * Such a failure still aborts COMMIT; it is not hidden by cleanup. */
        const motor_device_status_t final_tx=motor_device_set_torque(false);
        const bool b_valid=sequence_audit_capture(s,3,(uint8_t)final_tx);
        if(!a_valid||!b_valid||final_tx!=MOTOR_DEVICE_OK)error=YPWM_MODE_FAILED;
    }
    if(!motor_device_verify_torque_off_detailed(&unknown,&vr)){
        if(!error){diagnostic_failure(detail,YPWM_DIAG_FINAL_OFF_VERIFY,&vr,YPWM_DIAG_IO_REGISTER,vr.io_status);ypwm_put16(detail+18,unknown);}
        if(!error)error=YPWM_MODE_FAILED;
    }
    if(!error&&s->operation==YPWM_COMMIT){
        const control_context current=snapshot();
        if(current.phase!=YPWM_PREPARING||current.epoch!=s->epoch)error=YPWM_INVALID_STATE;
        else{
            /* BENCH ONLY: do not persist this experimental selection. */
        }
    }
    if(!error){detail[3]=YPWM_DIAG_COMPLETE;ypwm_put16(detail+18,unknown);}
    ypwm_put16(detail+16,verified);ypwm_put16(detail+40,motor_device_register_lock_uncertain_mask());
    ypwm_put32(detail+20,milliseconds()-started);ypwm_put32(detail+56,milliseconds());detail[6]=error;
    memset(axes,0,sizeof axes);last_sample=last_feedback_ms=0;
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
    p.phase=ok?YPWM_CANCELED:YPWM_FAULT;p.dirty=true;
    if(!ok&&!p.flash_read_failed)p.reason=YPWM_REASON_ROLLBACK_FAILED;
    if(p.transaction_pending){if(!ok)p.result=YPWM_MODE_FAILED;p.result_pending=true;p.transaction_pending=false;}
    p.snapshot_valid=false;p.preference_touched=false;p.boot_restored=false;
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
    if(s.snapshot_valid&&(s.phase==YPWM_READY||s.phase==YPWM_NATIVE_READY)&&host_protocol_is_armed()){
        taskENTER_CRITICAL();p.snapshot_valid=false;taskEXIT_CRITICAL();
    }
    if(s.phase==YPWM_NATIVE_READY)return false;
    if(s.phase==YPWM_UNVERIFIED){
        uint16_t unknown=YPWM_ALL_MASK,verified=0;
        const bool off=motor_device_set_torque(false)==MOTOR_DEVICE_OK&&motor_device_verify_torque_off(&unknown);
        bool ready=off&&motor_device_verify_modes(s.backend,s.mask,&verified);
        if(ready&&s.backend==YPWM_PID){const int16_t zero[12]={0};
            ready=motor_device_write_pwm(s.mask,zero)==MOTOR_DEVICE_OK;
            const bool final_off=motor_device_verify_torque_off(&unknown);ready=ready&&final_off;
        }
        taskENTER_CRITICAL();if(p.phase==YPWM_UNVERIFIED){p.unknown=unknown;p.verified=verified;
            p.phase=ready?(s.backend==YPWM_PID?YPWM_READY:YPWM_NATIVE_READY):YPWM_FAULT;
            p.arm_allowed=ready&&s.backend==YPWM_PID;p.dirty=true;}taskEXIT_CRITICAL();return true;
    }
    if(transitioning(s.phase)&&now-s.started_ms>=YPWM_PREPARATION_TIMEOUT_MS&&s.phase!=YPWM_ROLLING_BACK){
        pwm_control_stop(YPWM_REASON_TTL);s=snapshot();
    }
    if(s.phase==YPWM_ROLLING_BACK){rollback_mode(&s);return true;}
    if(s.phase==YPWM_STAGED||s.phase==YPWM_CANCELED)return true;
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
