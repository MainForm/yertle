#ifndef REGISTER_WRITE_DIAGNOSTICS_H
#define REGISTER_WRITE_DIAGNOSTICS_H
#include <string.h>
#include "motor_device.h"
#define REGISTER_WRITE_DIAGNOSTICS_TYPE 0x90U
#define REGISTER_WRITE_DIAGNOSTICS_LENGTH 64U
static inline void register_diag_u32(uint8_t *p,uint32_t v)
{ for(unsigned i=0;i<4U;++i)p[i]=(uint8_t)(v>>(8U*i)); }
/* Version1, LE; no changes to existing four-byte ACK/NACK payloads. */
static inline void register_write_diag_encode(uint8_t *p,uint16_t sequence,
    const motor_register_write_request_t *q,const motor_register_write_report_t *r,
    uint32_t completed,uint32_t failed)
{
    memset(p,0,REGISTER_WRITE_DIAGNOSTICS_LENGTH);
    p[0]=1U;p[1]=1U|(r->target_write_attempted?2U:0U)|(r->lock_restore_attempted?4U:0U)|
        (r->lock_restore_verified?8U:0U)|(r->lock_deadline_exhausted?16U:0U);
    p[2]=(uint8_t)sequence;p[3]=(uint8_t)(sequence>>8U);
    p[4]=q->motor_id;p[5]=q->address;p[6]=q->length;p[7]=(uint8_t)r->status;
    p[8]=(uint8_t)r->primary_status;p[9]=(uint8_t)r->failure_stage;
    p[10]=r->servo_error;p[11]=r->cleanup_servo_error;
    p[12]=r->lock_expected;p[13]=r->lock_actual;p[14]=r->lock_read_status;p[15]=r->lock_write_status;
    p[16]=r->lock_attempts;p[17]=r->lock_read_servo_error;p[18]=r->lock_write_servo_error;
    p[19]=r->first_lock_failure;
    register_diag_u32(p+20,r->elapsed_ms);register_diag_u32(p+24,r->lock_elapsed_ms);
    register_diag_u32(p+28,completed);register_diag_u32(p+32,failed);
    p[36]=r->first_lock_read_status;p[37]=r->first_lock_write_status;
    p[38]=r->first_lock_actual;p[39]=r->first_lock_servo_error;
    register_diag_u32(p+40,r->lock_write_attempts);
    register_diag_u32(p+44,MOTOR_LOCK_UART_TIMEOUT_MS);register_diag_u32(p+48,MOTOR_LOCK_BUDGET_MS);
    register_diag_u32(p+52,MOTOR_LOCK_SETTLE_MS);register_diag_u32(p+56,MOTOR_LOCK_ATTEMPT_LIMIT);
}
#endif
