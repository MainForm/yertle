#ifndef YERTLE_PWM_CONTRACT_H
#define YERTLE_PWM_CONTRACT_H
/* Shared byte contract. Copy-identical to pc-connect-api/include/yertle/pwm_contract.h.
 * No packed structs, host endianness, implicit arming or motor I/O. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define YPWM_MODE_MESSAGE 0x0eU
#define YPWM_RELEASE_MESSAGE 0x0fU
#define YPWM_CAPABILITY 0x00000004UL
#define YPWM_EVENT_CODE 4U
#define YPWM_SCHEMA 2U
#define YPWM_MODE_LENGTH 8U
#define YPWM_RELEASE_LENGTH 1U
#define YPWM_STATUS_LENGTH 38U
#define YPWM_MOTOR_COUNT 12U
#define YPWM_ALL_MASK 0x0fffU
#define YPWM_POSITION 0U
#define YPWM_PID 1U
/* Untuned controller output ceiling, not calibrated torque/current. */
#define YPWM_DUTY_LIMIT 100
#define YPWM_TARGET_TIMEOUT_MS 500U
#define YPWM_FEEDBACK_MAX_MS 50U
#define YPWM_FOLLOWING_ERROR_LIMIT 512U
#define YPWM_STATUS_PERIOD_MS 100U
enum ypwm_phase { YPWM_UNVERIFIED,YPWM_NATIVE_READY,YPWM_PREPARING,YPWM_READY,
    YPWM_ACTIVE,YPWM_STOPPING,YPWM_FAULT };
enum ypwm_reason { YPWM_REASON_NONE,YPWM_REASON_RELEASE,YPWM_REASON_TTL,
    YPWM_REASON_SENSOR,YPWM_REASON_CURRENT,YPWM_REASON_TRAVEL,YPWM_REASON_SPEED,
    YPWM_REASON_DRIVER,YPWM_REASON_LINK,YPWM_REASON_SESSION,YPWM_REASON_MODE,
    YPWM_REASON_THERMAL_VOLTAGE,YPWM_REASON_DEADLINE };
enum ypwm_error { YPWM_OK=0,YPWM_INVALID_STATE=5,YPWM_INVALID_PAYLOAD=8,
    YPWM_BUSY=11,YPWM_MODE_FAILED=14,YPWM_STALE_EPOCH=18,YPWM_OLD_COMMAND=19 };
typedef struct { uint8_t backend; uint16_t mask; uint32_t epoch; } ypwm_mode_request;
static inline uint16_t ypwm_u16(const uint8_t *p) { return (uint16_t)p[0]|(uint16_t)((uint16_t)p[1]<<8); }
static inline uint32_t ypwm_u32(const uint8_t *p) { return (uint32_t)ypwm_u16(p)|((uint32_t)ypwm_u16(p+2)<<16); }
static inline void ypwm_put16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8); }
static inline void ypwm_put32(uint8_t *p,uint32_t v) { ypwm_put16(p,(uint16_t)v);ypwm_put16(p+2,(uint16_t)(v>>16)); }
static inline bool ypwm_single(uint16_t m) { return m && !(m&(uint16_t)(m-1U)) && !(m&~YPWM_ALL_MASK); }
static inline bool ypwm_mask(uint16_t m) { return m && !(m&~YPWM_ALL_MASK); }
static inline bool ypwm_decode_mode(const uint8_t *p,size_t n,ypwm_mode_request *v) {
    if(!p||!v||n!=YPWM_MODE_LENGTH||p[0]!=YPWM_SCHEMA||p[1]>YPWM_PID)return false;
    v->backend=p[1];v->mask=ypwm_u16(p+2);v->epoch=ypwm_u32(p+4);
    return v->epoch!=0 && (v->backend==YPWM_PID?ypwm_mask(v->mask):v->mask==YPWM_ALL_MASK);
}
static inline bool ypwm_decode_release(const uint8_t *p,size_t n) { return p&&n==1U&&p[0]==YPWM_SCHEMA; }
/* Host int16 two's complement is NOT the servo's signed-magnitude wire value. */
static inline uint16_t ypwm_servo_word(int16_t v) {
    const int32_t w=v;return (uint16_t)(w<0?(-w)|0x0400:w);
}
#endif
