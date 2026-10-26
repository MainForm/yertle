#ifndef YERTLE_PWM_CONTRACT_H
#define YERTLE_PWM_CONTRACT_H
/* Shared byte contract mirrored by the active policy bundle API header.
 * No packed structs, host endianness, implicit arming or motor I/O. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define YPWM_MODE_MESSAGE 0x0eU
#define YPWM_DIRECT_MESSAGE 0x0fU
#define YPWM_DIRECT_LENGTH 24U
#define YPWM_DIRECT_CAPABILITY 0x00000020UL
#define YPWM_CALIBRATED_POSITION_CAPABILITY 0x00000400UL
#define YPWM_CAPABILITY 0x00000008UL
#define YPWM_IDLE_MONITOR_CAPABILITY 0x00000010UL
#define YPWM_HOST_MODE_RETRY_CAPABILITY 0x00000040UL
#define YPWM_LONG_PREPARATION_CAPABILITY 0x00000080UL
/* EVENT telemetry v1: six 64-byte payloads, ID pairs 1/2..11/12.
 * header: event7,version1,sensor_sequence u32,output_uptime_ms u32,
 * target_sequence u16,first_id u8,count2 u8,PID_dt_ms u8,reserved0.
 * Per motor24B: reference,error,P,I,D float32 LE; sentPWM i16 LE;
 * flags u8,reserved0. Contributions precede polarity; bit5 marks negative.
 * Flags: valid1, duty-clipped2, slew-clipped4, I-clamped8, antiwindup16,
 * negative-polarity32, reset64. Invalid/unselected records are all zero.
 * EVENT8/version1/length28: uptime,published,coalesced,completed,TXfailures,
 * last_sensor_sequence (all u32 LE). Counters reset only on MCU boot/init.
 * These are cached software commands, NOT measured torque/current/PWM. */
#define YPWM_PID_DETAIL_EVENT 7U
#define YPWM_PID_STATS_EVENT 8U
#define YPWM_PID_DETAIL_SCHEMA 1U
#define YPWM_PID_DETAIL_LENGTH 64U
#define YPWM_PID_STATS_LENGTH 28U
#define YPWM_PID_DETAIL_CAPABILITY 0x00000800UL
#define YPWM_EVENT_CODE 4U
#define YPWM_SCHEMA 3U
#define YPWM_MODE_LENGTH 9U
#define YPWM_PREPARE 0U
#define YPWM_COMMIT 1U
#define YPWM_CANCEL 2U
#define YPWM_PREPARATION_TIMEOUT_MS 120000U
#define YPWM_PREPARATION_LINK_TIMEOUT_MS 20000U
#define YPWM_STATUS_LENGTH 38U
#define YPWM_MOTOR_COUNT 12U
#define YPWM_ALL_MASK 0x0fffU
#define YPWM_POSITION 0U
#define YPWM_PID 1U
/* Untuned controller output ceiling, not calibrated torque/current. */
#define YPWM_DUTY_LIMIT 800
#define YPWM_TARGET_TIMEOUT_MS 500U
#define YPWM_FEEDBACK_MAX_MS 50U
#define YPWM_FOLLOWING_ERROR_LIMIT 512U
#define YPWM_STATUS_PERIOD_MS 100U
/* FAILED (wire value 6): backend unavailable after failure/protective stop;
 * distinct from the global host FSM FAULT. Naming only; wire values unchanged. */
enum ypwm_phase { YPWM_UNVERIFIED,YPWM_NATIVE_READY,YPWM_PREPARING,YPWM_READY,
    YPWM_ACTIVE,YPWM_STOPPING,YPWM_FAILED,YPWM_STAGED,YPWM_ROLLING_BACK,YPWM_CANCELED };
enum ypwm_reason { YPWM_REASON_NONE,YPWM_REASON_RELEASE,YPWM_REASON_TTL,
    YPWM_REASON_SENSOR,YPWM_REASON_CURRENT,YPWM_REASON_TRAVEL,YPWM_REASON_SPEED,
    YPWM_REASON_DRIVER,YPWM_REASON_LINK,YPWM_REASON_SESSION,YPWM_REASON_MODE,
    YPWM_REASON_THERMAL_VOLTAGE,YPWM_REASON_DEADLINE,
    YPWM_REASON_FLASH_READ,YPWM_REASON_ROLLBACK_FAILED,YPWM_REASON_CALIBRATION };
enum ypwm_error { YPWM_OK=0,YPWM_INVALID_STATE=5,YPWM_INVALID_PAYLOAD=8,
    YPWM_BUSY=11,YPWM_MODE_FAILED=14,YPWM_STALE_EPOCH=18,YPWM_OLD_COMMAND=19,
    YPWM_FLASH_READ_FAILED=20,YPWM_ROLLBACK_FAILED=21,YPWM_PWM_MODE_REQUIRED=22 };
typedef struct { uint8_t operation,backend; uint16_t mask; uint32_t epoch; } ypwm_mode_request;
static inline uint16_t ypwm_u16(const uint8_t *p) { return (uint16_t)p[0]|(uint16_t)((uint16_t)p[1]<<8); }
static inline uint32_t ypwm_u32(const uint8_t *p) { return (uint32_t)ypwm_u16(p)|((uint32_t)ypwm_u16(p+2)<<16); }
static inline void ypwm_put16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8); }
static inline void ypwm_put32(uint8_t *p,uint32_t v) { ypwm_put16(p,(uint16_t)v);ypwm_put16(p+2,(uint16_t)(v>>16)); }
static inline bool ypwm_single(uint16_t m) { return m && !(m&(uint16_t)(m-1U)) && !(m&~YPWM_ALL_MASK); }
static inline bool ypwm_mask(uint16_t m) { return m && !(m&~YPWM_ALL_MASK); }
static inline bool ypwm_decode_mode(const uint8_t *p,size_t n,ypwm_mode_request *v) {
    if(!p||!v||n!=YPWM_MODE_LENGTH||p[0]!=YPWM_SCHEMA||p[1]>YPWM_CANCEL||p[2]>YPWM_PID)return false;
    v->operation=p[1];v->backend=p[2];v->mask=ypwm_u16(p+3);v->epoch=ypwm_u32(p+5);
    return v->epoch!=0 && (v->backend==YPWM_PID?ypwm_mask(v->mask):v->mask==YPWM_ALL_MASK);
}
/* Host int16 two's complement is NOT the servo's signed-magnitude wire value. */
static inline uint16_t ypwm_servo_word(int16_t v) {
    const int32_t w=v;return (uint16_t)(w<0?(-w)|0x0400:w);
}
#endif
