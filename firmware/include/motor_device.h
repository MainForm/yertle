#ifndef MOTOR_DEVICE_H
#define MOTOR_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define YERTLE_MOTOR_COUNT 12U
#define YERTLE_ALL_MOTORS_MASK ((uint16_t)((1U << YERTLE_MOTOR_COUNT) - 1U))
#define MOTOR_REGISTER_MAX_LENGTH 32U

/* Wire values for MOTOR_REGISTER (0x8F), independent of offset-query status. */
typedef enum {
    MOTOR_REGISTER_OK = 0,
    MOTOR_REGISTER_BUSY = 1,
    MOTOR_REGISTER_COMMUNICATION_ERROR = 2,
    MOTOR_REGISTER_REPLY_ERROR = 3,
    MOTOR_REGISTER_SERVO_ERROR = 4
} motor_register_status_t;

typedef struct {
    uint8_t servo_error;
    uint8_t data[MOTOR_REGISTER_MAX_LENGTH];
} motor_register_data_t;

static inline bool motor_register_request_valid(uint8_t id, uint8_t address, uint8_t length)
{
    return id >= 1U && id <= YERTLE_MOTOR_COUNT && length >= 1U &&
           length <= MOTOR_REGISTER_MAX_LENGTH && (uint16_t)address + length <= 256U;
}

/* SET 0x0D uses existing ACK/NACK result codes, not GET 0x8F status. */
typedef enum {
    MOTOR_WRITE_OK = 0, MOTOR_WRITE_INVALID_STATE = 5, MOTOR_WRITE_BUSY = 11,
    MOTOR_WRITE_INVALID_REGISTER = 12, MOTOR_WRITE_FORBIDDEN = 13,
    MOTOR_WRITE_IO_ERROR = 14, MOTOR_WRITE_VERIFY_MISMATCH = 15,
    MOTOR_WRITE_SERVO_ERROR = 16, MOTOR_WRITE_LOCK_RESTORE_FAILED = 17
} motor_register_write_status_t;
#define MOTOR_REGISTER_ALLOW_EEPROM 1U
typedef struct {
    uint8_t motor_id, address, length, options;
    uint8_t data[MOTOR_REGISTER_MAX_LENGTH];
} motor_register_write_request_t;

/* FEETECH STS memory table and vendored docs/sts3215_memory_table.md:
 * P/D/I and minimum startup torque 0..254 (1 byte); max/current torque
 * limit 0..1000, u16 little-endian. For 0x18 follow the current FEETECH
 * table, NOT the vendor's older 2-byte definition: 0x19 is a separate field.
 * Exact individual widths only. Never infer limits from DEFAULT constants. */
static inline motor_register_write_status_t motor_register_validate_write(
    const motor_register_write_request_t *request)
{
    if (request == NULL || !motor_register_request_valid(request->motor_id, request->address, request->length) ||
        (request->options & ~MOTOR_REGISTER_ALLOW_EEPROM) != 0U) return MOTOR_WRITE_INVALID_REGISTER;
    uint8_t width;
    uint16_t maximum;
    switch (request->address) {
    case 0x10U: case 0x30U: width=2U; maximum=1000U; break;
    case 0x15U: case 0x16U: case 0x17U: case 0x18U: width=1U; maximum=254U; break;
    default: return MOTOR_WRITE_FORBIDDEN;
    }
    if (request->length != width) return MOTOR_WRITE_INVALID_REGISTER;
    const uint16_t value=(uint16_t)request->data[0] |
        (width==2U ? (uint16_t)((uint16_t)request->data[1]<<8U) : 0U);
    if (value > maximum) return MOTOR_WRITE_INVALID_REGISTER;
    if (request->address != 0x30U && (request->options & MOTOR_REGISTER_ALLOW_EEPROM)==0U)
        return MOTOR_WRITE_FORBIDDEN;
    return MOTOR_WRITE_OK;
}

typedef enum {
    MOTOR_WRITE_STAGE_NONE, MOTOR_WRITE_STAGE_TORQUE, MOTOR_WRITE_STAGE_RESPONSE_LEVEL,
    MOTOR_WRITE_STAGE_RECOVER_LOCK, MOTOR_WRITE_STAGE_READ_INITIAL, MOTOR_WRITE_STAGE_READ_LOCK,
    MOTOR_WRITE_STAGE_UNLOCK, MOTOR_WRITE_STAGE_TARGET, MOTOR_WRITE_STAGE_VERIFY,
    MOTOR_WRITE_STAGE_RESTORE
} motor_register_write_stage_t;
typedef struct {
    motor_register_write_status_t status, primary_status;
    motor_register_write_stage_t failure_stage;
    uint8_t servo_error, cleanup_servo_error;
    bool target_write_attempted, lock_restore_attempted, lock_restore_verified;
    bool lock_deadline_exhausted;
    uint8_t lock_expected, lock_actual, lock_read_status, lock_write_status;
    uint8_t lock_attempts, lock_read_servo_error, lock_write_servo_error;
    uint8_t first_lock_failure, first_lock_read_status, first_lock_write_status,
            first_lock_actual, first_lock_servo_error;
    uint32_t elapsed_ms, lock_elapsed_ms, lock_write_attempts;
} motor_register_write_report_t;

#define MOTOR_LOCK_ATTEMPT_LIMIT 2U
#define MOTOR_LOCK_UART_TIMEOUT_MS 10U
#define MOTOR_LOCK_SETTLE_MS 2U
#define MOTOR_LOCK_BUDGET_MS 100U
/* Unavailable register/status bytes must never masquerade as measured zero. */
static inline motor_register_write_report_t motor_register_initial_report(void)
{
    motor_register_write_report_t r;memset(&r,0,sizeof(r));
    r.lock_expected=r.lock_actual=r.lock_read_status=r.lock_write_status=255U;
    r.first_lock_read_status=r.first_lock_write_status=r.first_lock_actual=255U;
    return r;
}
typedef struct {
    uint32_t completed, failed;
    uint8_t motor_id, address;
    motor_register_write_report_t report;
} motor_register_write_diagnostics_t;

typedef struct {
    uint16_t position;
    uint16_t speed;
    uint8_t acceleration;
} motor_target_t;

typedef struct {
    uint32_t sequence;
    motor_target_t motor[YERTLE_MOTOR_COUNT];
} motor_target_set_t;

typedef struct {
    uint8_t id;
    bool valid;
    int16_t position;
    int16_t speed;
    int16_t load;
    int16_t current;
    uint8_t voltage;
    uint8_t temperature;
    uint8_t servo_error;
} motor_feedback_t;

typedef struct {
    uint32_t sequence;
    uint32_t uptime_ms;
    uint16_t valid_mask;
    motor_feedback_t motor[YERTLE_MOTOR_COUNT];
} motor_feedback_set_t;

typedef enum {
    MOTOR_DEVICE_OK = 0,
    MOTOR_DEVICE_INVALID_ARGUMENT,
    MOTOR_DEVICE_NOT_INITIALIZED,
    MOTOR_DEVICE_BUSY,
    MOTOR_DEVICE_PROTOCOL_ERROR,
    MOTOR_DEVICE_UART_ERROR,
    MOTOR_DEVICE_PARTIAL_FEEDBACK
} motor_device_status_t;

typedef struct {
    uint16_t raw;
    int16_t offset_steps;
    uint8_t servo_error;
} motor_position_offset_t;

bool motor_device_init(void);
bool motor_device_validate_targets(const motor_target_set_t *targets);
motor_device_status_t motor_device_write_targets(const motor_target_set_t *targets);
motor_device_status_t motor_device_set_torque(bool enabled);
motor_device_status_t motor_device_read_feedback(motor_feedback_set_t *feedback);
/* Read-only EEPROM address 31..32. Bit 11 is sign, bits 0..10 magnitude.
 * No torque, calibration, EEPROM unlock or write is performed. */
motor_device_status_t motor_device_read_position_offset(uint8_t id, motor_position_offset_t *offset);
/* One bounded READ transaction under the bus mutex. Raw bytes only, no writes,
 * torque changes, retry loop or register-dependent conversion. */
motor_register_status_t motor_device_read_register(uint8_t id, uint8_t address,
                                                  uint8_t length, motor_register_data_t *result);
/* allowed() is rechecked under the bus mutex before every non-cleanup write.
 * Cleanup is bounded and cannot be skipped by a newly arriving safety request. */
motor_register_write_report_t motor_device_write_register(const motor_register_write_request_t *request,
                                                        bool (*allowed)(void));
uint16_t motor_device_register_lock_uncertain_mask(void);
bool motor_device_get_register_write_diagnostics(motor_register_write_diagnostics_t *result);

#endif /* MOTOR_DEVICE_H */
