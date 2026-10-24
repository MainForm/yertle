#ifndef YERTLE_PWM_MODE_DIAGNOSTICS_H
#define YERTLE_PWM_MODE_DIAGNOSTICS_H
/* Additive EVENT payload; independent of the schema3 status and ACK/NACK.
 * All multibyte values are little endian. 255 means unobserved byte/status. */
#define YPWM_DIAG_EVENT 5U
#define YPWM_DIAG_VERSION 1U
#define YPWM_DIAG_LENGTH 64U
enum ypwm_diag_stage { YPWM_DIAG_NONE, YPWM_DIAG_TORQUE_OFF_TX,
    YPWM_DIAG_TORQUE_OFF_VERIFY, YPWM_DIAG_MODE_TRANSACTION,
    YPWM_DIAG_MODE_VERIFY, YPWM_DIAG_ZERO_TX, YPWM_DIAG_FINAL_OFF_VERIFY,
    YPWM_DIAG_COMPLETE, YPWM_DIAG_CANCELED };
enum ypwm_diag_io { YPWM_DIAG_IO_NONE, YPWM_DIAG_IO_DEVICE, YPWM_DIAG_IO_REGISTER };
/* 0 event,1 version,2 backend,3 stage,4 seq:u16,6 result,7 motor_id(0 unknown),
 * 8 epoch:u32,12 selected:u16,14 transaction_ok:u16,16 verified:u16,
 * 18 torque_unknown_at_failure:u16,20 elapsed_ms:u32,24 address,25 expected,
 * 26 actual,27 IO domain,28 IO status,29 servo_error,30 has_register_report,
 * 31 write_status,32 primary_status,33 write_stage,34 initial_value,
 * 35 verified_value,36 last_read_status,37 last_write_status,38 last_read_addr,
 * 39 last_write_addr,40 lock_unknown:u16,42 expected_lock,43 actual_lock,
 * 44 lock_read_status,45 lock_write_status,46 report_flags,47 lock_attempts,
 * 48 lock_elapsed_ms:u32,52 transaction_elapsed_ms:u32,56 mcu_ms:u32,
 * 60 last_primary_read_value,61..63 reserved0. Report flags: target1,restore_attempt2,restored4,budget8.
 * Correlated full lock history follows in existing 0x90 (request_seq/id/address).
 */
#endif
