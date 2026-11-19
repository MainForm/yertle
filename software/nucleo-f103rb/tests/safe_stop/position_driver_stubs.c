#include <assert.h>
#include "calibration_store.h"
#include "motor_device.h"
#include "pwm_contract.h"

/* Position actuator tests link the actual FSM and mock motor I/O. */
bool calibration_cache_get(uint8_t id, uint16_t *raw, int16_t *steps)
{ (void)id; (void)raw; (void)steps; assert(false); return false; }
bool calibration_cache_position(uint8_t id, int16_t raw, int16_t *position)
{ (void)id; (void)raw; (void)position; assert(false); return false; }
motor_device_status_t motor_device_torque_mask(uint16_t mask, bool enabled)
{ assert(mask == YERTLE_ALL_MOTORS_MASK); return motor_device_set_torque(enabled); }
bool motor_device_verify_modes(uint8_t backend, uint16_t mask, uint16_t *verified)
{ assert(backend == YPWM_POSITION && mask == YERTLE_ALL_MOTORS_MASK); *verified = mask; return true; }
bool motor_device_verify_torque_off_detailed(uint16_t *unknown, motor_verify_report_t *report)
{ *report = motor_verify_initial_report(); return motor_device_verify_torque_off(unknown); }
bool motor_device_verify_modes_detailed(uint8_t backend, uint16_t mask, uint16_t *verified, motor_verify_report_t *report)
{ *report = motor_verify_initial_report(); return motor_device_verify_modes(backend, mask, verified); }
motor_device_status_t motor_device_write_pwm(uint16_t mask, const int16_t duty[YERTLE_MOTOR_COUNT])
{ (void)mask; (void)duty; assert(false); return MOTOR_DEVICE_INVALID_ARGUMENT; }

void motor_device_read_modes(uint16_t *position, uint16_t *pwm)
{ *position=YPWM_ALL_MASK; *pwm=0; }
