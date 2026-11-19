# Control mode at startup

The F103 confirms torque OFF and reads the mode register 0x21 from all 12
servos. All 0 enables native position control, and all 2 enables PWM control.
Mixed, unreadable or unsupported values leave the motor controller BLOCKED.
The host can correct a mode through SET_MOTOR_REGISTER while disarmed.

Servo EEPROM is the source of truth. The former F103 mode Flash region is
reserved and ignored. The separate calibration journal remains in use.
