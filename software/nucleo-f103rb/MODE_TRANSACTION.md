# Motor mode register control

HELLO capability 0x2000 advertises register-based mode selection. While
DISARMED, the PC sends one SET_MOTOR_REGISTER (0x0D) per servo. Register
0x21 takes one byte, 0 for position or 2 for PWM, with the EEPROM option.
GET_MOTOR_REGISTER (0x0C) reads the same register. Writes share readback,
lock restoration and ACK/NACK with the 0x90 diagnostic. An already matching
write confirms the mode cache without programming EEPROM.

At boot the F103 confirms torque OFF and reads all twelve mode registers.
All twelve must be readable and equal before the controller permits ARM
or motor output. Mixed or unverified modes block motion but allow correcting
the mode registers while disarmed. Earlier successful changes survive
disconnect, restart and later write failures. Calibration retains its separate
F103 Flash storage.

EVENT4 keeps its layout. The former epoch word holds a configuration generation.
BLOCKED reason 16 identifies mixed modes and 17 indicates an unverified mode.
SET_CONTROL_MODE (0x0E) is no longer supported.
