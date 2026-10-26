# PID motor output polarity for the bench robot

The position controller compares signed encoder-raw coordinates. Its final
PWM output is multiplied by a physical actuator polarity. This is independent
of the calibrated simulation joint-angle/raw mapping and shoulder/thigh/shin
linkage. Do not multiply the API direction_sign into the raw PID error.

| Motor IDs | PID output polarity |
|---|---:|
| 1, 2, 4, 5, 6, 7, 8, 10, 11, 12 | -1 |
| 3, 9 | +1 (retained; physical direction not established) |

Evidence:
- robot_20261026_192238_20012.jsonl: IDs1,2,6,10,11,12 showed
  sustained opposite raw motion during H/seated.
- robot_20261026_203423_16352.jsonl: after the first correction, IDs4,5,7,8
  showed sustained opposite motion during U/standing; corrected IDs1,2,10,11
  moved toward their requested raw direction. Extend reversal to these four.
  The run stopped with F103 thermal/voltage protection and a PC >6A aggregate
  absolute-feedback-current trip. Retain both protections without changes.

This is a provisional bench correction; it is not a completed 12-axis unloaded
direction calibration or a validated walking gain selection.

service_pid selects polarity by physical motor ID immediately before calling
position_pid_step. P/I/D, signed error and calibrated current position remain
unchanged. The complete final P+I+D output is reversed, including its derivative
and integral contributions. Telemetry flags bit5 reports negative polarity;
P/I/D contributions remain before polarity and pwm_raw is the actual command.
Generic default config remains +1; invalid IDs return 0 and fail PID validation.

Native motor position mode and explicit SET_MOTOR_PWM retain their existing
semantics. Only the F103 position-to-PWM PID route uses this table. Neither
EEPROM phase registers nor raw calibration nor PC/API/model files are changed.
P=2, I=0.5, D=0.05, integral clamp20 and PWM ceiling800 are unchanged. Firmware
guards and PC current limits remain in place. No new message type is added.

Tests cover all12 IDs with positive/negative errors, zero output, actual service
dispatch, telemetry polarity flag and existing raw-PWM/native/stop paths.
After flashing yertle_f103rb.bin and resetting, F3/F5/H remain manual. Verify
the H motion before U/P; physical validation is not performed by the build.
