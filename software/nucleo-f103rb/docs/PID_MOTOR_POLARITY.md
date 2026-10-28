# PID motor output polarity for the bench robot

The position controller compares signed encoder-raw coordinates. Its final
PWM output is multiplied by a physical actuator polarity. This is independent
of the calibrated simulation joint-angle/raw mapping and shoulder/thigh/shin
linkage. Do not multiply the API direction_sign into the raw PID error.

| Motor IDs | PID output polarity |
|---|---:|
| 1 through 12 | -1 |

Evidence:
- robot_20261026_192238_20012.jsonl: IDs1,2,6,10,11,12 showed
  sustained opposite raw motion during H/seated.
- robot_20261026_203423_16352.jsonl: after the first correction, IDs4,5,7,8
  showed sustained opposite motion during U/standing; corrected IDs1,2,10,11
  moved toward their requested raw direction. Extend reversal to these four.
  The run stopped with F103 thermal/voltage protection and a PC >6A aggregate
  absolute-feedback-current trip. Retain both protections without changes.

- robot_20261026_213947_37532.jsonl: during H/seated, ID3 target decreased
  from2112 toward2048 while measured raw increased to2533 and PWM saturated
  at-800. F103 reported following protection (reason5), then the PC received
  invalid_state NACK. Reverse ID3 from+1 to-1. The user explicitly requested
  ID9 reversal as well; its observed movement was too small to establish its
  physical direction. Both are now-1; ID9 still needs physical validation.

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
Current F103 defaults: P=5, I=0, D=0.10, integral-output clamp +/-100
(inactive with Ki=0), PWM ceiling +/-900. See PID_GAIN_TRIAL_5_3.md for the P-only reduction trial. It follows the polarity corrections;
it does not change any direction or calibration. Firmware guards and PC current
limits remain in place. No new message type is added.

Tests cover all12 IDs with positive/negative errors, zero output, actual service
dispatch, telemetry polarity flag and existing raw-PWM/native/stop paths.
After flashing yertle_f103rb.bin and resetting, F3/F5/H remain manual. Verify
the H motion before U/P; physical validation is not performed by the build.
