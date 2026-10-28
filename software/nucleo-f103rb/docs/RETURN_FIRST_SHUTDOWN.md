# Return-first shutdown (2026-10-27)

User-selected behavior: every exit/protection attempts calibrated raw2048
before OFF; if recovery cannot execute or fails, fall back to OFF. Arrival
is not guaranteed. No changes to joint/motor mapping, calibration, policy
reference or scale, current thresholds, PWM polarity or P10/I3/D1.0 gains.

## PWM / F103 PID

The actuator task alone owns recovery. It no longer immediately zeros PWM
and disables torque at the first STOPPING pass. A successfully issued armed
PWM/position command permits recovery, including a release racing the initial
torque-ON. Preparation/boot/unverified modes never start a centering motion.
No extra torque-ON, mode/register writes, host targets, or heartbeat is needed.
Direct PWM control enters the same calibrated position-return path.

Reset reference, integral and derivative history at current measured position.
Use existing physical motor polarity, P10/I3/D1.0, I output cap100 and PWM900;
center2048. Each fresh feedback sample advances the reference by up to 4 raw
toward 2048, sampled no faster than 20ms (nominal 200 raw/s), and the reference
lead is limited to 8 raw from the measurement. This limits the target, not actual
physical speed. All12 motors must be within16raw
on3 fresh snapshots before declaring centered. Duplicate samples never advance
the controller or settlement. Lack of12raw progress for500ms,15s total elapsed,
three consecutive missing, invalid or stale feedback checks (20ms apart),
voltage at or below 4.0V or above 7.4V,
three fresh >=60C samples, PID or motorTX failure ends the attempt. The first two
feedback failures pause output at PWM0; a successful sample resets the count.
Persistent
hot feedback is re-evaluated during recovery too. Follow with PWM0, explicit
torqueOFF and OFF readback. A failed OFF is retried without restarting return.

Partial PWM bench masks can center selected axes only if unselected native
axes are already centered; otherwise precheck fails and OFF is attempted.
The active CLI uses all12 PWM motors. Native-position safe-stop keeps its
existing MCU-owned center/timeout/fault recovery; it is regression tested.

## Temperature

The existing PWM guard now requires the same motor >=60C on3 consecutive
fresh complete sensor snapshots. Duplicate snapshots do not advance counts;
a reading below60C, missing sequence, or stale gap resets the streak. Voltage,
servo error, age and remaining guards retain their prior behavior. There is
still no F103 current trip: the PC owns the2A/motor and6A/sum thresholds.
These software thresholds do not guarantee a current ceiling during recovery.

## Diagnostics and PC completion

Existing EVENT0x83 gets a20byte subtype9/v1. Offsets0 code9,1 version1,2stage,
3 original stop reason,4u32uptime_ms,8u32elapsed_ms,12u16max_error_raw,
14u8settled_count,15reserved,16u16commanded_torque_mask,18u16unknown_mask.
Stages1 returning,2 centered,3 ineligible,4 feedback,5 voltage,6 hot,
7 stalled,8 timeout,9 PID,10 motorTX. Every200ms and on outcome; best effort,
no motor reads or hostUART writes in the actuator. This adds no new command.

PC release waits up to30s for a fresh DISARMED heartbeat with safety_action0.
Transient global FAULT must complete its recovery rather than fail the wait
immediately. CLI displays progress and logs return2048 rows; centered and
fallback OFF are distinguishable. OFF confirmation is not proof of arrival.
This behavior requires the newly built firmware; PC changes alone cannot
change an old MCU's immediate-OFF implementation.

## Verification

PWM/controller suites6/6 and legacy native stop/target-timeout/control-safety
tests3/3 pass on Windows. Cases include arrival from2100 to2048, duplicate
feedback, signed calibration, stall fallback, invalid feedback, arm-race
release, directPWM TTL, no boot auto-arm and retained host-only current guards.
ARM build checks new return PID/calibration/telemetry stack paths plus Flash
layout. No hardware commands or flashing performed by this verification.
