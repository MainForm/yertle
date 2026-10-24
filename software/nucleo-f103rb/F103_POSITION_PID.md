# F103 position PID backend (schema 3)

PC still sends `SET_JOINT_TARGETS 0x06`: 12 x (position u16, speed u16,
acceleration u8), total 60 bytes. Calibration, motor ID/sign mappings and
ONNX inputs/outputs are unchanged. No host PWM duty command is accepted.

## Backends and protocol

- Native backend0: existing servo position mode0 and existing native stop path.
- F103 PID backend1: selected servos use hardware PWM mode2; the F103 actuator
  task generates profiled position references, computes PID and SyncWrites PWM.
  Any nonempty 12-bit mask is supported; normal robot operation selects all12.
  Nonselected servos are torqueOFF, not simultaneous native controlled axes.
- HELLO capability mask `0x00000008`, schema3. SET_CONTROL_MODE0x0E is9 bytes:
  schema, operation(PREPARE/COMMIT/CANCEL), backend, mask u16LE, epoch u32LE.
  Normal servo mode writes now come from PC through SET_MOTOR_REGISTER0x0D.
  See [MODE_TRANSACTION.md](MODE_TRANSACTION.md) for the exact state/rollback contract.
- `0x0F` is removed. Normal release stops targets/heartbeat and uses existing
  watchdogs; PID safe-stop writes PWM0/torqueOFF, never native raw2048.
- Mode ACK means completed torqueOFF/mode/lock verification, not torqueON.
  Target ACK means position goal accepted, not motor arrival. EVENT4 retains
  the 38-byte layout with schema3; counters are now accepted position goals,
  timeout=500ms, following-error limit=512steps, output=first selected axis PWM.

Explicit mode changes occur only DISARMED/torqueOFF, preserve EEPROM locks and
require independent mode readback. Reboot does not reset servo EEPROM mode.
Boot verifies actual native modes; unknown/PWM mode never enters native arming
or the native raw2048 return path. Mode-register SET is legal only inside the
matching STAGED transition with exact ID/value/EEPROM options.

## Controller and units

The portable numerical kernel is `position_pid.c`; it has no UART or RTOS calls.
Gains are **not** the servo's P/D/I registers. Initial untuned defaults:

| Parameter | Default and unit |
| --- | --- |
| Kp | 2.0 duty-raw / encoder step |
| Ki | 0.5 duty-raw / (step second) |
| Kd | 0.05 duty-raw seconds / step |
| I term clamp | +/-20 duty-raw |
| PWM clamp | +/-100 of nominal1000 (10%, NOT torque/current percent) |
| Output slew | 2000 duty-raw/second |
| D measurement filter | time constant0.04 seconds |
| Output polarity | +1 relative to increasing encoder raw; **must validate unloaded** |

PID runs only on a fresh motor feedback sequence and uses its MCU time delta.
The actuator service is nominally10ms; feedback is nominally20ms, so PID updates
are nominally50Hz, not a claimed1kHz/current control loop. Actual jitter needs measurement.
Repeated samples do not integrate or differentiate again. A new goal does not
reset I/D history; mode preparation and stop reset the states. D is based on
measured position change (no setpoint derivative kick). Integral integration is
suppressed when its error direction would deepen duty/slew saturation.

In this software backend, **Speed explicitly means encoder steps/second**,
regardless of the servo's phase-dependent native speed units. ACC retains
8.7deg/s^2 per count, converted with4096steps/revolution; ACC0 skips the ramp.
Speed0 is rejected in PID mode, avoiding native mode's phase-dependent zero
meaning. The PC packet itself is unchanged. No angular wrapping or modulo is
applied to absolute raw positions0..4095.

The reference starts at the measured position and advances with speed/acceleration
limits. The controller does not infer a model start pose or change calibrated
absolute-angle/raw mappings. Motors' existing internal PID/startup/phase registers
are not written by this controller. Hardware PWM direction and the profile units
must be verified on the actual motor configuration before loaded operation.

## Protection and stop

Goal timeout500ms, feedback age50ms, controller-service gap>30ms, invalid motor
samples/servo faults, profile-following error>512steps, temperature>=60C or voltage
outside5.0..7.4V stop output. F103 does not enforce a per-motor or summed-current
threshold in the PWM/PID backend; current telemetry is still reported unchanged.

Current-threshold ownership is PC-only: robot_policy_cli checks each received
sensor snapshot for any absolute motor current>=2A or sum of absolute motor
currents>6A (existing6.5mA/raw conversion: first trips308raw and924raw). It stops
normal targets, requests control release and exits; in this PID backend release
uses the PWM0/torqueOFF path below. Existing PC guards are not removed or bypassed.
This relies on a functioning PC and sensor/command transport; short unseen spikes
or delayed queries can evade/delay a trip. F103 link/target/sensor timeout guards
remain, but are not a substitute for current limiting. No instantaneous hardware
current clamp is provided. Servo-reported faults still stop output independently;
the motor's own protection registers/firmware are not changed.

Stop: invalidate control and I state, PWM0 only to verified mode2 axes, torqueOFF
all motors, independent OFF readback. Never write PWM to native register aliases
or send native position2048 in PID mode. UART failure remains unknown until
readback confirms OFF. A fresh explicit mode preparation is required to rearm.
The former single-axis32step/1second bench constraints are removed; they were
not position-control limits. Conservative PWM/I clamps remain until tuning.

PID computation runs in the actuator task, not the host packet or ISR path.
All motor I/O is outside critical sections. The 2.5KB actuator stack is retained.
The historical schema2 unoptimized ARM static audit reported actuator376 + service312 + PID776
=1464 bytes before nested calls; the PWM build path adds motor144 + SyncWrite176
+ pack40 + checksum24 =384 bytes (1848 total, excluding RTOS/library/context
overhead). Schema3 transition/rollback stack sizes require a new ARM audit; those
historical figures are not a current stack proof. Actual high-water/heap/timing
measurements are still required on hardware. No claim of stable walking or tuned gains.

## Verification

`tests/pwm`: numerical P/I/D, saturation/reset/D-kick/profile/polarity and synthetic
plant tests; actual host+actuator+PID integration routes unchanged0x06 to PWM;
multi-axis/fresh-sample/timeout/cancellation/temperature/OFF protections, host-only
current ownership (signed/boundary/summed high telemetry does not stop F103, while
an explicit host release still zeros PWM and verifies torqueOFF);
schema3 rejects old direct-PWM and schema2 payloads. `tests/diagnostics` retains legacy
position stop, DMA, heartbeat and register lock tests. Synthetic tests do not
validate the real mechanism. Support the full robot and start unloaded.

Source for servo mode/PWM BIT10, phase-dependent native speed and ACC units:
https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/
