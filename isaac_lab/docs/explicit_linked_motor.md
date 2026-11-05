# Explicit linked-motor training and playback

`--explicit-linked-motor` selects a separate flat-task actuator profile in
`train.py` and `play.py`. Without this flag the existing implicit profile is
unchanged. The flag requires `--signed-joint-reference` and cannot be combined
with the joint-level `--effort-limit-sim` or `--velocity-limit-sim` options.

- Physics: 1 ms, 20 physics steps per 20 ms policy action, position solver 64,
  velocity solver 0; the contact sensor follows the 1 ms physics period.
- Controller: transform joint targets and positions through the existing
  `MotorRawTransform`, round to encoder steps and apply the F103 raw-position
  PID separately to each motor every 20 ms. Its firmware parameters are
  P=7 PWM/step, I=3 PWM/(step s), D=0.13 PWM/(step/s), the accumulated I
  contribution is bounded to +/-100 PWM, and PWM output is limited to +/-1000.
  Derivative uses the measured-position finite difference, without a filter.
  The simulation assumes a fixed 20 ms feedback interval (the real firmware
  uses actual fresh-feedback dt) and rounds the output as the firmware does.
- Firmware-style conditional integration blocks windup at its +/-1000 PWM
  output limit. The subsequent 1.024 Nm torque clamp and 4.6 rad/s directional
  cutoff are simulation plant constraints and do not feed back into that
  firmware anti-windup calculation. An
  individually reset environment clears its own I state and is refreshed on
  the next physics write without advancing other environments' clocks. I,
  previous measured position, initialization and invalid-position guard are
  included in environment checkpoints.
- Ideal thigh/shin linkage: the physical motor PWM output is converted to
  motor torque using the **uncalibrated approximation** 800 PWM = 1.024 Nm,
  then each motor torque is clamped to 1.024 Nm and transformed back to the
  joint torques passed to PhysX. The model does not reproduce voltage, current,
  back-EMF, load-dependent torque, motor travel or communication dynamics.
  Joint-level simulation effort/velocity
  bounds are made nonbinding for this profile; use the signed-reference
  release-position-limits event for the position stops.
- Motor speed 4.6 rad/s is a **directional drive cutoff sampled at controller
  updates**, not a hard speed cap. Momentum, contact, gravity and the 20 ms
  hold can carry the actual speed beyond it. There is no electrical or
  voltage-sag model in this actuator.
- `dof_torques_l2`, `dof_acc_l2`, `joint_vel_l2`, `joint_velocity_excess`
  and `energy` retain their existing names and weights, but use motor-space
  quantities in this profile. The `joint_velocity_excess` reward retains its
  separate 2.3 rad/s threshold and -0.1 weight; this is not the 4.6 rad/s
  drive cutoff. Existing total/per-leg current penalties already transform
  applied joint torques into motor torque estimates; they remain uncalibrated
  current proxies, not actual current limits. The operating-range penalty
  already converts target and measured joint positions to raw motor space.
- A goal or measurement outside the valid 0..4095 encoder range latches a
  zero-output guard for that environment until reset, matching the firmware's
  invalid-position stop in principle, not its full communication safety path.
- Joint axes, action/observation layout and calibration are not modified.
  The ideal linkage is an assumption and must be compared with the hardware.

For a new training run, add `--explicit-linked-motor --signed-joint-reference`
to the desired flat-task training options; use the same two flags in `play.py`
for that checkpoint. Existing implicit checkpoints can be evaluated in this
new physics as an experiment, but their behavior may differ. If starting from
an implicit checkpoint, intentionally reset incompatible environment state
with `--resume-env-state off`; fresh runs need no checkpoint. Checkpoints from
the same explicit configuration can use the normal environment-state resume.
PhysX's internal solver state is not exactly restored by that mechanism.
