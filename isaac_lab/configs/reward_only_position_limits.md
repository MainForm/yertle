# Reward-only linkage position envelope experiment

For the current `--signed-joint-reference` path, a separate flattened USD removes
the 12 revolute joints' lower/upper limit properties before PhysX creation.
The schema fallback is -inf/+inf, creating free rather than limited motion.
A startup event verifies the runtime bounds and synchronizes the data buffers. The
original/source USD, joint frames/axes, default angles, action offset/scale and
absolute-angle-to-motor-raw mapping remain unchanged. No target clipping or
action projection is added. Old checkpoints now run with different physics
when this option is used; this is not an identical-condition legacy evaluation.

- Disable `dof_pos_limits` (the old independent soft intervals).
- Increase `motor_operating_range_penalty` from -10 to -100. Keep the measured
  linkage-dependent CSV interpolation, target/measured maximum and all formulas.
- Keep PhysX torque 1.9 Nm and velocity 4.6 rad/s, and the velocity reward
  threshold 10 rad/s / weight -0.10.
- Explicitly copy unlimited bounds into both hard/soft position data buffers,
  avoiding the SDK midpoint calculation `(-inf + inf)/2` and its NaNs.
- Setting infinite bounds on already-created limited joints was rejected by
  this PhysX version; the verified implementation removes limits at spawn.
- Existing live training processes retain their already loaded config. New
  training uses the change. No restart or commit is implied by this edit.

A larger reward weight cannot guarantee zero hardware-envelope violations.
The -100 term includes a per-offending-motor unit cost, so over-penalization
can suppress exploration or favor early termination. Monitor measured and
requested envelope violations, episode lengths and termination rates. Use an
independent hardware safety limit when deploying to real motors; reward is
not a safety interlock.

The unsigned legacy path keeps its original PhysX joint stops. Do not interpret
the removal as a change to calibrated coordinates or a claim that unrestricted
angles are mechanically possible on the real robot.
