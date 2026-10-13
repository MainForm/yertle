# v6.8 tracking weights

Only tracking weights change from v6.7:

- `track_lin_vel_xy_cauchy`: +2.0 -> +1.0.
- `track_ang_vel_z_cauchy`: +0.75 -> +0.5.

Keep command-relative Cauchy formulas, min/relative scales and command gates.
The fresh vx-only run selects linear components `(0,)`, commands vx 0..0.3 m/s,
vy=0 and yaw=0; this does not re-enable vy tracking. The sole-point `feet_slide`
implementation/weight -0.10 and all other rewards remain unchanged. No axes,
motor API mapping, observations, actions, pose or actuator limits change.

Historical v6.7 checkpoints and training event/config files remain preserved.
The lower weights apply only when a new environment is initialized.
