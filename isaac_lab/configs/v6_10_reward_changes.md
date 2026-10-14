# v6.10 reward configuration

- Restore linear Cauchy tracking weight: 1.0 -> 2.0.
- Restore yaw Cauchy tracking weight: 0.5 -> 0.75.
- Strengthen scheduled swing clearance penalty: -0.06 -> -0.20.
- Retain Cauchy formulas/scales and vx-only component selection.
- Retain rubber surface height target 0.045 m and transition 0.015 m.
- All other reward terms, joint frames, API mapping, action reference/scale,
  observations, contact geometry and fixed trot remain unchanged.

Fresh training: 1024 environments, seed 0, 7000 policy updates, no checkpoint,
vx 0..0.3 m/s, vy/yaw zero, 1.9 Nm / 4.6 rad/s simulation limits,
standing/action reference 0/+34/-85 degrees, roll or pitch >=45 degrees
termination, no external disturbance. Prior v6.9 results are preserved.
