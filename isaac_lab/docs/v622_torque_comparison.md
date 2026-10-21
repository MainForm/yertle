# v6.22 / v6.22.2 torque comparison

| Profile | PhysX joint effort cap | PhysX joint velocity cap | Start |
|---|---:|---:|---|
| v6.22 | 0.64 Nm | 2.3 rad/s | preserve existing history / resume saved state |
| v6.22.2 | 1.024 Nm | 2.3 rad/s | fresh, no checkpoint |

The fresh profiles use identical arguments except for effort_limit_sim.
Both target 7000 updates with 1024 environments, seed 0, trot 1.7 Hz and 60%
stance, commands vx [-0.3,0.5], vy [-0.2,0.2], yaw [-1,1], base-contact
termination, 1.5 cm phase clearance and the existing support rewards.
The soft velocity-excess threshold stays 2.3 rad/s; rewards are not reweighted.

1.024 = 0.64 * 1.6. The association with 80% PWM is only the requested
experimental assumption. It is not a calibrated torque/PWM relationship,
and does not implement real-servo PID, acceleration profiles or supply effects.
Limits are anatomical PhysX joint caps, not a coupled motor-side model.

Shared YERTLE_CFG defaults and old profiles are preserved. Model inputs/actions,
motor IDs/signs/calibration, action references and joint axes are unchanged.
Use the explicit profile actuator limits when evaluating either model.

For continuation, select the latest saved checkpoint and require saved
environment state (`--resume-env-state required`). Remaining updates to this
target are `7000 - saved_next_iteration`; the final global iteration is 6999.
Do not overwrite previous result/event directories or resume v6.22 into the
fresh v6.22.2 experiment.
