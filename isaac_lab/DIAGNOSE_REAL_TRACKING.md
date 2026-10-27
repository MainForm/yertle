# v6.22.2 real/simulation tracking comparison

`diagnose_real_tracking.py` reuses the environment, signed-reference setup,
start-pose helper, command application, RSL-RL wrapper and policy-loading path
used by `play.py`. It does not open a serial port or modify training, deployment
calibration, observation semantics, motor mapping or the active ONNX model.

This experiment uses model_6999.pt from v6.22.2,55 observations/12 actions,
reference0/+34/-85deg and action scale0.25. Saved training settings are
effort_limit_sim1.024Nm,velocity_limit_sim2.3rad/s,implicit stiffness25,
damping0.5,control20ms,gait1.7Hz/duty0.6. They are isolated runtime overrides;
no configuration defaults are edited. Test seed0, five separated environments,
no reset noise/push, commands vx=vy=yaw=0 matching the actual recorded trial.

The prepared dataset is mounted at
`/workspace/yertle-artifacts/real_sim_tracking_20261027`. It contains the
checkpoint, original env.yaml, calibrated C++ API snapshot/exported mapping,
current production C PID compiled as libpid.so, and real_replay.json. Source
log: robot_20261026_225235_33236.jsonl, SHA256
7cbe1cdc342cf522887ecf0ee48380e2f78d44472f71655119a907bb251b6d0e.
The628 actions and actual60byte position/speed/ACC packets are paired and every
converted target is checked against its real transmitted raw positions.
All628 recorded55-element observations also reproduce the logged ONNX output
within max7.16e-7. Absolute angle-to-raw and action-to-angle are checked separately.

## Run

```powershell
docker exec yertle-isaaclab-container-1 /isaac-sim/python.sh -u /workspace/yertle/isaac_lab/diagnose_real_tracking.py --data-dir /workspace/yertle-artifacts/real_sim_tracking_20261027 --headless --device cuda:0 --label repeat_5ms
```

For the numerical sensitivity comparison add `--physics-dt 0.001 --label repeat_1ms`.
For a short command check add `--steps 10 --label command_check`.
Outputs are `<label>_trace.npz` and `<label>_summary.json` in the dataset directory.
These assets/results are deliberately outside Git; the script and this guide
are versioned. `comparison.json` in the prepared dataset includes pre-step
metrics and hashes; default summaries contain post-step metrics.

## Cases

1. Policy recomputes actions from simulation state, normal body velocity.
2. Same closed loop with only observed linear velocity XYZ forced to zero;
   IMU/body angular velocity is retained, matching the PC test option.
3. Replay recorded real actions through the native implicit actuator.
4. Replay real goals through the current F103 position-reference generator,
   then have native implicit actuators follow that generated reference.
5. Closed-loop policy with zero observed linear velocity and the same reference
   generator before the native implicit actuator.

The reference generator is the production C controller with P5/I3/D0.1,
I-cap100,Speed200..3000,ACC100. Its PWM output is **not** converted to torque.
These are not simulations of an identified physical PWM motor. When its guard
fails, retained later trace samples are counterfactual; reported valid metrics
stop at the first guard and must not be used to claim continued operation.

## Verified outcome and limits

Both5ms and1ms physics runs completed628 control steps (12.56s). Native
closed-loop cases1/2 had no base-contact force >1N at either resolution.
Pre-step mean absolute raw error was68.47/73.93 at5ms and67.54/73.93 at1ms.
The real log's341 command-sequence-matched PID sets had final-goal mean error
132.89raw, profile-gap113.49raw, and internal PID error42.43raw.
These are different streams/sample counts, not a controlled matched trial.

Replay case3 fell (base contact1.42s at5ms,2.78s at1ms), despite smaller mean
position error. The real robot was held then grounded at an unrecorded time;
simulation starts on the ground. Replaying actions without the same state and
support does not reproduce the real closed loop or prove its cause.

Reference-generator cases are sensitive to the physics interval: case4 first
guard1.18s at5ms, but no guard at1ms (it still falls); case5 no guard at5ms,
first guard2.38s at1ms. Thus these approximations do not establish that PID
tuning alone explains the hardware failure. The observed real profile gap is
an additional factor to inspect independently of motor torque/gains.

Only headless computation was validated. The container logged Vulkan graphics
initialization/probe errors; CUDA/PhysX stepping and inference completed with
exit0 and saved results. No GUI rendering, hardware motion, flashing, or actual
return-to2048 was tested. The new return/temperature firmware needs flashing.
