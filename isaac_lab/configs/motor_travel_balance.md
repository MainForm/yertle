# v6.21: measured motor travel balance and 1 Hz trot

Fresh 7000 updates, 1024 environments, seed 0. All v6.20.3 settings retained
except the fixed gait clock (2.2 -> 1.0 Hz) and this opt-in reward.
No checkpoint is loaded. Existing v6.20.3 process/configuration is not changed.

`--motor-travel-balance` adds `motor_travel_balance_penalty`, weight **-0.5**.
For each leg over a rolling one-cycle window (50 control samples at 1 Hz):

    T_thigh = sum(abs(delta measured thigh servo angle))
    T_shin  = sum(abs(delta measured shin servo angle))
    penalty = mean_legs(((T_thigh - T_shin) / max(T_thigh + T_shin, 5 deg))**2)

Shin servo angle comes from the EXISTING `MotorRawTransform`, including
thigh+shin coupling, signs and calibration. It is not anatomical knee travel
alone. Targets/actions/energy are not used. Measurement is at control rate
(50 Hz), not the 200 Hz physics trace used in the previous diagnostic.

Bounded score 0..1, no positive stand-still reward. Both stationary is zero,
not a locomotion incentive; velocity rewards still provide that incentive.
Only moving commands (XY norm >0.05 m/s OR |yaw| >0.1 rad/s) apply this cost.
Zero-command windows clear history. Partial windows are used immediately;
the first post-reset sample initializes position without counting teleport.
Repeated calls at the same step do not update history twice. Rolling history,
cursor, previous position and cached result are tensor attributes captured by
the existing environment checkpoint adapter.

Matching travel is a heuristic, NOT proof of a natural gait. Small back-and-
forth motion can match travel without matching range of motion; inspect ROM,
tracking and contact behavior after training. No additional ROM cost added.
Isaac Lab scales the weighted score by control dt (0.02 s), as for other terms.

Default frequency remains 2.2 Hz for older profiles. `--trot-frequency-hz 1.0`
changes the shared observation/reward clock together; use the same flag for
v6.21 play. Observation layout/meaning, action reference/scale, axes, motor
mapping and hardware API remain unchanged. This does not hard-limit actions.

Validation: 181 standalone regression tests passed (12 new travel/clock tests).
Actual Docker GPU simulation, 8 environments / 2 PPO updates, saved model_1
and next_iteration=2. New reward logged a finite negative value; checkpoint
contains the 8x50x4x2 history and all reset/cache tensors. Saved environment
diff against v6.20.3's identical small test contains ONLY the new reward and
the explicit frequency field. Kit emitted existing Vulkan graphics teardown
errors after TRAIN_DONE; simulation/update/checkpoint verification passed.
