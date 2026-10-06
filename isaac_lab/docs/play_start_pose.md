# Policy play start pose

Add one of these options to the existing `play.py` command:

```text
--start-pose random
--start-pose default
--pose-json "C:\path\to\saved-pose.json"
```

- `random` is the unchanged legacy behavior and the default when no options
  are supplied (joint positions scaled by 0.8–1.2 around the standing reference).
- `default` uses the exact standing reference, with zero joint velocity:
  shoulder 0, thigh -0.6, shin +0.9 rad.
- `--pose-json` selects `json` automatically. The explicit form is
  `--start-pose json --pose-json <path>`. Use schema-version-1 JSON saved by
  the pose editor. All 12 angles are matched by simulation joint name;
  motor IDs, array order, and motor-frame metadata do not change the mapping.

The selected pose is applied at the initial reset and every automatic episode
reset. Joint velocity is zeroed and position targets are initialized to the
selected pose before policy inference. Original default joint angles (action
offsets and observation references), actuator limits, rewards, and training
configuration are not modified. Base height/orientation/reset randomization
is unchanged; this is a joint-pose option, not a full-body pose initializer.

JSON angles must be within the loaded robot's actual PhysX hard joint limits.
Invalid values are rejected with the joint name and valid range, not clipped
or used to expand the limits. The paused editor's +/-180 degree range does
not grant permission to use those angles in a dynamic simulation. In
particular, a thigh +75 / shin -160 degree reference may be rejected.
Even an accepted seated pose does not guarantee that a walking-only policy
can stand up; balance, ground clearance and the transition require testing.

For explicit **simulation-only** testing of an editor pose outside those
limits, add `--extend-start-pose-limits` alongside `--pose-json`. This expands
only the running play environment's hard limits to include the selected pose
(e.g. knee upper limit 90 -> 160 degrees). Isaac Lab also updates its soft
limit buffers. It does not edit the USD/URDF, training config, pose JSON,
default joint angles or policy action offsets. All other joint limits remain
unchanged, and expansion never shrinks an existing range. Angles beyond
 +/-180 degrees are still rejected. Without this flag, strict rejection is
unchanged. These expanded limits persist for the lifetime of that play process;
this is not a claim of hardware safety or a trained stand-up capability.

Verification without launching Isaac Sim:

```text
python -m unittest discover -s isaac_lab/tests -p test_play_start_pose.py -v
```

Actual Isaac Sim integration (two environments, repeated resets and bounded
policy rollouts for all three modes, preserved action offsets and hard limits):

```text
<isaac_venv>\Scripts\python.exe -u isaac_lab\tests\check_play_start_pose_sim.py --checkpoint <path.pt> --headless --experience isaaclab.python.yertle_min_d3d12.kit --rendering_mode performance
```

Require all four `PLAY_START_POSE_SIM_PASS` lines (default, json, random,
and rejected-invalid-pose). A zero exit status alone is not sufficient:
Kit's shutdown can mask a pending Python exception. Failures are printed
before cleanup as `PLAY_START_POSE_SIM_FAIL` with their traceback.
