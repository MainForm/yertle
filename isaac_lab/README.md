# Yertle on Isaac Lab (GPU RL)

자세 편집·삼발 지지 궤적 재생의 통합 실행은 `pose_control.py`를 사용한다.
정적 확인과 중력/접촉 물리 재생을 구분하며, Windows D3D12 실행 명령과 지원 도구는
[pose control guide](pose_control_tools/README.md)에 정리되어 있다.

A second, GPU-accelerated locomotion pipeline for Yertle built on **NVIDIA Isaac
Lab** (Isaac Sim 5.1 + PhysX). It trains a velocity-tracking walking policy with
thousands of parallel environments on an RTX GPU, complementing the CPU
[PyBullet pipeline](../learning/README.md).

```
isaac_lab/
    yertle_cfg.py           Articulation config (spawns the converted USD, leg actuators)
    flat_env_cfg.py         Flat-terrain velocity task (inherits Isaac Lab's locomotion env)
    rough_env_cfg.py        Rough-terrain task (procedural terrain + height scanner)
    rsl_rl_ppo_cfg.py       PPO runner configs (flat and rough)
    distill_env_cfg.py      Distillation env (student obs group without privileged terms)
    rsl_rl_distill_cfg.py   Teacher-student distillation runner config
    train.py                Train on GPU (--task flat|rough)
    play.py                 Roll out / record a trained policy (--task flat|rough)
    distill.py              Distill the trained teacher into a deployable student
    ros2_bridge.py          Expose the Isaac simulation on ROS 2 topics (bidirectional)
```

The environment inherits Isaac Lab's proven `LocomotionVelocityRoughEnvCfg` (the
task that trains ANYmal / Unitree Go2 to walk) and swaps in the Yertle robot,
remapping body names to Yertle's (`base_link`, feet = `*_shin`).

## Prerequisites

Isaac Sim needs Python 3.10/3.11 and an NVIDIA RTX GPU. Create a dedicated
Python 3.11 virtual environment (referred to as `<isaac_venv>` below) and
install per Isaac Lab's pip instructions:

```bash
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
pip install "isaaclab[isaacsim,all]==2.3.2.post1" --extra-index-url https://pypi.nvidia.com
```

## 1. Convert the robot model (URDF -> USD)

```bash
python IsaacLab/scripts/tools/convert_urdf.py \
    simulation/yertle.urdf simulation/usd/yertle.usd --merge-joints --headless
```

This is already done; the USD lives in `simulation/usd/`.

## 2. Train (GPU)

Run from the repo root with the Isaac Sim python and the EULA accepted. Run the
scripts as plain files (not `-m`): on Windows, importing heavy libraries before
the Kit app launches is what avoids the DLL-conflict crash, and the scripts
handle that ordering themselves.

```bat
set OMNI_KIT_ACCEPT_EULA=YES
<isaac_venv>\Scripts\python.exe isaac_lab\train.py --task flat --headless --num_envs 4096 --max_iterations 1500
<isaac_venv>\Scripts\python.exe isaac_lab\train.py --task rough --headless --num_envs 4096 --max_iterations 1000
```

Logs and checkpoints: `isaac_lab/runs/<experiment>/<timestamp>/`. Watch with
`tensorboard --logdir isaac_lab/runs`.

The flat environment defaults to fixed diagonal trot at **2.2 Hz** with a 0.5
stance duty factor over the full vx [-0.5, 0.3] m/s, vy [-0.2, 0.2] m/s, and
yaw-rate [-1.0, 1.0] rad/s command ranges. Command sampling does not exclude
low-speed or straight-line motion. The policy outputs 12 joint-position
actions; it does not select gait frequency. The existing three-channel gait
observation is retained, with trot always selected.

The v6.1 actuator defaults are `effort_limit_sim=1.9` N.m and
`velocity_limit_sim=4.6` rad/s. Training can override these with
`--effort-limit-sim` and `--velocity-limit-sim`; use
`--velocity-limit-sim 5.45` to select the v6.0 speed limit. Saved checkpoints
and training histories are stored separately from these code settings.

The additional `joint_velocity_excess` reward penalizes measured joint speeds
only above **2.30 rad/s** (approximately 1500 encoder steps/s with 4096 steps
per revolution). Its cost is
`sum(max(abs(joint_velocity) / 2.30 - 1, 0)^2)` with initial weight **-0.10**.
The existing `joint_vel_l2` weight remains -0.001. This soft penalty does not
guarantee a speed bound or change the actuator default of 4.6 rad/s; use
`--velocity-limit-sim 2.30` to also set the simulation limit for a new run.
Isaac Lab's reward manager records this term in episode reward logs.

### Random episode start poses (flat recovery training)

Pass repeatable `--reset-pose-json <path>` options to sample a complete joint
pose independently for each environment on **every episode reset**. The pool
contains the default standing pose plus each JSON pose, with equal probability
(one JSON means 50% standing / 50% JSON). Without JSON options, the existing
0.8–1.2 standing-joint randomization remains unchanged. This is a discrete pose
pool, not independent random joint angles. Root-height/yaw reset behavior stays
unchanged: a folded pose initially settles under gravity from the usual spawn
height, rather than being teleported into floor contact.

```bash
/isaac-sim/python.sh isaac_lab/train.py --headless --task flat --num_envs 1024 --max_iterations 2000 --signed-joint-reference --reset-standing-pose-json isaac_lab/configs/standing_api_reference.json --reset-pose-json isaac_lab/configs/seated_api_reference.json --extend-reset-pose-limits
```

The tracked `configs/seated_api_reference.json` uses the current motor API's
raw-2048 seated reference: all shoulders **0 degrees**, thighs **+75 degrees**,
and shins **-160 degrees**. With this option the pool remains 50% standing /
50% seated. `--reset-standing-pose-json configs/standing_api_reference.json`
replaces pool index 0 with the current shoulder 0 / thigh +34 / shin -85 degrees.
For new policies, also pass `--signed-joint-reference` to both train and play
so the action/observation/reward reference matches this reset pose. This new
reference is not interchangeable with existing v6.4/v6.5 checkpoints.
The old default standing pose is NOT added as a third candidate. The two
profiles are literal sim joint angles, not a coordinate-conversion layer: USD
joint axes are unchanged, so sign reversal can change physical geometry.
Without `--signed-joint-reference`, observation/action default offsets stay
at 0 / -0.6 / +0.9 radians for legacy checkpoints. Reset options change only
episode reset states, not the policy's output reference or a hardware API.
Earlier run snapshots used thighs -75 / shins +160; those records
and checkpoints are preserved, not retroactively changed. The JSON affects new
episode resets only, not trained action offsets or an existing model's weights.
For direct launches, include both options shown above; `train.py` without a
JSON option still retains its legacy standing randomization.

Pose JSONs use the calibration editor's schema-1 named joint angles. Limits are
validated for the whole pool before sampling. The seated reference is outside
the original PhysX joint bounds (in particular shin -160 degrees). The explicit
`--extend-reset-pose-limits` flag expands the **runtime hard and soft joint
limits of every environment for the entire run**, covering every candidate
pose, within +/-180 degrees. It does not edit the USD, standing defaults,
12-action position offsets, or actuator torque/speed settings. This expansion
is a simulation experiment, not proof that the corresponding motion is safe
on hardware. Omit it to reject out-of-limit poses.

Flat environments now disable `base_contact` termination and add
`base_contact_penalty` with initial weight **-1.0**: a cost of 1 when base_link
contact-force magnitude exceeds **1 N** in the sensor's recent history, else 0.
Isaac Lab multiplies reward terms by control dt; at 50 Hz this contributes
**-0.02 per contacted control step**. It permits learning recovery, but does not
guarantee standing-up behavior. Time-out and all other rewards remain enabled.
Rough-terrain termination/rewards are not changed by this flat-task change.
The pose pool is saved in `params/env.yaml`, and episode reward logs include
`base_contact_penalty`. With base termination disabled, its old termination-rate
metric is no longer a useful contact-incidence measure.

Bounded native or container integration verification:

```bash
python isaac_lab/tests/check_train_start_pose_sim.py --headless --pose-json <pose.json>
```

## 3. Play / record

```bat
<isaac_venv>\Scripts\python.exe isaac_lab\play.py --task flat --checkpoint <path> --num_envs 16 --video
```

### Manual pose calibration (no policy)

`show_default_pose.py` opens a separate **Yertle pose calibration** window with
12 degree sliders and numeric fields, labeled by joint and physical motor ID.
The body is held fixed and physics stays paused; the tool does not communicate
with servos or change training/default action settings. Every editor joint now
has a **-180 to +180 degree** range; only this paused editor's live articulation
limits are overridden. The source USD/URDF and train/play limits are unchanged.
This static geometry reference does **not** check
balance, contact, self-collision, or motor torque.

The default `--motor-frame reverse-front` uses physical forward=-sim X and
physical right=+sim Y (180-degree yaw). IDs 1/2/3 map to lb thigh/shin/shoulder,
4/5/6 to rb, 7/8/9 to lf, and 10/11/12 to rf. The UI shows physical leg labels
and unchanged sim joint names together. `--motor-frame sim` restores the old
mapping; `--motor-frame swap-front-rear` swaps only front/rear labels while
retaining the old left/right labels. No joint-angle sign is changed by mapping.

```powershell
<isaac_venv>\Scripts\python.exe isaac_lab\show_default_pose.py --rendering_mode performance --experience isaaclab.python.yertle_min_d3d12.kit
```

Use **Restore default standing pose** to reset all joints. **Save pose JSON**
creates a new timestamped file in `isaac_lab/runs/pose_calibration` (override with
`--output-dir`). Files contain named angles in rad/deg and motor-ID mapping, but
leave hardware raw reference values and direction signs unset. Matching a real
pose still requires hardware calibration; a visually similar seated pose alone
is not an exact joint-angle measurement.

To reopen a saved pose, pass `--pose-json <absolute-path-to-pose.json>` at
startup. In the editor, enter a path and click **Load pose JSON**, or use
**Browse...** to choose a local JSON file. Angles are matched by joint name,
not array order. Loading checks schema, all 12 joint names, finite rad values,
the active editor angle limits, and redundant rad/deg data. An invalid file leaves the
existing pose unchanged; loading never changes the default standing reference.
Old JSON is loaded by sim joint name, without permuting its angles. A different
file motor mapping is reported in the UI. New JSON records the motor reference
frame and both editor and original USD limits; values beyond the original limits
are visualization references, not safe training or hardware targets.

Save/load/reset buttons enqueue requests for the outer loop. They never call
simulation pause/update/render from inside a UI callback, avoiding nested
D3D12 rendering on Save. Unknown raw/sign hardware values remain unset.

The live robot viewport is docked on the left and joint controls on the right
after Kit's startup layout completes. Slider/angle-field changes appear on
the next rendered frame while the controls remain open; no closing or applying
a settings dialog is needed. Physics stays paused and the body stays fixed.
For this static editor only, computed PhysX link poses are explicitly mirrored
to the renderer's transform hierarchy. The usual physics-to-Fabric writer is
disabled, since paused tensor kinematics do not automatically update rendered
meshes. This does not change the training/play configuration or source USD.

Bounded verification of actual UI callbacks, articulation/link transforms,
fixed body, reset, and saved JSON:

```bash
/isaac-sim/python.sh isaac_lab/show_default_pose.py --headless --self-test
python -m unittest discover -s isaac_lab/tests -p test_pose_calibration.py -v
```

Native GUI regression with three real mouse-click save/reset/load cycles and
rendered standing/folded viewport captures while the controls remain open:

```powershell
<isaac_venv>\Scripts\python.exe isaac_lab\show_default_pose.py --ui-self-test --rendering_mode performance --experience isaaclab.python.yertle_min_d3d12.kit
```

## 4. Distill to a deployable policy

The flat policy observes the base linear velocity, which the real robot cannot
measure. Distillation trains a student from a deployable observation set (no
base linear velocity, no height scan) to imitate the trained teacher:

```bat
<isaac_venv>\Scripts\python.exe isaac_lab\distill.py --headless --teacher isaac_lab\runs\yertle_flat\<run>\model_1499.pt
```

Result on this robot: student mean reward 40.4 against the teacher's 40.5, with
an imitation loss of about 1e-3.

## 5. ROS 2 bridge

`ros2_bridge.py` publishes `/joint_states` and `/clock` and subscribes to
`/joint_command`, exposing the GPU simulation to any ROS 2 stack. Validated
bidirectionally; see [ros2/README.md](../ros2/README.md) for the demo and the
Windows setup notes.

## Notes

- The default standing pose and actuator gains in `yertle_cfg.py` are starting
  points carried over from the PyBullet env; tune if the robot spawns collapsed.
- The URDF now carries firmware-derived joint limits (revolute joints) and the
  actuator enforces effort/velocity caps, so learned behavior stays within what
  the hardware can execute. An explicit DC-motor actuator model was evaluated
  and rejected as underdamped at these gains (see the comment in
  `yertle_cfg.py` and the technical report).
- A quick model sanity check after any URDF/actuator change: command the
  standing pose with zero action and confirm the base height holds steady.
