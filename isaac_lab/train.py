r"""Train the Yertle flat locomotion policy in Isaac Lab with rsl_rl (GPU).

Run from the repo root with the Isaac Sim python:

    set OMNI_KIT_ACCEPT_EULA=YES
    <isaac_venv>\Scripts\python.exe -m isaac_lab.train --headless --num_envs 4096 --max_iterations 500

Logs and checkpoints go to ``isaac_lab/runs/yertle_flat/<timestamp>``.
"""

import argparse
import os
import sys
from datetime import datetime, timezone

# IMPORTANT (Windows): import torch and rsl_rl (which pulls tensordict) BEFORE
# launching Isaac Sim. Kit and torch both bundle math/OpenMP runtimes; loading
# torch's first avoids a hard DLL-conflict crash when rsl_rl imports later.
import torch  # noqa: F401,E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train Yertle locomotion in Isaac Lab (rsl_rl).")
parser.add_argument("--task", type=str, default="flat", choices=["flat", "rough"])
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=500)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--checkpoint", default=None, help="Optional compatible 55-D model_*.pt to resume.")
parser.add_argument("--enable-disturbance", action="store_true")
parser.add_argument("--force-body-weight", type=float, default=1.0)
parser.add_argument("--push-duration", type=float, default=0.10)
parser.add_argument("--push-interval-min", type=float, default=3.0)
parser.add_argument("--push-interval-max", type=float, default=6.0)
parser.add_argument("--perturbed-env-fraction", type=float, default=0.30)
parser.add_argument("--lin-vel-x-min", type=float, default=None, help="Optional lower bound for commanded vx (m/s).")
parser.add_argument("--lin-vel-x-max", type=float, default=None, help="Optional upper bound for commanded vx (m/s).")
parser.add_argument("--vx-only", action="store_true", help="Set vy/yaw commands to zero and disable heading commands.")
parser.add_argument("--signed-joint-reference", action="store_true", help="New-policy coordinates: reverse thigh/shin axes and use 0/+34.4/-51.6 deg action/observation/reward reference. Not compatible with legacy checkpoints.")
parser.add_argument("--body-tilt-termination-deg", type=float, default=None, help="Terminate when absolute roll OR pitch reaches this angle; (0,90] degrees.")
parser.add_argument("--velocity-limit-sim", type=float, default=None, help="Override PhysX actuator velocity limit (rad/s).")
parser.add_argument("--effort-limit-sim", type=float, default=None, help="Override PhysX actuator effort limit (N.m).")
parser.add_argument("--reset-pose-json", action="append", default=[], help="Repeatable pose JSON; sample standing + these poses uniformly on every episode reset.")
parser.add_argument("--reset-standing-pose-json", help="Replace only pool index 0's standing reset pose; preserve trained observation/action reference angles.")
parser.add_argument("--extend-reset-pose-limits", action="store_true", help="Explicitly expand simulation joint limits to include all reset poses for the entire run.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if (args_cli.lin_vel_x_min is None) != (args_cli.lin_vel_x_max is None):
    parser.error("--lin-vel-x-min and --lin-vel-x-max must be supplied together.")
if args_cli.body_tilt_termination_deg is not None and not 0 < args_cli.body_tilt_termination_deg <= 90:
    parser.error("--body-tilt-termination-deg must be in (0,90].")
if args_cli.extend_reset_pose_limits and not args_cli.reset_pose_json and not args_cli.reset_standing_pose_json:
    parser.error("--extend-reset-pose-limits requires --reset-pose-json or --reset-standing-pose-json.")
# Validate saved poses before launching Kit. Direct script and -m both supported.
if __package__:
    from .play_start_pose import load_pose_json
else:
    from play_start_pose import load_pose_json
try:
    reset_poses = [load_pose_json(path) for path in args_cli.reset_pose_json]
    standing_reset_pose = load_pose_json(args_cli.reset_standing_pose_json) if args_cli.reset_standing_pose_json else None
except (OSError, ValueError) as exc:
    parser.error(str(exc))

# Launch Isaac Sim first; everything below needs the running app.
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402

from isaaclab.utils.io import dump_yaml  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

# Make the repo root importable so `import isaac_lab` works, then register tasks.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import isaac_lab  # noqa: F401,E402  (registers the gym task)
from isaac_lab.disturbance_cfg import TimedPlanarForceImpulse  # noqa: E402
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg  # noqa: E402
from isaac_lab.rough_env_cfg import YertleRoughEnvCfg  # noqa: E402
from isaac_lab.rsl_rl_ppo_cfg import YertleFlatPPORunnerCfg, YertleRoughPPORunnerCfg  # noqa: E402
from isaac_lab.train_start_pose import configure_reset_pose_pool  # noqa: E402
from isaac_lab.train_terminations import configure_body_tilt_termination  # noqa: E402
from isaac_lab.signed_joint_reference import configure_signed_joint_reference  # noqa: E402

_TASKS = {
    "flat": ("Isaac-Velocity-Flat-Yertle-v0", YertleFlatEnvCfg, YertleFlatPPORunnerCfg),
    "rough": ("Isaac-Velocity-Rough-Yertle-v0", YertleRoughEnvCfg, YertleRoughPPORunnerCfg),
}


def main():
    TASK, EnvCfg, RunnerCfg = _TASKS[args_cli.task]
    env_cfg = EnvCfg()
    if args_cli.task == "flat" and args_cli.enable_disturbance:
        env_cfg.sim.physx.enable_external_forces_every_iteration = True
        env_cfg.events.base_external_force_torque = None
        env_cfg.events.planar_force_impulse = TimedPlanarForceImpulse.event_cfg(
            control_dt=env_cfg.decimation * env_cfg.sim.dt,
            enabled=True,
            force_body_weight=args_cli.force_body_weight,
            duration_s=args_cli.push_duration,
            interval_s=(args_cli.push_interval_min, args_cli.push_interval_max),
            perturbed_env_fraction=args_cli.perturbed_env_fraction,
        )
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    if args_cli.signed_joint_reference:
        configure_signed_joint_reference(env_cfg)
    configure_reset_pose_pool(env_cfg, reset_poses, args_cli.extend_reset_pose_limits, standing_reset_pose)
    if args_cli.lin_vel_x_min is not None:
        env_cfg.commands.base_velocity.ranges.lin_vel_x = (args_cli.lin_vel_x_min, args_cli.lin_vel_x_max)
    if args_cli.vx_only:
        env_cfg.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        env_cfg.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        env_cfg.commands.base_velocity.heading_command = False
        env_cfg.commands.base_velocity.rel_heading_envs = 0.0
    configure_body_tilt_termination(env_cfg, args_cli.body_tilt_termination_deg)
    legs = env_cfg.scene.robot.actuators["legs"]
    if args_cli.velocity_limit_sim is not None:
        legs.velocity_limit_sim = args_cli.velocity_limit_sim
    if args_cli.effort_limit_sim is not None:
        legs.effort_limit_sim = args_cli.effort_limit_sim

    agent_cfg = RunnerCfg()
    agent_cfg.max_iterations = args_cli.max_iterations
    agent_cfg.seed = args_cli.seed

    log_dir = os.path.join(
        os.path.dirname(__file__), "runs", agent_cfg.experiment_name,
        datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S"),
    )
    os.makedirs(log_dir, exist_ok=True)

    env = gym.make(TASK, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", None))

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    if args_cli.checkpoint:
        runner.load(args_cli.checkpoint, load_optimizer=True, map_location=agent_cfg.device)
        print(f"RESUME_CHECKPOINT {args_cli.checkpoint}", flush=True)
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)

    print(f"TRAIN_START task={TASK} num_envs={args_cli.num_envs} iters={args_cli.max_iterations}", flush=True)
    print(
        "COMMAND_RANGES "
        f"vx={env_cfg.commands.base_velocity.ranges.lin_vel_x} "
        f"vy={env_cfg.commands.base_velocity.ranges.lin_vel_y} "
        f"yaw={env_cfg.commands.base_velocity.ranges.ang_vel_z}",
        flush=True,
    )
    print(
        f"ACTUATOR_LIMITS effort_limit_sim={legs.effort_limit_sim} "
        f"velocity_limit_sim={legs.velocity_limit_sim}",
        flush=True,
    )
    print(f"TRAIN_LOGDIR {log_dir}", flush=True)
    print(f"TRAIN_SIGNED_JOINT_REFERENCE enabled={args_cli.signed_joint_reference}", flush=True)
    print(f"TRAIN_BODY_TILT_TERMINATION deg={args_cli.body_tilt_termination_deg} roll_or_pitch=True", flush=True)
    print(f"TRAIN_RESET_POSES standing_plus_json={args_cli.reset_pose_json} "
          f"standing_override={args_cli.reset_standing_pose_json} "
          f"uniform_pool_size={len(reset_poses) + 1 if reset_poses or standing_reset_pose else 0} "
          f"extend_limits={args_cli.extend_reset_pose_limits}", flush=True)
    runner.learn(
        num_learning_iterations=agent_cfg.max_iterations,
        init_at_random_ep_len=args_cli.checkpoint is None,
    )
    print("TRAIN_DONE", flush=True)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
