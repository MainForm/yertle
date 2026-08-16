r"""Roll out / record a trained Yertle Isaac Lab policy.

    set OMNI_KIT_ACCEPT_EULA=YES
    <isaac_venv>\Scripts\python.exe isaac_lab\play.py --checkpoint <path.pt> --num_envs 32 --video
"""

import argparse
import os
import sys

# Windows DLL-order fix: import torch/rsl_rl before launching Isaac Sim. Also
# pre-import h5py so its bundled HDF5 DLL loads before Isaac Sim's rendering kit
# pulls in a conflicting one (video path imports isaaclab.utils.datasets -> h5py).
import h5py  # noqa: F401
import torch  # noqa: F401
from rsl_rl.runners import OnPolicyRunner

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play a trained Yertle policy in Isaac Lab.")
parser.add_argument("--task", type=str, default="flat", choices=["flat", "rough"])
parser.add_argument("--checkpoint", type=str, required=True, help="Path to a saved model_*.pt")
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--steps", type=int, default=600)
parser.add_argument("--video", action="store_true")
parser.add_argument("--video_length", type=int, default=400)
parser.add_argument("--keyboard", action="store_true", help="Control vx, vy, and yaw from this terminal.")
parser.add_argument("--vx", type=float, default=0.0)
parser.add_argument("--vy", type=float, default=0.0)
parser.add_argument("--yaw-rate", dest="yaw_rate", type=float, default=0.0)
parser.add_argument("--step-v", type=float, default=0.05)
parser.add_argument("--step-yaw", type=float, default=0.10)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.video:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import isaac_lab  # noqa: F401,E402  (registers tasks)
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY  # noqa: E402
from isaac_lab.rough_env_cfg import YertleRoughEnvCfg_PLAY  # noqa: E402
from isaac_lab.rsl_rl_ppo_cfg import YertleFlatPPORunnerCfg, YertleRoughPPORunnerCfg  # noqa: E402

try:
    import msvcrt
except ImportError:  # pragma: no cover
    msvcrt = None


VX_RANGE = (-0.5, 0.3)
VY_RANGE = (-0.2, 0.2)
YAW_RANGE = (-1.0, 1.0)

_TASKS = {
    "flat": ("Isaac-Velocity-Flat-Yertle-Play-v0", YertleFlatEnvCfg_PLAY, YertleFlatPPORunnerCfg),
    "rough": ("Isaac-Velocity-Rough-Yertle-Play-v0", YertleRoughEnvCfg_PLAY, YertleRoughPPORunnerCfg),
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def read_key():
    if msvcrt is None or not msvcrt.kbhit():
        return None
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0") and msvcrt.kbhit():
        key += msvcrt.getwch()
    return key.lower()


def update_from_keyboard(command, step_v, step_yaw):
    key = read_key()
    if key is None:
        return False, False
    if key == "w": command[0] = clamp(command[0] + step_v, *VX_RANGE)
    elif key == "s": command[0] = clamp(command[0] - step_v, *VX_RANGE)
    elif key == "q": command[1] = clamp(command[1] + step_v, *VY_RANGE)
    elif key == "e": command[1] = clamp(command[1] - step_v, *VY_RANGE)
    elif key == "a": command[2] = clamp(command[2] + step_yaw, *YAW_RANGE)
    elif key == "d": command[2] = clamp(command[2] - step_yaw, *YAW_RANGE)
    elif key == " ": command[:] = [0.0, 0.0, 0.0]
    elif key in ("x", "\x1b"): return False, True
    else: return False, False
    return True, False


def apply_command(base_env, command_tensor):
    term = base_env.command_manager.get_term("base_velocity")
    term.vel_command_b[:, :] = command_tensor
    term.is_standing_env[:] = False
    if hasattr(term, "is_heading_env"):
        term.is_heading_env[:] = False
    term.time_left[:] = 1.0e6


def main():
    TASK, EnvCfg, RunnerCfg = _TASKS[args_cli.task]
    env_cfg = EnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.keyboard:
        env_cfg.commands.base_velocity.resampling_time_range = (1.0e6, 1.0e6)
        env_cfg.commands.base_velocity.rel_standing_envs = 0.0
        env_cfg.commands.base_velocity.rel_heading_envs = 0.0
        env_cfg.commands.base_velocity.heading_command = False
        env_cfg.commands.base_velocity.debug_vis = False
    agent_cfg = RunnerCfg()

    env = gym.make(TASK, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    if args_cli.video:
        video_folder = os.path.join(os.path.dirname(os.path.abspath(args_cli.checkpoint)), "videos_play")
        env = gym.wrappers.RecordVideo(
            env,
            video_folder=video_folder,
            step_trigger=lambda step: step == 0,
            video_length=args_cli.video_length,
            disable_logger=True,
        )
        print(f"PLAY_VIDEO_DIR {video_folder}", flush=True)

    base_env = env.unwrapped
    env = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", None))

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs = env.get_observations()
    if isinstance(obs, tuple):
        obs = obs[0]

    print("PLAY_START", flush=True)
    command = [clamp(args_cli.vx, *VX_RANGE), clamp(args_cli.vy, *VY_RANGE), clamp(args_cli.yaw_rate, *YAW_RANGE)]
    command_tensor = torch.tensor(command, device=base_env.device, dtype=torch.float32).repeat(base_env.num_envs, 1)
    if args_cli.keyboard:
        apply_command(base_env, command_tensor)
        print("W/S: vx, Q/E: vy, A/D: yaw, Space: stop, X: quit", flush=True)
        print(f"COMMAND vx={command[0]:.2f} vy={command[1]:.2f} yaw={command[2]:.2f}", flush=True)
    with torch.inference_mode():
        for _ in range(args_cli.steps):
            if args_cli.keyboard:
                changed, should_quit = update_from_keyboard(command, args_cli.step_v, args_cli.step_yaw)
                if should_quit:
                    break
                command_tensor[:] = torch.tensor(command, device=base_env.device, dtype=torch.float32)
                apply_command(base_env, command_tensor)
                if changed:
                    print(f"COMMAND vx={command[0]:.2f} vy={command[1]:.2f} yaw={command[2]:.2f}", flush=True)
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            if isinstance(obs, tuple):
                obs = obs[0]
    print("PLAY_DONE", flush=True)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
