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
parser.add_argument("--push-body-weight", type=float, default=0.20, help="Manual push strength as a fraction of body weight.")
parser.add_argument("--push-duration", type=float, default=0.10)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.video:
    args_cli.enable_cameras = True
if not 0.05 <= args_cli.push_body_weight <= 1.00:
    parser.error("--push-body-weight must be within 0.05..1.00")
if args_cli.push_duration <= 0.0:
    parser.error("--push-duration must be positive")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402

from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import isaac_lab  # noqa: F401,E402  (registers tasks)
from isaac_lab import phase_generator  # noqa: E402
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
GAIT_MODE_NAMES = ("walk", "trot", "bound")
PUSH_KEYS = {"i": (1.0, 0.0, "+X"), "k": (-1.0, 0.0, "-X"), "j": (0.0, 1.0, "+Y"), "l": (0.0, -1.0, "-Y")}

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
        return False, False, None, None
    if key == "w": command[0] = clamp(command[0] + step_v, *VX_RANGE)
    elif key == "s": command[0] = clamp(command[0] - step_v, *VX_RANGE)
    elif key == "q": command[1] = clamp(command[1] + step_v, *VY_RANGE)
    elif key == "e": command[1] = clamp(command[1] - step_v, *VY_RANGE)
    elif key == "a": command[2] = clamp(command[2] + step_yaw, *YAW_RANGE)
    elif key == "d": command[2] = clamp(command[2] - step_yaw, *YAW_RANGE)
    elif key == " ": command[:] = [0.0, 0.0, 0.0]
    elif key in PUSH_KEYS: return False, False, PUSH_KEYS[key], None
    elif key == "[": return False, False, None, -0.10
    elif key == "]": return False, False, None, 0.10
    elif key in ("x", "\x1b"): return False, True, None, None
    else: return False, False, None, None
    return True, False, None, None


def apply_command(base_env, command_tensor):
    term = base_env.command_manager.get_term("base_velocity")
    term.vel_command_b[:, :] = command_tensor
    term.is_standing_env[:] = False
    if hasattr(term, "is_heading_env"):
        term.is_heading_env[:] = False
    term.time_left[:] = 1.0e6


def gait_status(base_env):
    phase, mode_ids, frequencies, duties, _ = phase_generator._gait_state(base_env, "base_velocity")
    mode_id = int(mode_ids[0].item())
    label = GAIT_MODE_NAMES[mode_id]
    if mode_id == 2:
        direction = "front->rear" if base_env.command_manager.get_command("base_velocity")[0, 0].item() > 0.0 else "rear->front"
        label = f"bound ({direction})"
    return f"GAIT mode={label} frequency={frequencies[0].item():.2f}Hz duty={duties[0].item():.2f} phase={phase[0].item():.2f}"


class ForcePulse:
    """Apply a short manual planar force pulse to the Yertle body."""

    def __init__(self, base_env, fraction, duration_s):
        self.base_env = base_env
        self.fraction = fraction
        self.env_ids = torch.arange(base_env.num_envs, device=base_env.device, dtype=torch.long)
        self.asset = base_env.scene["robot"]
        self.asset_cfg = SceneEntityCfg("robot", body_names=["base_link"])
        self.asset_cfg.resolve(base_env.scene)
        self.remaining = 0
        self.duration_steps = max(1, round(duration_s / base_env.step_dt))
        self.force_w = torch.zeros(base_env.num_envs, 3, device=base_env.device)

    def set_fraction(self, fraction):
        self.fraction = clamp(fraction, 0.05, 1.00)

    def trigger(self, x, y, label):
        direction = torch.tensor((x, y, 0.0), device=self.base_env.device)
        direction /= torch.linalg.vector_norm(direction)
        masses = self.asset.root_physx_view.get_masses()[self.env_ids.cpu()].sum(dim=1).to(self.base_env.device)
        self.force_w = masses.unsqueeze(-1) * (self.fraction * 9.81) * direction
        self.remaining = self.duration_steps
        print(f"PUSH direction={label} force={self.fraction:.2f}*m*g duration={self.duration_steps * self.base_env.step_dt:.2f}s", flush=True)

    def apply(self):
        if self.remaining <= 0:
            self.force_w.zero_()
        forces = self.force_w.unsqueeze(1)
        self.asset.permanent_wrench_composer.set_forces_and_torques(
            forces=forces,
            torques=torch.zeros_like(forces),
            body_ids=self.asset_cfg.body_ids,
            env_ids=self.env_ids,
            is_global=True,
        )
        if self.remaining > 0:
            self.remaining -= 1


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
    pulse = ForcePulse(base_env, args_cli.push_body_weight, args_cli.push_duration)

    obs = env.get_observations()
    if isinstance(obs, tuple):
        obs = obs[0]

    print("PLAY_START", flush=True)
    command = [clamp(args_cli.vx, *VX_RANGE), clamp(args_cli.vy, *VY_RANGE), clamp(args_cli.yaw_rate, *YAW_RANGE)]
    command_tensor = torch.tensor(command, device=base_env.device, dtype=torch.float32).repeat(base_env.num_envs, 1)
    if args_cli.keyboard:
        apply_command(base_env, command_tensor)
        print("W/S: vx, Q/E: vy, A/D: yaw, I/K/J/L: push, [/]: push strength, Space: stop, X: quit", flush=True)
        print(f"COMMAND vx={command[0]:.2f} vy={command[1]:.2f} yaw={command[2]:.2f}", flush=True)
        print(f"PUSH_STRENGTH {pulse.fraction:.2f}*m*g", flush=True)
        if args_cli.task == "flat":
            print(gait_status(base_env), flush=True)
    with torch.inference_mode():
        for _ in range(args_cli.steps):
            if args_cli.keyboard:
                changed, should_quit, push, push_delta = update_from_keyboard(command, args_cli.step_v, args_cli.step_yaw)
                if should_quit:
                    break
                if push_delta is not None:
                    pulse.set_fraction(pulse.fraction + push_delta)
                    print(f"PUSH_STRENGTH {pulse.fraction:.2f}*m*g", flush=True)
                if push is not None:
                    pulse.trigger(*push)
                command_tensor[:] = torch.tensor(command, device=base_env.device, dtype=torch.float32)
                apply_command(base_env, command_tensor)
                if changed:
                    print(f"COMMAND vx={command[0]:.2f} vy={command[1]:.2f} yaw={command[2]:.2f}", flush=True)
                    if args_cli.task == "flat":
                        print(gait_status(base_env), flush=True)
            pulse.apply()
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            if isinstance(obs, tuple):
                obs = obs[0]
    pulse.force_w.zero_()
    pulse.apply()
    print("PLAY_DONE", flush=True)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
