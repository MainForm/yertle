r"""Roll out / record a trained Yertle Isaac Lab policy.

    set OMNI_KIT_ACCEPT_EULA=YES
    <isaac_venv>\Scripts\python.exe isaac_lab\play.py --checkpoint <path.pt> --num_envs 32 --video
"""

import argparse
import json
import os
import sys

if __package__:
    from .play_start_pose import configure_start_pose, select_start_pose
    from .play_controls import (VX_RANGE, VY_RANGE, YAW_RANGE, clamp, apply_command,
                                handle_key, configure_play_terminations, reset_play_environment, clear_manual_push)
else:
    from play_start_pose import configure_start_pose, select_start_pose
    from play_controls import (VX_RANGE, VY_RANGE, YAW_RANGE, clamp, apply_command,
                               handle_key, configure_play_terminations, reset_play_environment, clear_manual_push)

# Windows DLL-order fix: import torch/rsl_rl before launching Isaac Sim. Also
# pre-import h5py so its bundled HDF5 DLL loads before Isaac Sim's rendering kit
# pulls in a conflicting one (video path imports isaaclab.utils.datasets -> h5py).
import h5py  # noqa: F401
import torch  # noqa: F401
from rsl_rl.runners import OnPolicyRunner

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play a trained Yertle policy in Isaac Lab.")
parser.add_argument('--trot-frequency-hz', type=float, default=None, help='Fixed gait clock used for training; match the checkpoint profile.')
parser.add_argument('--trot-duty-factor', type=float, default=None, help='Stance fraction [0.5,1); must match training. Legacy default is 0.5.')
parser.add_argument('--effort-limit-sim', type=float, default=None, help='PhysX joint torque limit in Nm; match the checkpoint training profile.')
parser.add_argument('--velocity-limit-sim', type=float, default=None, help='PhysX joint velocity limit in rad/s; match the checkpoint training profile.')
parser.add_argument("--task", type=str, default="flat", choices=["flat", "rough"])
parser.add_argument('--trot-support-penalties', action='store_true', help='Enable v6.20.3 support reward terms for evaluation; does not change policy inputs/actions.')
parser.add_argument('--disable-rewards', action='store_true', help='Skip reward term calculations during playback; reward values become zero.')
parser.add_argument('--explicit-linked-motor', action='store_true', help='Use linked motor PID with 20ms PWM hold, 1ms back-EMF physics, solver 64, and motor-space penalties (flat task).')
parser.add_argument('--sim2real-contact', action='store_true', help='Use the training foot contact and body center-of-mass profile.')
parser.add_argument('--motor-model-path', type=str, help='Load the recorded motor model used by the training run.')
parser.add_argument("--checkpoint", type=str, required=True, help="Path to a saved model_*.pt")
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--steps", type=int, default=600)
parser.add_argument("--zero-base-linear-velocity", action="store_true",
                    help="Feed zero XYZ base linear velocity to the policy; leave physical velocity and commands unchanged.")
parser.add_argument("--signed-joint-reference", action="store_true", help="Use 0/+34/-85 deg reference with original axes and reward-only position envelope (no PhysX position stops). Existing fixed-stop checkpoints see different physics; not for original-reference v6.0 policies.")
parser.add_argument("--start-pose", choices=["random", "default", "json"], default=None,
                    help="Joint pose at every reset: random (legacy), default (exact standing), or json.")
parser.add_argument("--pose-json", type=str,
                    help="Saved editor pose JSON; implies --start-pose json if the mode is omitted.")
parser.add_argument("--extend-start-pose-limits", action="store_true",
                    help="Simulation-only: expand runtime joint limits just enough to include the JSON pose.")
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
if args_cli.trot_frequency_hz is not None and not 0.1 <= args_cli.trot_frequency_hz <= 10.0:
    parser.error('--trot-frequency-hz must be finite and within 0.1..10 Hz')
if args_cli.trot_duty_factor is not None and not 0.5 <= args_cli.trot_duty_factor < 1.0:
    parser.error('--trot-duty-factor must be finite and within [0.5,1.0)')
if args_cli.explicit_linked_motor and (args_cli.task != 'flat' or not args_cli.signed_joint_reference):
    parser.error('--explicit-linked-motor requires --task flat and --signed-joint-reference')
if args_cli.explicit_linked_motor and (args_cli.velocity_limit_sim is not None or args_cli.effort_limit_sim is not None):
    parser.error('--explicit-linked-motor uses motor limits; do not combine with joint --effort-limit-sim/--velocity-limit-sim')
for name in ('effort_limit_sim', 'velocity_limit_sim'):
    value = getattr(args_cli, name)
    if value is not None and not 0.0 < value < float('inf'):
        parser.error('--' + name.replace('_', '-') + ' must be finite and positive')
try:
    start_pose_mode, start_joint_angles = select_start_pose(args_cli.start_pose, args_cli.pose_json)
    if args_cli.extend_start_pose_limits and start_pose_mode != "json":
        raise ValueError("--extend-start-pose-limits requires --pose-json.")
except (OSError, ValueError) as exc:
    parser.error(str(exc))
if args_cli.video:
    args_cli.enable_cameras = True
if not 0.05 <= args_cli.push_body_weight <= 1.00:
    parser.error("--push-body-weight must be within 0.05..1.00")
if args_cli.push_duration <= 0.0:
    parser.error("--push-duration must be positive")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402


class StreamingRecordVideo(gym.wrappers.RecordVideo):
    """Write frames as they arrive instead of retaining the whole video."""
    def start_recording(self, video_name):
        import imageio.v2 as imageio
        self.recording = True
        self._video_name = video_name
        self._stream = imageio.get_writer(
            os.path.join(self.video_folder, video_name + '.mp4'),
            fps=self.frames_per_sec, codec='libx264', quality=8, macro_block_size=1,
        )

    def _capture_frame(self):
        frame = self.env.render()
        if isinstance(frame, list):
            if not frame:
                return
            frame = frame[-1]
        self._stream.append_data(frame)

    def stop_recording(self):
        self._stream.close()
        self.recording = False
        self._video_name = None

from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
if args_cli.motor_model_path:
    import importlib.util
    import hashlib
    motor_path = os.path.abspath(args_cli.motor_model_path)
    spec = importlib.util.spec_from_file_location('isaac_lab.motor_drive_math', motor_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules[spec.name] = module
    with open(motor_path, 'rb') as motor_file:
        print('PLAY_MOTOR_MODEL sha256=' + hashlib.sha256(motor_file.read()).hexdigest(), flush=True)
import isaac_lab  # noqa: F401,E402  (registers tasks)
from isaac_lab import phase_generator  # noqa: E402
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY  # noqa: E402
from isaac_lab.rough_env_cfg import YertleRoughEnvCfg_PLAY  # noqa: E402
from isaac_lab.rsl_rl_ppo_cfg import YertleFlatPPORunnerCfg, YertleRoughPPORunnerCfg  # noqa: E402
from isaac_lab.signed_joint_reference import configure_signed_joint_reference  # noqa: E402
from isaac_lab.flat_env_cfg_reward import configure_trot_support_rewards  # noqa: E402

try:
    import msvcrt
except ImportError:  # pragma: no cover
    msvcrt = None


GAIT_MODE_NAMES = ("walk", "trot", "bound")


def zero_base_linear_velocity_observation(env):
    """Zero only policy XYZ linear-velocity observations, not the robot state."""
    return torch.zeros((env.num_envs, 3), device=env.device)


_TASKS = {
    "flat": ("Isaac-Velocity-Flat-Yertle-Play-v0", YertleFlatEnvCfg_PLAY, YertleFlatPPORunnerCfg),
    "rough": ("Isaac-Velocity-Rough-Yertle-Play-v0", YertleRoughEnvCfg_PLAY, YertleRoughPPORunnerCfg),
}


def read_key():
    if msvcrt is None or not msvcrt.kbhit():
        return None
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0") and msvcrt.kbhit():
        key += msvcrt.getwch()
    return key.lower()


def update_from_keyboard(command, step_v, step_yaw):
    return handle_key(read_key(), command, step_v, step_yaw)


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
    env_cfg.episode_length_s = 30.0
    env_cfg.viewer.resolution = (640, 360)
    env_cfg.viewer.origin_type = "asset_root"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.eye = (0.9, 0.9, 0.55)
    env_cfg.viewer.lookat = (0.0, 0.0, 0.0)
    if args_cli.explicit_linked_motor:
        from isaac_lab.linked_motor_training import configure_linked_motor_actuator
        configure_linked_motor_actuator(env_cfg)
        drive = env_cfg.scene.robot.actuators['legs']
        print(f'PLAY_EXPLICIT_LINKED_MOTOR torque_Nm=1.024 speed_cutoff_rad_s=4.6 free_speed_rad_s={drive.motor_free_speed} physics_dt=0.001 solver_position=64 control_period=0.020 pid_pwm={drive.kp_pwm},{drive.ki_pwm},{drive.kd_pwm} integral_limit_pwm={drive.integral_limit_pwm} speed_is_not_hard_limit=True', flush=True)
    if args_cli.sim2real_contact:
        from isaac_lab.sim2real_contact import configure_sim2real_contact
        configure_sim2real_contact(env_cfg)
    if args_cli.zero_base_linear_velocity:
        env_cfg.observations.policy.base_lin_vel.func = zero_base_linear_velocity_observation
    print(f"PLAY_BASE_LINEAR_VELOCITY_INPUT {'zero' if args_cli.zero_base_linear_velocity else 'measured'}", flush=True)
    if args_cli.trot_frequency_hz is not None:
        env_cfg.trot_frequency_hz = args_cli.trot_frequency_hz
    if args_cli.trot_duty_factor is not None:
        env_cfg.trot_duty_factor = args_cli.trot_duty_factor
    if args_cli.trot_support_penalties:
        configure_trot_support_rewards(env_cfg)
    env_cfg.scene.num_envs = args_cli.num_envs
    # AppLauncher selects the app device, but does not override this separately
    # constructed environment config. Honor --device for physics and inference.
    env_cfg.sim.device = args_cli.device
    legs = env_cfg.scene.robot.actuators['legs']
    if args_cli.effort_limit_sim is not None:
        legs.effort_limit_sim = args_cli.effort_limit_sim
    if args_cli.velocity_limit_sim is not None:
        legs.velocity_limit_sim = args_cli.velocity_limit_sim
    print(f'PLAY_ACTUATOR_LIMITS effort_limit_sim={legs.effort_limit_sim} velocity_limit_sim={legs.velocity_limit_sim}', flush=True)
    if args_cli.signed_joint_reference:
        configure_signed_joint_reference(env_cfg)
    print(f"PLAY_SIGNED_JOINT_REFERENCE enabled={args_cli.signed_joint_reference}", flush=True)
    configure_play_terminations(env_cfg)
    print("PLAY_TERMINATIONS base_contact=disabled time_out=unchanged", flush=True)
    configure_start_pose(env_cfg, start_pose_mode, start_joint_angles,
                         extend_limits=args_cli.extend_start_pose_limits)
    print(f"PLAY_START_POSE mode={start_pose_mode} file={args_cli.pose_json or '-'}", flush=True)
    if True:
        env_cfg.commands.base_velocity.resampling_time_range = (1.0e6, 1.0e6)
        env_cfg.commands.base_velocity.rel_standing_envs = 0.0
        env_cfg.commands.base_velocity.rel_heading_envs = 0.0
        env_cfg.commands.base_velocity.heading_command = False
        env_cfg.commands.base_velocity.debug_vis = False
    agent_cfg = RunnerCfg()
    agent_cfg.device = env_cfg.sim.device

    env = gym.make(TASK, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    if args_cli.video:
        checkpoint_name = os.path.splitext(os.path.basename(args_cli.checkpoint))[0]
        video_folder = os.path.join(os.path.dirname(os.path.abspath(args_cli.checkpoint)), f"videos_xyz_sequential_{checkpoint_name}")
        env = StreamingRecordVideo(
            env,
            video_folder=video_folder,
            step_trigger=lambda step: step == 0,
            video_length=args_cli.video_length,
            disable_logger=True,
        )
        print(f"PLAY_VIDEO_DIR {video_folder}", flush=True)

    base_env = env.unwrapped
    if args_cli.disable_rewards:
        for name in base_env.reward_manager.active_terms:
            base_env.reward_manager.get_term_cfg(name).weight = 0.0
    print(f"PLAY_REWARDS {'disabled' if args_cli.disable_rewards else 'enabled'}", flush=True)
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
        print("W/S: vx, Q/E: vy, A/D: yaw, I/K/J/L: push, [/]: push strength, Space: stop, R: reset pose/commands, X: quit", flush=True)
        print(f"COMMAND vx={command[0]:.2f} vy={command[1]:.2f} yaw={command[2]:.2f}", flush=True)
        print(f"PUSH_STRENGTH {pulse.fraction:.2f}*m*g", flush=True)
        if args_cli.task == "flat":
            print(gait_status(base_env), flush=True)
    sweep_trace = []
    with torch.inference_mode():
        for step in range(args_cli.steps):
            # 1,000 control steps at 50 Hz: each axis spans its full training
            # interval once, including both endpoints; other axes stay zero.
            if step < 334:
                vx, vy, yaw = -0.3 + 0.8 * step / 333, 0.0, 0.0
            elif step < 667:
                vx, vy, yaw = 0.0, -0.2 + 0.4 * (step - 334) / 332, 0.0
            else:
                vx, vy, yaw = 0.0, 0.0, -1.0 + 2.0 * (step - 667) / 332
            command_tensor[:] = torch.tensor((vx, vy, yaw), device=base_env.device)
            apply_command(base_env, command_tensor)
            sweep_trace.append({'step': step, 'time_s': step * base_env.step_dt, 'vx_m_s': vx, 'vy_m_s': vy, 'yaw_rad_s': yaw})
            if step % 100 == 0 or step in (333, 334, 666, 667, 999):
                print(f"SWEEP step={step} t={step * base_env.step_dt:.2f} vx={vx:.5f} vy={vy:.5f} yaw={yaw:.5f}", flush=True)
            if args_cli.keyboard:
                changed, should_quit, push, push_delta, should_reset = update_from_keyboard(command, args_cli.step_v, args_cli.step_yaw)
                if should_quit:
                    break
                if should_reset:
                    obs = reset_play_environment(env, pulse, command, command_tensor)
                    print("PLAY_MANUAL_RESET command=(0,0,0) push=cleared start_pose=" + start_pose_mode, flush=True)
                    if args_cli.task == "flat":
                        print(gait_status(base_env), flush=True)
                    continue
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
    if args_cli.video:
        with open(os.path.join(video_folder, 'command_trace.json'), 'w', encoding='utf-8') as trace_file:
            json.dump(sweep_trace, trace_file, indent=2)
    clear_manual_push(pulse)
    print("PLAY_DONE", flush=True)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
