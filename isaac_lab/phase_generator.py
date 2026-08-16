"""Command-conditioned walk, trot, and direction-aware bound phases for Yertle."""

import math

import torch

from isaaclab.managers import SceneEntityCfg

_FOOT_NAMES = ("lf_shin", "rf_shin", "lb_shin", "rb_shin")
_MODE_NAMES = ("walk", "trot", "bound")
_PHASE_OFFSETS = (
    (0.00, 0.50, 0.75, 0.25),  # walk: LF -> RB -> RF -> LB
    (0.00, 0.50, 0.50, 0.00),  # trot: LF+RB, then RF+LB
    (0.00, 0.00, 0.50, 0.50),  # bound: front pair, then rear pair
)
_FORWARD_BOUND_OFFSETS = (0.00, 0.00, 0.50, 0.50)
_REVERSE_BOUND_OFFSETS = (0.50, 0.50, 0.00, 0.00)
_FREQUENCIES_HZ = (1.50, 2.20, 2.80)
_DUTY_FACTORS = (0.65, 0.50, 0.42)
def _command(env, command_name: str) -> torch.Tensor:
    return env.command_manager.get_command(command_name)


def _mode_ids(env, command_name: str) -> torch.Tensor:
    """Choose bound for nearly straight high-speed motion in either X direction."""
    command = _command(env, command_name)
    planar_speed = torch.linalg.vector_norm(command[:, :2], dim=1)
    yaw_rate = command[:, 2].abs()
    mode = torch.ones(env.num_envs, dtype=torch.long, device=command.device)
    mode[planar_speed < 0.12] = 0
    bound = (command[:, 0].abs() > 0.24) & (command[:, 1].abs() < 0.06) & (yaw_rate < 0.25)
    mode[bound] = 2
    return mode


def _gait_state(env, command_name: str):
    command = _command(env, command_name)
    mode = _mode_ids(env, command_name)
    device = mode.device
    frequencies = torch.tensor(_FREQUENCIES_HZ, device=device, dtype=torch.float32)[mode]
    duties = torch.tensor(_DUTY_FACTORS, device=device, dtype=torch.float32)[mode]
    offsets = torch.tensor(_PHASE_OFFSETS, device=device, dtype=torch.float32)[mode]
    forward_bound = (mode == 2) & (command[:, 0] > 0.0)
    reverse_bound = (mode == 2) & (command[:, 0] < 0.0)
    offsets[forward_bound] = torch.tensor(_FORWARD_BOUND_OFFSETS, device=device)
    offsets[reverse_bound] = torch.tensor(_REVERSE_BOUND_OFFSETS, device=device)
    phase = torch.remainder(env.episode_length_buf.to(torch.float32) * env.step_dt * frequencies, 1.0)
    return phase, mode, frequencies, duties, offsets


def _moving(env, command_name: str) -> torch.Tensor:
    command = _command(env, command_name)
    return (torch.linalg.vector_norm(command[:, :2], dim=1) > 0.05) | (command[:, 2].abs() > 0.10)


def phase_clock_sin_cos(env, command_name: str = "base_velocity") -> torch.Tensor:
    phase, _, _, _, _ = _gait_state(env, command_name)
    angle = 2.0 * math.pi * phase
    return torch.stack((torch.sin(angle), torch.cos(angle)), dim=-1)


def gait_mode_one_hot(env, command_name: str = "base_velocity") -> torch.Tensor:
    _, mode, _, _, _ = _gait_state(env, command_name)
    return torch.nn.functional.one_hot(mode, num_classes=len(_MODE_NAMES)).to(torch.float32)


def gait_frequency(env, command_name: str = "base_velocity") -> torch.Tensor:
    _, _, frequency, _, _ = _gait_state(env, command_name)
    return frequency.unsqueeze(-1)


def gait_duty_factor(env, command_name: str = "base_velocity") -> torch.Tensor:
    _, _, _, duty, _ = _gait_state(env, command_name)
    return duty.unsqueeze(-1)


def _actual_contact(env, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    sensor = env.scene.sensors[sensor_cfg.name]
    body_ids = [sensor.body_names.index(name) for name in _FOOT_NAMES]
    force = torch.linalg.vector_norm(sensor.data.net_forces_w_history[:, :, body_ids, :], dim=-1).amax(dim=1)
    return force > 1.0


def _desired_contact(env, command_name: str) -> torch.Tensor:
    phase, _, _, duty, offsets = _gait_state(env, command_name)
    return torch.remainder(phase.unsqueeze(-1) + offsets, 1.0) < duty.unsqueeze(-1)


def phase_contact_schedule(env, sensor_cfg: SceneEntityCfg, command_name: str = "base_velocity") -> torch.Tensor:
    return (_actual_contact(env, sensor_cfg) == _desired_contact(env, command_name)).to(torch.float32).mean(dim=1) * _moving(env, command_name)


def phase_swing_clearance(env, command_name: str = "base_velocity", min_height_m: float = 0.045, transition_m: float = 0.015) -> torch.Tensor:
    asset = env.scene["robot"]
    body_ids = [asset.data.body_names.index(name) for name in _FOOT_NAMES]
    height = asset.data.body_pos_w[:, body_ids, 2]
    shortfall = ((min_height_m - height).clamp_min(0.0) / transition_m).square()
    return (shortfall * ~_desired_contact(env, command_name)).mean(dim=1) * _moving(env, command_name)


def phase_touchdown_timing(env, sensor_cfg: SceneEntityCfg, command_name: str = "base_velocity") -> torch.Tensor:
    sensor = env.scene.sensors[sensor_cfg.name]
    body_ids = [sensor.body_names.index(name) for name in _FOOT_NAMES]
    touchdown = sensor.compute_first_contact(env.step_dt)[:, body_ids]
    score = torch.where(_desired_contact(env, command_name), touchdown.to(torch.float32), -touchdown.to(torch.float32))
    return score.mean(dim=1) * _moving(env, command_name)
