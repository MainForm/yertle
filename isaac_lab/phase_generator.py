"""Fixed 2.2 Hz diagonal trot phases for Yertle."""

import math

import torch

from isaaclab.managers import SceneEntityCfg

_FOOT_NAMES = ("lf_shin", "rf_shin", "lb_shin", "rb_shin")
# Keep the existing three-channel observation layout for checkpoint compatibility.
_MODE_NAMES = ("walk", "trot", "bound")
_TROT_PHASE_OFFSETS = (0.00, 0.50, 0.50, 0.00)  # LF+RB, then RF+LB
_TROT_FREQUENCY_HZ = 2.20
_TROT_DUTY_FACTOR = 0.50


def _command(env, command_name: str) -> torch.Tensor:
    return env.command_manager.get_command(command_name)


def _mode_ids(env, command_name: str) -> torch.Tensor:
    """Use trot for every velocity command, including zero and reverse motion."""
    command = _command(env, command_name)
    return torch.ones(env.num_envs, dtype=torch.long, device=command.device)


def _gait_state(env, command_name: str):
    mode = _mode_ids(env, command_name)
    device = mode.device
    frequencies = torch.full((env.num_envs,), _TROT_FREQUENCY_HZ, device=device)
    duties = torch.full((env.num_envs,), _TROT_DUTY_FACTOR, device=device)
    offsets = torch.tensor(_TROT_PHASE_OFFSETS, device=device, dtype=torch.float32).repeat(env.num_envs, 1)
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


def phase_unscheduled_contact_penalty(env, sensor_cfg: SceneEntityCfg, command_name: str = "base_velocity") -> torch.Tensor:
    """Fraction of feet contacting during their scheduled swing phase.

    This complements ``phase_contact_schedule``: standing still no longer earns
    phase reward merely because all feet remain in contact.
    """
    unscheduled = _actual_contact(env, sensor_cfg) & ~_desired_contact(env, command_name)
    return unscheduled.to(torch.float32).mean(dim=1) * _moving(env, command_name)


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
