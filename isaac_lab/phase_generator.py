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


def phase_unscheduled_contact_penalty(env, sensor_cfg: SceneEntityCfg, command_name: str = "base_velocity", transition_grace_s: float = 0.0) -> torch.Tensor:
    """Fraction of feet contacting during their scheduled swing phase.

    This complements ``phase_contact_schedule``: standing still no longer earns
    phase reward merely because all feet remain in contact.
    """
    unscheduled = _actual_contact(env, sensor_cfg) & ~_desired_contact(env, command_name)
    score = unscheduled.to(torch.float32).mean(dim=1) * _moving(env, command_name)
    if transition_grace_s:
        score = score * ~_trot_transition(env, command_name, transition_grace_s)
    return score


def _trot_transition(env, command_name: str, grace_s: float) -> torch.Tensor:
    """Symmetric grace around each diagonal-pair swap; does not change the clock."""
    if not math.isfinite(grace_s) or not 0 <= grace_s < 0.25 / _TROT_FREQUENCY_HZ:
        raise ValueError('Transition grace must be finite and below a quarter trot cycle')
    phase, _, frequency, _, _ = _gait_state(env, command_name)
    if grace_s == 0:
        return torch.zeros_like(phase, dtype=torch.bool)
    half_phase = torch.remainder(phase, 0.5)
    seconds_to_swap = torch.minimum(half_phase, 0.5 - half_phase) / frequency
    return seconds_to_swap <= grace_s


def _instant_foot_contact(env, sensor_cfg: SceneEntityCfg, contact_threshold_n: float) -> torch.Tensor:
    """Current shin-link net force in LF/RF/LB/RB order, not history maximum.

    Existing nonsole-contact terms still reject forbidden parts of a foot link.
    Net force alone does not identify the rubber contact patch.
    """
    if not math.isfinite(contact_threshold_n) or contact_threshold_n < 0:
        raise ValueError('Contact threshold must be finite and nonnegative')
    sensor = env.scene.sensors[sensor_cfg.name]
    ids = [sensor.body_names.index(name) for name in _FOOT_NAMES]
    return torch.linalg.vector_norm(sensor.data.net_forces_w[:, ids, :], dim=-1) > contact_threshold_n


def phase_stance_missing_contact_penalty(env, sensor_cfg: SceneEntityCfg, command_name: str = 'base_velocity', contact_threshold_n: float = 1.0, transition_grace_s: float = 0.02) -> torch.Tensor:
    """Fraction of scheduled support feet missing current contact (0, 1/2, 1).

    Only commanded motion requires phase-specific diagonal support. Four-foot
    contact has zero cost here; existing swing-contact cost discourages standing.
    """
    desired = _desired_contact(env, command_name)
    missing = desired & ~_instant_foot_contact(env, sensor_cfg, contact_threshold_n)
    fraction = missing.float().sum(-1) / desired.sum(-1).clamp_min(1)
    return fraction * _moving(env, command_name) * ~_trot_transition(env, command_name, transition_grace_s)


def all_feet_airborne_penalty(env, sensor_cfg: SceneEntityCfg, contact_threshold_n: float = 0.1) -> torch.Tensor:
    """One while all four foot links lack contact, including zero commands.

    No phase grace: complete loss of foot support is always penalized. This is
    contact-based, unlike the stricter height+force diagnostic flight criterion.
    """
    return (~_instant_foot_contact(env, sensor_cfg, contact_threshold_n).any(-1)).float()


def _foot_sole_heights(env):
    from .foot_geometry import foot_sole_heights
    return foot_sole_heights(env)


def phase_swing_clearance(env, command_name: str = "base_velocity", min_height_m: float = 0.045, transition_m: float = 0.015) -> torch.Tensor:
    height = _foot_sole_heights(env)
    shortfall = ((min_height_m - height).clamp_min(0.0) / transition_m).square()
    return (shortfall * ~_desired_contact(env, command_name)).mean(dim=1) * _moving(env, command_name)


def phase_touchdown_timing(env, sensor_cfg: SceneEntityCfg, command_name: str = "base_velocity", transition_grace_s: float = 0.0) -> torch.Tensor:
    sensor = env.scene.sensors[sensor_cfg.name]
    body_ids = [sensor.body_names.index(name) for name in _FOOT_NAMES]
    touchdown = sensor.compute_first_contact(env.step_dt)[:, body_ids]
    score = torch.where(_desired_contact(env, command_name), touchdown.to(torch.float32), -touchdown.to(torch.float32))
    if transition_grace_s:
        score = torch.where(_trot_transition(env, command_name, transition_grace_s).unsqueeze(-1), score.clamp_min(0.0), score)
    return score.mean(dim=1) * _moving(env, command_name)
