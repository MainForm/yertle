"""Yertle-specific reward calculations.

This module deliberately contains no environment configuration classes.  The
single environment configuration in :mod:`flat_env_cfg` imports these
functions and registers them with Isaac Lab's reward manager.
"""

import torch

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab_tasks.manager_based.locomotion.velocity import mdp
from isaaclab_tasks.manager_based.locomotion.velocity.config.spot.mdp.rewards import GaitReward
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

from . import phase_generator

FOOT_NAMES = ("lf_shin", "rf_shin", "lb_shin", "rb_shin")
SHOULDER_NAMES = ("lf_shoulder", "rf_shoulder", "lb_shoulder", "rb_shoulder")
NON_FEET = ".*_(shoulder|thigh)"
LEFT_FEET = (0, 2)
RIGHT_FEET = (1, 3)
BASE = "base_link"
FEET = ".*_shin"
CONTACT_SENSOR = SceneEntityCfg("contact_forces", body_names=list(FOOT_NAMES))


def configure_rewards(cfg: LocomotionVelocityRoughEnvCfg) -> None:
    """Register the complete Yertle reward stack on an Isaac Lab config.

    The caller owns the environment lifecycle. This function only changes its
    reward manager configuration, including the original terms' weights and
    Yertle-specific additions.
    """
    rewards = cfg.rewards
    rewards.feet_air_time.params["sensor_cfg"].body_names = FEET
    rewards.feet_air_time.weight = 0.50
    rewards.feet_air_time.params["threshold"] = 0.50
    rewards.flat_orientation_l2.weight = -2.5
    rewards.dof_torques_l2.weight = -0.0002
    rewards.track_lin_vel_xy_exp.weight = 1.5
    rewards.track_ang_vel_z_exp.weight = 0.75
    rewards.action_rate_l2.weight = -0.04
    rewards.ang_vel_xy_l2.weight = -0.05
    rewards.dof_acc_l2.weight = -5.0e-7
    rewards.dof_pos_limits.weight = -0.25

    rewards.joint_vel_l2 = RewTerm(func=mdp.joint_vel_l2, weight=-0.001, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")})
    rewards.feet_slide = RewTerm(func=mdp.feet_slide, weight=-0.10, params={"asset_cfg": SceneEntityCfg("robot", body_names=FEET), "sensor_cfg": CONTACT_SENSOR})
    rewards.undesired_contacts = RewTerm(func=mdp.undesired_contacts, weight=-0.50, params={"threshold": 1.0, "sensor_cfg": SceneEntityCfg("contact_forces", body_names=NON_FEET)})
    rewards.air_time_variance_penalty = RewTerm(func=air_time_variance_penalty, weight=-0.25, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.wrong_side_foot_placement = RewTerm(func=wrong_side_foot_placement, weight=-0.25, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.shoulder_posture_barrier = RewTerm(func=shoulder_posture_barrier, weight=-0.05, params={"asset_cfg": SceneEntityCfg("robot", joint_names=SHOULDER_NAMES)})
    rewards.stance_width_barrier = RewTerm(func=stance_width_barrier, weight=-0.04, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.swing_foot_clearance_barrier = RewTerm(func=swing_foot_clearance_barrier, weight=-0.03, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.body_height_barrier = RewTerm(func=body_height_barrier, weight=-0.08, params={"asset_cfg": SceneEntityCfg("robot")})
    rewards.foot_workspace_barrier = RewTerm(func=foot_workspace_barrier, weight=-0.02, params={})
    rewards.energy = RewTerm(func=joint_energy_l1, weight=-2.0e-5, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")})
    rewards.joint_pos = RewTerm(func=mdp.stand_still_joint_deviation_l1, weight=-0.10, params={"command_name": "base_velocity", "command_threshold": 0.05, "asset_cfg": SceneEntityCfg("robot", joint_names=".*")})
    rewards.feet_contact_forces = RewTerm(func=mdp.contact_forces, weight=-0.01, params={"threshold": 5.0, "sensor_cfg": CONTACT_SENSOR})
    rewards.diagonal_trot_gait = RewTerm(func=GaitReward, weight=0.25, params={"std": 0.1, "max_err": 0.2, "velocity_threshold": 0.05, "synced_feet_pair_names": (("lf_shin", "rb_shin"), ("rf_shin", "lb_shin")), "asset_cfg": SceneEntityCfg("robot"), "sensor_cfg": SceneEntityCfg("contact_forces")})
    rewards.diagonal_trot_gait = None
    rewards.phase_contact_schedule = RewTerm(func=phase_generator.phase_contact_schedule, weight=0.20, params={"sensor_cfg": CONTACT_SENSOR, "command_name": "base_velocity"})
    rewards.phase_swing_clearance = RewTerm(func=phase_generator.phase_swing_clearance, weight=-0.06, params={"command_name": "base_velocity", "min_height_m": 0.045, "transition_m": 0.015})
    rewards.phase_touchdown_timing = RewTerm(func=phase_generator.phase_touchdown_timing, weight=0.10, params={"sensor_cfg": CONTACT_SENSOR, "command_name": "base_velocity"})


def air_time_variance_penalty(env, sensor_cfg: SceneEntityCfg, max_time: float = 0.5):
    """Penalize unequal completed contact and air intervals across four feet."""
    sensor = env.scene.sensors[sensor_cfg.name]
    if not sensor.cfg.track_air_time:
        raise RuntimeError("ContactSensor.track_air_time must be enabled")
    air = torch.clamp(sensor.data.last_air_time[:, sensor_cfg.body_ids], max=max_time)
    contact = torch.clamp(sensor.data.last_contact_time[:, sensor_cfg.body_ids], max=max_time)
    return torch.var(air, dim=1) + torch.var(contact, dim=1)


def _interval_barrier(value: torch.Tensor, lower: float, upper: float, transition: float):
    lower_violation = (lower - value).clamp_min(0.0) / transition
    upper_violation = (value - upper).clamp_min(0.0) / transition
    return lower_violation.square() + upper_violation.square()


def _foot_contact(env, sensor_cfg: SceneEntityCfg):
    sensor = env.scene.sensors[sensor_cfg.name]
    body_ids = [sensor.body_names.index(name) for name in FOOT_NAMES]
    force = torch.linalg.vector_norm(sensor.data.net_forces_w_history[:, :, body_ids, :], dim=-1).amax(dim=1)
    return force > 1.0


def _feet_in_base_xy(asset, foot_body_ids: list[int]):
    feet = asset.data.body_pos_w[:, foot_body_ids, :2]
    delta = feet - asset.data.root_pos_w[:, :2].unsqueeze(1)
    w, x, y, z = asset.data.root_quat_w.unbind(dim=-1)
    yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    local_x = torch.cos(yaw).unsqueeze(1) * delta[..., 0] + torch.sin(yaw).unsqueeze(1) * delta[..., 1]
    local_y = -torch.sin(yaw).unsqueeze(1) * delta[..., 0] + torch.cos(yaw).unsqueeze(1) * delta[..., 1]
    return torch.stack((local_x, local_y), dim=-1)


def joint_energy_l1(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """Mechanical joint-power magnitude, ``sum(abs(torque * velocity))``."""
    asset = env.scene[asset_cfg.name]
    return torch.sum(torch.abs(asset.data.applied_torque[:, asset_cfg.joint_ids] * asset.data.joint_vel[:, asset_cfg.joint_ids]), dim=1)


def wrong_side_foot_placement(env, sensor_cfg: SceneEntityCfg, contact_threshold: float = 1.0, violation_scale_m: float = 0.02):
    """Penalize a contacting left/right foot only after crossing the centerline."""
    asset = env.scene["robot"]
    sensor = env.scene.sensors[sensor_cfg.name]
    feet = _feet_in_base_xy(asset, [asset.data.body_names.index(name) for name in FOOT_NAMES])
    crossing = torch.stack(((-feet[:, 0, 1]).clamp_min(0.0), feet[:, 1, 1].clamp_min(0.0), (-feet[:, 2, 1]).clamp_min(0.0), feet[:, 3, 1].clamp_min(0.0)), dim=1)
    ids = [sensor.body_names.index(name) for name in FOOT_NAMES]
    force = torch.linalg.vector_norm(sensor.data.net_forces_w_history[:, :, ids, :], dim=-1).amax(dim=1)
    return torch.sum((crossing / violation_scale_m) * (force > contact_threshold), dim=1)


def shoulder_posture_barrier(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), lower: float = -0.35, upper: float = 0.35, transition: float = 0.12):
    asset = env.scene[asset_cfg.name]
    return torch.sum(_interval_barrier(asset.data.joint_pos[:, asset_cfg.joint_ids], lower, upper, transition), dim=1)


def stance_width_barrier(env, sensor_cfg: SceneEntityCfg, lower_m: float = 0.12, upper_m: float = 0.22, transition_m: float = 0.025):
    asset = env.scene["robot"]
    feet = _feet_in_base_xy(asset, [asset.data.body_names.index(name) for name in FOOT_NAMES])
    contact = _foot_contact(env, sensor_cfg)
    left, right = contact[:, LEFT_FEET], contact[:, RIGHT_FEET]
    valid = left.any(dim=1) & right.any(dim=1)
    left_y = (feet[:, LEFT_FEET, 1] * left).sum(dim=1) / left.sum(dim=1).clamp_min(1)
    right_y = (feet[:, RIGHT_FEET, 1] * right).sum(dim=1) / right.sum(dim=1).clamp_min(1)
    return _interval_barrier(torch.abs(left_y - right_y), lower_m, upper_m, transition_m) * valid


def swing_foot_clearance_barrier(env, sensor_cfg: SceneEntityCfg, min_height_m: float = 0.035, transition_m: float = 0.015):
    asset = env.scene["robot"]
    ids = [asset.data.body_names.index(name) for name in FOOT_NAMES]
    shortfall = ((min_height_m - asset.data.body_pos_w[:, ids, 2]).clamp_min(0.0) / transition_m).square()
    return torch.sum(shortfall * ~_foot_contact(env, sensor_cfg), dim=1)


def body_height_barrier(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), lower_m: float = 0.20, upper_m: float = 0.30, transition_m: float = 0.02):
    return _interval_barrier(env.scene[asset_cfg.name].data.root_pos_w[:, 2], lower_m, upper_m, transition_m)


def foot_workspace_barrier(env, lower_x_m: float = -0.20, upper_x_m: float = 0.20, lower_abs_y_m: float = 0.03, upper_abs_y_m: float = 0.20, transition_m: float = 0.03):
    asset = env.scene["robot"]
    feet = _feet_in_base_xy(asset, [asset.data.body_names.index(name) for name in FOOT_NAMES])
    x_cost = _interval_barrier(feet[..., 0], lower_x_m, upper_x_m, transition_m)
    y_cost = _interval_barrier(feet[..., 1].abs(), lower_abs_y_m, upper_abs_y_m, transition_m)
    return torch.sum(x_cost + y_cost, dim=1)
