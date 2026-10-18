"""Yertle-specific reward calculations.

This module deliberately contains no environment configuration classes.  The
single environment configuration in :mod:`flat_env_cfg` imports these
functions and registers them with Isaac Lab's reward manager.
"""

import torch

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import ManagerTermBase
from isaaclab_tasks.manager_based.locomotion.velocity import mdp
from isaaclab_tasks.manager_based.locomotion.velocity.config.spot.mdp.rewards import GaitReward
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

from . import phase_generator
from .foot_geometry import foot_sole_contact_velocities
from .ground_contact import GroundContactPenalty
from .motor_operating_range import MotorRawTransform, OperatingRangeCost
from .motor_travel_balance import MotorTravelBalancePenalty

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
    # RewardManager multiplies by control dt: -250 * 0.02 = -5 per fall.
    # Read the existing termination result; do not broaden the failure condition.
    rewards.fall_termination_penalty = RewTerm(
        func=fall_termination_penalty, weight=-250.0,
        params={"term_name": "body_tilt_and_nonsole_contact"},
    )
    rewards.feet_air_time.params["sensor_cfg"].body_names = FEET
    rewards.feet_air_time.weight = 1.00
    # A 2.2 Hz / 50% duty trot has a ~0.227 s swing. The inherited 0.5 s
    # threshold penalized its normal touchdowns instead of rewarding them.
    rewards.feet_air_time.func = feet_air_time_trot
    rewards.feet_air_time.params["threshold"] = 0.15
    rewards.flat_orientation_l2.weight = -3.0
    rewards.lin_vel_z_l2.weight = -0.5
    rewards.dof_torques_l2.weight = -0.0002
    # Slow commands demand a tight match; the acceptable error grows with the
    # magnitude of each commanded velocity component.
    rewards.track_lin_vel_xy_exp = None
    rewards.track_ang_vel_z_exp = None
    rewards.track_lin_vel_xy_cauchy = RewTerm(
        func=track_lin_vel_xy_cauchy,
        weight=2.0,
        params={"command_name": "base_velocity", "min_scale": 0.05, "relative_scale": 0.50,
                "components": (0, 1)},
    )
    rewards.track_ang_vel_z_cauchy = RewTerm(
        func=track_ang_vel_z_cauchy,
        weight=0.75,
        params={"command_name": "base_velocity", "min_scale": 0.10, "relative_scale": 0.50},
    )
    rewards.action_rate_l2.weight = -0.04
    rewards.action_second_difference_l2 = RewTerm(
        func=ActionSecondDifferencePenalty, weight=-0.05, params={"action_name": "joint_pos"})
    rewards.ang_vel_xy_l2.weight = -0.05
    rewards.dof_acc_l2.weight = -5.0e-7
    # Independent per-joint soft intervals cannot describe the measured linkage.
    rewards.dof_pos_limits = None

    rewards.motor_operating_range_penalty = RewTerm(
        func=MotorOperatingRangePenalty,
        weight=-100.0,
        params={"action_name": "joint_pos", "asset_cfg": SceneEntityCfg("robot"),
                "command_speed_steps_per_s": 200, "excess_scale_raw": 100.0},
    )

    rewards.joint_vel_l2 = RewTerm(func=mdp.joint_vel_l2, weight=-0.001, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")})
    rewards.joint_velocity_excess = RewTerm(
        func=joint_velocity_excess,
        weight=-0.10,
        params={
            "velocity_limit_rad_s": 2.30,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
        },
    )
    rewards.feet_slide = RewTerm(func=feet_slide_sole, weight=-0.10,
                               params={"sensor_cfg": CONTACT_SENSOR, "surface_band_m": 0.001})
    rewards.undesired_contacts = RewTerm(func=GroundContactPenalty, weight=-0.50,
        params={"body_names": [f"{leg}_{joint}" for leg in ("lf","rf","lb","rb") for joint in ("shoulder","thigh")],
                "separation_tolerance_m": 0.001, "sole_tolerance_m": 0.005})
    rewards.base_contact_penalty = RewTerm(
        func=GroundContactPenalty, weight=-1.0,
        params={"body_names": [BASE], "separation_tolerance_m": 0.001, "sole_tolerance_m": 0.005},
    )
    rewards.nonsole_foot_contact_penalty = RewTerm(func=GroundContactPenalty, weight=-0.50,
        params={"body_names": list(FOOT_NAMES), "separation_tolerance_m": 0.001, "sole_tolerance_m": 0.005})
    rewards.air_time_variance_penalty = RewTerm(func=air_time_variance_penalty, weight=-0.25, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.wrong_side_foot_placement = RewTerm(func=wrong_side_foot_placement, weight=-0.25, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.shoulder_posture_barrier = RewTerm(func=shoulder_posture_barrier, weight=-0.20, params={"asset_cfg": SceneEntityCfg("robot", joint_names=SHOULDER_NAMES), "lower": -0.17453292519943295, "upper": 0.17453292519943295})
    rewards.stance_width_barrier = RewTerm(func=stance_width_barrier, weight=-0.04, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.swing_foot_clearance_barrier = RewTerm(func=swing_foot_clearance_barrier, weight=-0.03, params={"sensor_cfg": CONTACT_SENSOR})
    rewards.foot_workspace_barrier = RewTerm(func=foot_workspace_barrier, weight=-0.02, params={})
    rewards.energy = RewTerm(func=joint_energy_l1, weight=-2.0e-5, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")})
    rewards.joint_pos = RewTerm(func=mdp.stand_still_joint_deviation_l1, weight=-0.10, params={"command_name": "base_velocity", "command_threshold": 0.05, "asset_cfg": SceneEntityCfg("robot", joint_names=".*")})
    rewards.feet_contact_forces = RewTerm(func=mdp.contact_forces, weight=-0.01, params={"threshold": 5.0, "sensor_cfg": CONTACT_SENSOR})
    rewards.diagonal_trot_gait = RewTerm(func=GaitReward, weight=0.25, params={"std": 0.1, "max_err": 0.2, "velocity_threshold": 0.05, "synced_feet_pair_names": (("lf_shin", "rb_shin"), ("rf_shin", "lb_shin")), "asset_cfg": SceneEntityCfg("robot"), "sensor_cfg": SceneEntityCfg("contact_forces")})
    rewards.diagonal_trot_gait = None
    rewards.phase_contact_schedule = RewTerm(func=phase_generator.phase_contact_schedule, weight=0.20, params={"sensor_cfg": CONTACT_SENSOR, "command_name": "base_velocity"})
    rewards.phase_unscheduled_contact_penalty = RewTerm(
        func=phase_generator.phase_unscheduled_contact_penalty,
        weight=-0.40,
        params={"sensor_cfg": CONTACT_SENSOR, "command_name": "base_velocity"},
    )
    rewards.phase_swing_clearance = RewTerm(func=phase_generator.phase_swing_clearance, weight=-1.0, params={"command_name": "base_velocity", "min_height_m": 0.045, "transition_m": 0.015})
    rewards.phase_touchdown_timing = RewTerm(func=phase_generator.phase_touchdown_timing, weight=0.10, params={"sensor_cfg": CONTACT_SENSOR, "command_name": "base_velocity"})


def configure_motor_travel_balance(cfg):
    """Opt-in v6.21, one configured gait cycle of measured servo travel."""
    frequency = getattr(cfg, 'trot_frequency_hz', 2.2)
    cfg.rewards.motor_travel_balance_penalty = RewTerm(
        func=MotorTravelBalancePenalty, weight=-0.5,
        params={'asset_cfg': SceneEntityCfg('robot'), 'command_name': 'base_velocity',
                'window_s': 1.0 / frequency, 'travel_floor_rad': 0.08726646259971647})


def configure_trot_support_rewards(cfg):
    """Opt-in v6.20.3 support constraints; preserve v6.20.2 configs/digests.

    Negative weights are continuous costs, multiplied by control dt by Isaac
    Lab. They guide behavior but do not hard-lock joint actions or contacts.
    """
    cfg.rewards.phase_stance_missing_contact_penalty = RewTerm(
        func=phase_generator.phase_stance_missing_contact_penalty, weight=-1.0,
        params={'sensor_cfg': CONTACT_SENSOR, 'command_name': 'base_velocity',
                'contact_threshold_n': 1.0, 'transition_grace_s': 0.02})
    cfg.rewards.all_feet_airborne_penalty = RewTerm(
        func=phase_generator.all_feet_airborne_penalty, weight=-2.0,
        params={'sensor_cfg': CONTACT_SENSOR, 'contact_threshold_n': 0.1})
    # Permit brief double support while diagonal pairs exchange roles. The
    # default v6.20.2 terms keep their original zero-grace behavior.
    cfg.rewards.phase_unscheduled_contact_penalty.params['transition_grace_s'] = 0.02
    cfg.rewards.phase_touchdown_timing.params['transition_grace_s'] = 0.02


def fall_termination_penalty(env, term_name: str = "body_tilt_and_nonsole_contact"):
    """One event on the named fall termination; timeout alone is not a fall.

    A configuration without that optional termination produces zero. Read the
    manager's already computed flag before auto-reset, avoiding another contact
    query or changes to the termination decision. Coincident timeout and fall
    still count as a fall.
    """
    manager = env.termination_manager
    if term_name not in manager.active_terms:
        return torch.zeros(env.num_envs, device=env.device)
    return manager.get_term(term_name).float()


class ActionSecondDifferencePenalty(ManagerTermBase):
    """Sum squared second differences of dimensionless joint-position actions.

    Suppress the first two samples after each environment reset. Maintain
    histories per environment and compute once per control step, including
    diagnostic reward reevaluations. This is not measured joint acceleration
    and deliberately has no dt^-4 scaling or command/phase gate.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.action = env.action_manager.get_term(cfg.params["action_name"])
        actions = self.action.raw_actions
        self.previous = torch.zeros_like(actions)
        self.previous_previous = torch.zeros_like(actions)
        self.samples = torch.zeros(actions.shape[0], dtype=torch.long, device=actions.device)
        self.value = torch.zeros(actions.shape[0], dtype=actions.dtype, device=actions.device)
        self.last_step = None

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.previous[ids] = 0
        self.previous_previous[ids] = 0
        self.samples[ids] = 0
        self.value[ids] = 0

    def __call__(self, env, action_name: str):
        if self.last_step == env.common_step_counter:
            return self.value
        actions = self.action.raw_actions
        difference = actions - 2*self.previous + self.previous_previous
        self.value.copy_(difference.square().sum(dim=1) * (self.samples >= 2))
        self.previous_previous.copy_(self.previous)
        self.previous.copy_(actions)
        self.samples.add_(1).clamp_(max=2)
        self.last_step = env.common_step_counter
        return self.value


class MotorOperatingRangePenalty(ManagerTermBase):
    """CSV hardware limits on both requested targets and measured positions.

    CSV parsing, name resolution and device allocation happen once per env.
    Action names are read from Isaac Lab's resolved JointPositionAction order,
    not assumed to match either asset order or physical motor ID order.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        asset = env.scene[cfg.params["asset_cfg"].name]
        action = env.action_manager.get_term(cfg.params["action_name"])
        self.action = action
        self.target_transform = MotorRawTransform(
            action._joint_names, device=env.device, dtype=asset.data.joint_pos.dtype)
        self.measured_transform = MotorRawTransform(
            asset.joint_names, device=env.device, dtype=asset.data.joint_pos.dtype)
        self.cost = OperatingRangeCost(
            command_speed_steps_per_s=cfg.params["command_speed_steps_per_s"],
            excess_scale_raw=cfg.params["excess_scale_raw"],
            device=env.device, dtype=asset.data.joint_pos.dtype)

    def __call__(self, env, action_name: str, asset_cfg: SceneEntityCfg,
                 command_speed_steps_per_s: int, excess_scale_raw: float):
        return self.cost(self.target_transform(self.action.processed_actions),
                         self.measured_transform(env.scene[asset_cfg.name].data.joint_pos))


def joint_velocity_excess(
    env,
    velocity_limit_rad_s: float = 2.30,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Sum squared normalized speed excess; the reward weight must be negative.

    Uses measured joint velocity [rad/s], not action or target-position changes.
    Speeds at or below the threshold have zero cost. This is a soft training
    penalty, not a replacement for the actuator's physics-solver speed limit.
    """
    if velocity_limit_rad_s <= 0.0:
        raise ValueError("velocity_limit_rad_s must be positive")
    velocity = env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids]
    excess = (velocity.abs() / velocity_limit_rad_s - 1.0).clamp_min(0.0)
    return excess.square().sum(dim=1)


def feet_slide_sole(env, sensor_cfg: SceneEntityCfg, surface_band_m: float = 0.001):
    """Sum planar sole-point speed on contacting feet; keep the old contact gate.

    Geometry velocity is in FOOT_NAMES order; resolved sensor IDs may use a
    different articulation order. Match by name, not by coincidental indices.
    No command/phase gate: an actual contacting foot should not slide at rest.
    """
    sensor = env.scene.sensors[sensor_cfg.name]
    contacts = sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).amax(dim=1) > 1.0
    order = [FOOT_NAMES.index(sensor.body_names[i]) for i in sensor_cfg.body_ids]
    velocity = foot_sole_contact_velocities(env, surface_band_m)[:, order, :2]
    return torch.sum(torch.linalg.vector_norm(velocity, dim=-1) * contacts, dim=1)


def air_time_variance_penalty(env, sensor_cfg: SceneEntityCfg, max_time: float = 0.5):
    """Penalize unequal completed contact and air intervals across four feet."""
    sensor = env.scene.sensors[sensor_cfg.name]
    if not sensor.cfg.track_air_time:
        raise RuntimeError("ContactSensor.track_air_time must be enabled")
    air = torch.clamp(sensor.data.last_air_time[:, sensor_cfg.body_ids], max=max_time)
    contact = torch.clamp(sensor.data.last_contact_time[:, sensor_cfg.body_ids], max=max_time)
    return torch.var(air, dim=1) + torch.var(contact, dim=1)


def feet_air_time_trot(env, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float):
    """Touchdown air-time reward, gated by the same commands as phase rewards.

    Keep the inherited signed (air time - threshold) score and touchdown-only
    evaluation, but use the shared >0.05 m/s OR >0.10 rad/s moving gate. This
    gate is based on the command, not measured movement; zero commands remain
    unpenalized and pure commanded turns can receive a gait reward too.
    """
    sensor = env.scene.sensors[sensor_cfg.name]
    first_contact = sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = sensor.data.last_air_time[:, sensor_cfg.body_ids]
    score = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    return score * phase_generator._moving(env, command_name)


def _command_relative_cauchy(error: torch.Tensor, command: torch.Tensor, min_scale: float, relative_scale: float):
    """Return a Cauchy score with a command-relative half-width.

    A zero/slow command uses ``min_scale``.  Each component widens only in
    proportion to the magnitude commanded for that component, so large vx,
    vy, or yaw commands are not judged by an unrealistically tight absolute
    error tolerance.
    """
    scale = min_scale + relative_scale * command.abs()
    return 1.0 / (1.0 + (error / scale).square())


def track_lin_vel_xy_cauchy(
    env,
    command_name: str = "base_velocity",
    min_scale: float = 0.05,
    relative_scale: float = 0.50,
    components: tuple[int, ...] = (0, 1),
):
    """Mean Cauchy score for selected linear components; (0,) is vx-only.

    Full-command runs keep (0, 1). Explicit vx-only training uses (0,), so
    matching the unused vy=0 command cannot inflate the forward tracking score.
    The selected-axis score remains in [0, 1]; weights/scales are unchanged.
    """
    if not components or len(set(components)) != len(components) or any(i not in (0, 1) for i in components):
        raise ValueError("components must select unique vx/vy indices: (0,), (1,), or (0, 1)")
    asset = env.scene["robot"]
    command = env.command_manager.get_command(command_name)[:, list(components)]
    error = asset.data.root_lin_vel_b[:, list(components)] - command
    return _command_relative_cauchy(error, command, min_scale, relative_scale).mean(dim=1)


def track_ang_vel_z_cauchy(
    env,
    command_name: str = "base_velocity",
    min_scale: float = 0.10,
    relative_scale: float = 0.50,
):
    """Cauchy tracking score for yaw-rate command and body yaw velocity."""
    asset = env.scene["robot"]
    command = env.command_manager.get_command(command_name)[:, 2]
    error = asset.data.root_ang_vel_b[:, 2] - command
    return _command_relative_cauchy(error, command, min_scale, relative_scale)


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


def shoulder_posture_barrier(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), lower: float = -0.17453292519943295, upper: float = 0.17453292519943295, transition: float = 0.12):
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
    height = phase_generator._foot_sole_heights(env)
    shortfall = ((min_height_m - height).clamp_min(0.0) / transition_m).square()
    return torch.sum(shortfall * ~_foot_contact(env, sensor_cfg), dim=1)


def foot_workspace_barrier(env, lower_x_m: float = -0.20, upper_x_m: float = 0.20, lower_abs_y_m: float = 0.03, upper_abs_y_m: float = 0.20, transition_m: float = 0.03):
    asset = env.scene["robot"]
    feet = _feet_in_base_xy(asset, [asset.data.body_names.index(name) for name in FOOT_NAMES])
    x_cost = _interval_barrier(feet[..., 0], lower_x_m, upper_x_m, transition_m)
    y_cost = _interval_barrier(feet[..., 1].abs(), lower_abs_y_m, upper_abs_y_m, transition_m)
    return torch.sum(x_cost + y_cost, dim=1)
