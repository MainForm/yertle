"""Motor-coordinate penalties for the opt-in ideal linked actuator."""
import torch
from .motor_drive_math import joint_to_motor_effort, joint_to_motor_velocity


def _coordinates(env, asset_cfg):
    robot = env.scene[asset_cfg.name]
    names = list(robot.joint_names)
    thigh = [names.index(f'{leg}_thigh') for leg in ('lf', 'rf', 'lb', 'rb')]
    shin = [names.index(f'{leg}_shin') for leg in ('lf', 'rf', 'lb', 'rb')]
    return robot, thigh, shin


def motor_torques_l2(env, asset_cfg):
    robot, thigh, shin = _coordinates(env, asset_cfg)
    torque = joint_to_motor_effort(robot.data.applied_torque, thigh, shin)
    return torch.square(torque).sum(dim=1)


def motor_velocity_l2(env, asset_cfg):
    robot, thigh, shin = _coordinates(env, asset_cfg)
    velocity = joint_to_motor_velocity(robot.data.joint_vel, thigh, shin)
    return torch.square(velocity).sum(dim=1)


def motor_acc_l2(env, asset_cfg):
    robot, thigh, shin = _coordinates(env, asset_cfg)
    acceleration = joint_to_motor_velocity(robot.data.joint_acc, thigh, shin)
    return torch.square(acceleration).sum(dim=1)


def motor_velocity_excess(env, velocity_limit_rad_s, asset_cfg):
    if velocity_limit_rad_s <= 0:
        raise ValueError('velocity_limit_rad_s must be positive')
    robot, thigh, shin = _coordinates(env, asset_cfg)
    velocity = joint_to_motor_velocity(robot.data.joint_vel, thigh, shin)
    return torch.square((velocity.abs() / velocity_limit_rad_s - 1).clamp_min(0)).sum(dim=1)


def motor_energy_l1(env, asset_cfg):
    robot, thigh, shin = _coordinates(env, asset_cfg)
    torque = joint_to_motor_effort(robot.data.applied_torque, thigh, shin)
    velocity = joint_to_motor_velocity(robot.data.joint_vel, thigh, shin)
    return (torque * velocity).abs().sum(dim=1)


def configure_motor_space_rewards(cfg):
    """Preserve existing term names, weights, and thresholds in explicit mode."""
    from isaaclab.managers import SceneEntityCfg
    asset = SceneEntityCfg('robot')
    rewards = cfg.rewards
    rewards.dof_torques_l2.func = motor_torques_l2
    rewards.dof_torques_l2.params = {'asset_cfg': asset}
    rewards.dof_acc_l2.func = motor_acc_l2
    rewards.dof_acc_l2.params = {'asset_cfg': SceneEntityCfg('robot')}
    rewards.joint_vel_l2.func = motor_velocity_l2
    rewards.joint_vel_l2.params = {'asset_cfg': SceneEntityCfg('robot')}
    rewards.joint_velocity_excess.func = motor_velocity_excess
    rewards.joint_velocity_excess.params['asset_cfg'] = SceneEntityCfg('robot')
    rewards.energy.func = motor_energy_l1
    rewards.energy.params = {'asset_cfg': SceneEntityCfg('robot')}
