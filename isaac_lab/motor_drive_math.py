"""Ideal linked-motor coordinate and effort transforms; no calibration offsets."""
import torch


def joint_to_motor_velocity(velocity, thigh, shin):
    result = velocity.clone()
    result[..., shin] = velocity[..., thigh] + velocity[..., shin]
    return result


def joint_to_motor_effort(effort, thigh, shin):
    result = effort.clone()
    result[..., thigh] = effort[..., thigh] - effort[..., shin]
    return result


def motor_to_joint_effort(effort, thigh, shin):
    result = effort.clone()
    result[..., thigh] = effort[..., thigh] + effort[..., shin]
    return result


def limit_motor_effort(effort, velocity, torque_limit, speed_limit):
    """Torque cap and outward drive cutoff; external loads can exceed speed limit."""
    limited = torch.clamp(effort, -torque_limit, torque_limit)
    outward = ((velocity >= speed_limit) & (limited > 0)) | ((velocity <= -speed_limit) & (limited < 0))
    return torch.where(outward, torch.zeros_like(limited), limited)
