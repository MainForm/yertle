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


def motor_back_emf_effort(pwm, velocity, *, nm_per_pwm, free_speed, torque_limit, speed_limit):
    """Quasi-static DC motor: signed PWM voltage minus speed back-EMF.

    The 4.6 rad/s free speed is at 1000 PWM; 800 PWM at standstill gives
    1.024 Nm with nm_per_pwm=1.024/800. Neutral PWM assumes dynamic braking.
    Electrical current/inductance, supply sag and driver mode are not modeled.
    """
    if nm_per_pwm <= 0 or free_speed <= 0 or torque_limit <= 0 or speed_limit <= 0:
        raise ValueError('Motor model parameters must be positive')
    raw_effort = nm_per_pwm * (pwm - 1000.0 * velocity / free_speed)
    return limit_motor_effort(raw_effort, velocity, torque_limit, speed_limit)
