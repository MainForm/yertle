"""Ideal linked-motor coordinate and effort transforms; no calibration offsets."""
import torch


class MotorSupplySag:
    """Post-rewire shared wiring and total-current sag, in physical motor ID order."""

    iterations = 64
    back_emf_v_per_rad_s = 0.56673325231
    current_gain_a_per_v = 0.14422106232
    common_drop_onset_a = 1.0
    common_drop_v_per_a = 0.03420856

    def __init__(self, *, device, dtype):
        # Per-motor median low-load voltage of the calibration runs.
        self.baseline_v = torch.tensor(
            [6.8, 6.7, 6.8, 6.75, 6.7, 6.7, 6.7, 6.8, 6.7, 6.8, 6.8, 6.8],
            device=device, dtype=dtype,
        )
        motor = torch.arange(12, device=device)
        same_leg = motor[:, None] // 3 == motor[None, :] // 3
        downstream = motor % 3 != 1  # shin and shoulder
        shoulder = motor % 3 == 2
        self.volt_per_amp = (
            0.10429653745608872
            + 0.09887391913 * same_leg.to(dtype)
            + 0.02402835834 * (same_leg & downstream[:, None] & downstream[None, :]).to(dtype)
            + 0.05000679221 * (shoulder[:, None] & (motor[:, None] == motor[None, :])).to(dtype)
        )

    def solve(self, pwm, motor_velocity):
        """Use applied PWM and speed to resolve supply voltage and signed drive voltage."""
        duty = pwm / 1000.0
        back_emf = self.back_emf_v_per_rad_s * motor_velocity
        voltage = self.baseline_v.expand_as(pwm)
        for _ in range(self.iterations):
            current = self.current_gain_a_per_v * (duty * voltage - back_emf).abs()
            excess = (current.sum(dim=-1, keepdim=True) - self.common_drop_onset_a).clamp_min(0)
            common_drop = self.common_drop_v_per_a * excess
            voltage = (self.baseline_v - current @ self.volt_per_amp.T - common_drop).clamp_min(0)
        return voltage, duty * voltage - back_emf


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
