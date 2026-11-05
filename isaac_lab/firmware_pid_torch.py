"""Vectorized F103 position PID arithmetic in encoder-step and PWM units."""
import torch


def firmware_pid_step(goal, measured, previous, integral, initialized, dt,
                      kp=7.0, ki=3.0, kd=0.13, integral_limit=100.0, duty_limit=1000.0):
    """Return pre-polarity integer PWM, new integral and measured position.

    Inputs are validated and quantized by the caller. The F103 controller uses
    measured-position finite differences, conditional integration and lroundf.
    """
    previous = torch.where(initialized, previous, measured)
    error = goal - measured
    velocity = (measured - previous) / dt
    proposed = torch.clamp(integral + ki * error * dt, -integral_limit, integral_limit)
    pd = kp * error - kd * velocity
    unsaturated = pd + proposed
    blocks = ((unsaturated > duty_limit) & (error > 0)) | ((unsaturated < -duty_limit) & (error < 0))
    updated = torch.where(blocks, integral, proposed)
    requested = torch.clamp(pd + updated, -duty_limit, duty_limit)
    # C lroundf rounds half away from zero, unlike torch.round's ties-to-even.
    pwm = torch.sign(requested) * torch.floor(torch.abs(requested) + 0.5)
    return pwm, updated, measured
