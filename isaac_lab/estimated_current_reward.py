"""Provisional torque-based aggregate current proxy for flat Yertle.

The implicit actuator does not simulate electrical current.  This proxy is not
a battery-current model; its coefficient must be selected explicitly and later
calibrated.  Thigh/shin conversion assumes the ideal kinematic relation
motor_shin = joint_thigh + joint_shin, with no transmission loss.
"""
import math
import torch
from isaaclab.managers import SceneEntityCfg

LEGS = ('lf', 'rf', 'lb', 'rb')


def estimated_leg_current_sums(env, amp_per_nm: float, asset_cfg=SceneEntityCfg('robot')):
    """Return torque-derived current proxy [A] per leg in LEGS order."""
    if not math.isfinite(amp_per_nm) or amp_per_nm <= 0:
        raise ValueError('amp_per_nm must be finite and positive')
    robot = env.scene[asset_cfg.name]
    names = list(robot.joint_names)
    torque = robot.data.applied_torque
    if torque.shape[-1] != len(names):
        raise ValueError('torque layout must match joint_names')
    indices = {f'{leg}_{kind}': names.index(f'{leg}_{kind}')
               for leg in LEGS for kind in ('shoulder', 'thigh', 'shin')}
    legs = []
    for leg in LEGS:
        shoulder = torque[:, indices[f'{leg}_shoulder']]
        thigh = torque[:, indices[f'{leg}_thigh']]
        shin = torque[:, indices[f'{leg}_shin']]
        legs.append(shoulder.abs() + (thigh - shin).abs() + shin.abs())
    return torch.stack(legs, dim=-1) * amp_per_nm


def estimated_motor_current_sum(env, amp_per_nm: float, asset_cfg=SceneEntityCfg('robot')):
    """Return per-environment sum of all four leg current proxies [A]."""
    return estimated_leg_current_sums(env, amp_per_nm, asset_cfg).sum(dim=-1)


def estimated_total_current_excess(env, amp_per_nm: float, threshold_a: float = 7.0,
                                   scale_a: float = 1.0, asset_cfg=SceneEntityCfg('robot')):
    """Squared aggregate proxy excess; threshold/scale are amperes."""
    if not math.isfinite(threshold_a) or threshold_a < 0:
        raise ValueError('threshold_a must be finite and nonnegative')
    if not math.isfinite(scale_a) or scale_a <= 0:
        raise ValueError('scale_a must be finite and positive')
    total = estimated_motor_current_sum(env, amp_per_nm, asset_cfg)
    return torch.square(torch.clamp_min(total - threshold_a, 0.0) / scale_a)


def estimated_leg_current_excess(env, amp_per_nm: float, threshold_a: float = 3.0,
                                 scale_a: float = 1.0, asset_cfg=SceneEntityCfg('robot')):
    """Sum squared normalized excesses over the four independent leg limits."""
    if not math.isfinite(threshold_a) or threshold_a < 0:
        raise ValueError('threshold_a must be finite and nonnegative')
    if not math.isfinite(scale_a) or scale_a <= 0:
        raise ValueError('scale_a must be finite and positive')
    currents = estimated_leg_current_sums(env, amp_per_nm, asset_cfg)
    return torch.square(torch.clamp_min(currents - threshold_a, 0.0) / scale_a).sum(dim=-1)
