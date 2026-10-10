"""Optional roll/pitch fall termination for standing locomotion experiments."""
import math


def body_roll_pitch_limit(env, limit_angle, asset_cfg):
    """Terminate when absolute body roll OR pitch reaches the supplied radians.

    Body-frame gravity removes yaw dependence. Roll covers +/-pi, so an
    upside-down body is also terminated, not mistaken for an upright posture.
    """
    import torch
    gravity = env.scene[asset_cfg.name].data.projected_gravity_b
    roll = torch.atan2(gravity[:, 1], -gravity[:, 2]).abs()
    pitch = torch.atan2(gravity[:, 0], torch.linalg.vector_norm(gravity[:, 1:], dim=1)).abs()
    return (roll >= limit_angle) | (pitch >= limit_angle)


def configure_body_tilt_termination(env_cfg, limit_degrees):
    if limit_degrees is None:
        return
    if not math.isfinite(limit_degrees) or not 0 < limit_degrees <= 90:
        raise ValueError("Body tilt termination angle must be in (0, 90] degrees.")
    from isaaclab.managers import SceneEntityCfg, TerminationTermCfg
    env_cfg.terminations.body_tilt = TerminationTermCfg(
        func=body_roll_pitch_limit,
        params={"limit_angle": math.radians(limit_degrees), "asset_cfg": SceneEntityCfg("robot")},
    )
