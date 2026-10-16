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


def configure_base_contact_termination(env_cfg):
    """Restore the original base_link >1 N contact termination, no tilt gate."""
    from isaaclab.managers import SceneEntityCfg, TerminationTermCfg
    from isaaclab.envs import mdp
    env_cfg.terminations.body_tilt=None
    env_cfg.terminations.body_tilt_and_nonsole_contact=None
    env_cfg.terminations.base_contact=TerminationTermCfg(
        func=mdp.illegal_contact,
        params={'sensor_cfg':SceneEntityCfg('contact_forces',body_names='base_link'),'threshold':1.0},
    )
    env_cfg.rewards.fall_termination_penalty.params['term_name']='base_contact'


def configure_body_tilt_termination(env_cfg, limit_degrees):
    if limit_degrees is None:
        return
    if not math.isfinite(limit_degrees) or not 0 < limit_degrees <= 90:
        raise ValueError("Body tilt termination angle must be in (0, 90] degrees.")
    from isaaclab.managers import SceneEntityCfg, TerminationTermCfg
    # Do not leave the old tilt-only term active: that would defeat AND semantics.
    env_cfg.terminations.body_tilt = None
    env_cfg.terminations.body_tilt_and_nonsole_contact = TerminationTermCfg(
        func=body_tilt_and_nonsole_ground_contact,
        params={"limit_angle": math.radians(limit_degrees), "asset_cfg": SceneEntityCfg("robot"),
                "separation_tolerance_m": 0.001, "sole_tolerance_m": 0.005},
    )


def body_tilt_and_nonsole_ground_contact(env, limit_angle, asset_cfg,
                                       separation_tolerance_m=0.001, sole_tolerance_m=0.005):
    """Fall only when excessive roll/pitch AND forbidden ground contact coexist.

    Reuse the reward contact classifier: base/shoulder/thigh contact or a shin
    contact outside the authored Rubber surface. No contact-force threshold.
    This observes the last physics substep, not a latched history of contacts.
    """
    if not math.isfinite(separation_tolerance_m) or separation_tolerance_m < 0:
        raise ValueError('Contact separation tolerance must be finite and nonnegative')
    if not math.isfinite(sole_tolerance_m) or sole_tolerance_m < 0:
        raise ValueError('Sole tolerance must be finite and nonnegative')
    tilted = body_roll_pitch_limit(env, limit_angle, asset_cfg)
    if not tilted.any():
        return tilted
    if not hasattr(env, '_yertle_ground_contacts'):
        from .ground_contact import GroundContacts
        env._yertle_ground_contacts = GroundContacts(env)
    forbidden = env._yertle_ground_contacts.read(env, separation_tolerance_m, sole_tolerance_m)
    return tilted & forbidden.any(dim=1)
