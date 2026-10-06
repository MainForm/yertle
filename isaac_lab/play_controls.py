"""Play-only controls, independent of Isaac Sim startup and training config."""

VX_RANGE = (-0.5, 0.3)
VY_RANGE = (-0.2, 0.2)
YAW_RANGE = (-1.0, 1.0)
PUSH_KEYS = {"i": (1.0, 0.0, "+X"), "k": (-1.0, 0.0, "-X"),
             "j": (0.0, 1.0, "+Y"), "l": (0.0, -1.0, "-Y")}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def configure_play_terminations(env_cfg):
    """Disable body-contact resets only; preserve timeouts and other terms."""
    env_cfg.terminations.base_contact = None


def handle_key(key, command, step_v, step_yaw):
    """Return (changed, quit, push, push_delta, reset) for one terminal key."""
    if key is not None:
        key = key.lower()
    if key == "r": return False, False, None, None, True
    if key in ("x", "\x1b"): return False, True, None, None, False
    if key in PUSH_KEYS: return False, False, PUSH_KEYS[key], None, False
    if key == "[": return False, False, None, -0.10, False
    if key == "]": return False, False, None, 0.10, False
    if key == "w": command[0] = clamp(command[0] + step_v, *VX_RANGE)
    elif key == "s": command[0] = clamp(command[0] - step_v, *VX_RANGE)
    elif key == "q": command[1] = clamp(command[1] + step_v, *VY_RANGE)
    elif key == "e": command[1] = clamp(command[1] - step_v, *VY_RANGE)
    elif key == "a": command[2] = clamp(command[2] + step_yaw, *YAW_RANGE)
    elif key == "d": command[2] = clamp(command[2] - step_yaw, *YAW_RANGE)
    elif key == " ": command[:] = [0.0, 0.0, 0.0]
    else: return False, False, None, None, False
    return True, False, None, None, False


def apply_command(base_env, command_tensor):
    term = base_env.command_manager.get_term("base_velocity")
    term.vel_command_b[:, :] = command_tensor
    term.is_standing_env[:] = False
    if hasattr(term, "is_heading_env"):
        term.is_heading_env[:] = False
    term.time_left[:] = 1.0e6


def clear_manual_push(pulse):
    """Safely clear even a force tensor created inside policy inference mode."""
    import torch
    with torch.inference_mode():
        pulse.remaining = 0
        pulse.force_w.zero_()
        pulse.apply()


def reset_play_environment(env, pulse, command, command_tensor):
    """Reset all envs to the configured pose and stop motion commands/forces.

    Recompute observations AFTER overriding the reset's newly sampled command,
    so the next inference sees zero velocity commands and fresh reset state.
    Normal env.reset also clears action history and episode/phase counters.
    """
    clear_manual_push(pulse)
    env.reset()
    command[:] = [0.0, 0.0, 0.0]
    command_tensor.zero_()
    apply_command(env.unwrapped, command_tensor)
    pulse.apply()
    obs = env.get_observations()
    return obs[0] if isinstance(obs, tuple) else obs
