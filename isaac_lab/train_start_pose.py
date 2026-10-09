"""Episode-wise sampling of coherent joint poses without changing action offsets."""

import math


def configure_reset_pose_pool(env_cfg, poses, extend_limits=False, standing_pose=None):
    """Use standing + JSON poses uniformly; an empty pool preserves legacy resets."""
    if extend_limits and not poses and standing_pose is None:
        raise ValueError("Limit extension requires a seated or standing JSON reset pose.")
    if not poses and standing_pose is None:
        return
    event = env_cfg.events.reset_robot_joints
    asset_cfg = event.params.get("asset_cfg")
    event.func = reset_pose_pool
    event.params = {"poses": poses, "extend_limits": extend_limits}
    if standing_pose is not None:
        event.params["standing_pose"] = standing_pose
    if asset_cfg is not None:
        event.params["asset_cfg"] = asset_cfg


def reset_pose_pool(env, env_ids, poses, extend_limits=False, asset_cfg=None, standing_pose=None):
    """Sample independently at each reset, validating the entire pool first.

    Index 0 uses the trained reference unless a standing reset override is given.
    Overrides never change default observation/action offsets. Expansion covers ALL envs
    and ALL candidate poses, including candidates not selected on this reset.
    Expanded limits remain active throughout the episode, not only at reset.
    """
    import torch

    robot = env.scene["robot" if asset_cfg is None else asset_cfg.name]
    ids = (torch.arange(env.num_envs, device=env.device) if env_ids is None
           else torch.as_tensor(env_ids, device=env.device, dtype=torch.long))
    if ids.numel() == 0:
        return
    if (not poses and standing_pose is None) or any(set(p) != set(robot.joint_names) for p in poses):
        raise ValueError("Reset pool must contain complete named robot poses.")
    candidates = robot.data.default_joint_pos[:, None, :].expand(-1, len(poses) + 1, -1).clone()
    if standing_pose is not None:
        if set(standing_pose) != set(robot.joint_names):
            raise ValueError("Standing reset must contain complete named robot joints.")
        candidates[:, 0] = candidates.new_tensor([standing_pose[n] for n in robot.joint_names])
    if poses:
        candidates[:, 1:] = candidates.new_tensor([[p[n] for n in robot.joint_names] for p in poses])
    if not torch.isfinite(candidates).all():
        raise ValueError("Reset poses must contain finite angles.")
    limits = robot.data.joint_pos_limits.clone()
    if extend_limits:
        if (candidates.abs() > math.pi).any():
            raise ValueError("Reset limit extension accepts only +/-180 degree poses.")
        expanded = limits.clone()
        expanded[..., 0] = torch.minimum(limits[..., 0], candidates.amin(dim=1))
        expanded[..., 1] = torch.maximum(limits[..., 1], candidates.amax(dim=1))
        if not torch.equal(expanded, limits):
            robot.write_joint_position_limit_to_sim(expanded)
            print("TRAIN_RESET_POSE_LIMITS_EXTENDED simulation_only all_envs", flush=True)
        limits = expanded
    if ((candidates < limits[:, None, :, 0]) | (candidates > limits[:, None, :, 1])).any():
        raise ValueError("Reset pose outside PhysX joint limits; use --extend-reset-pose-limits explicitly.")
    selection = torch.randint(len(poses) + 1, (ids.numel(),), device=env.device)
    positions = candidates[ids, selection]
    velocities = torch.zeros_like(positions)
    robot.write_joint_state_to_sim(positions, velocities, env_ids=ids)
    robot.set_joint_position_target(positions, env_ids=ids)
    robot.set_joint_velocity_target(velocities, env_ids=ids)
    if not hasattr(env, "reset_pose_ids"):
        env.reset_pose_ids = torch.full((env.num_envs,), -1, device=env.device, dtype=torch.long)
    env.reset_pose_ids[ids] = selection
