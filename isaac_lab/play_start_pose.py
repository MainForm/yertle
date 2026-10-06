"""Play-only joint reset selection; never change the trained default offsets."""

import json
import math
from pathlib import Path


JOINT_NAMES = frozenset(
    f"{leg}_{kind}" for leg in ("lf", "rf", "lb", "rb")
    for kind in ("shoulder", "thigh", "shin")
)


def load_pose_json(path):
    """Read the editor's schema-1 format by sim joint name, not motor ID."""
    with Path(path).expanduser().open(encoding="utf-8-sig") as stream:
        data = json.load(stream)
    if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data["schema_version"] != 1:
        raise ValueError("Expected a pose JSON with schema_version=1.")
    names, angles = data.get("joint_names"), data.get("angles_rad")
    if (not isinstance(names, list) or len(names) != 12
            or any(not isinstance(name, str) for name in names) or set(names) != JOINT_NAMES):
        raise ValueError("Pose must contain exactly the 12 unique Yertle joint names.")
    if not isinstance(angles, list) or len(angles) != 12:
        raise ValueError("Pose must contain 12 angles_rad values.")
    if any(type(q) not in (int, float) or not math.isfinite(q) for q in angles):
        raise ValueError("Pose angles_rad must contain finite numbers.")
    if "angles_deg" in data:
        degrees = data["angles_deg"]
        if (not isinstance(degrees, list) or len(degrees) != 12
                or any(type(d) not in (int, float) or not math.isfinite(d)
                       or abs(math.radians(d) - q) > 1e-6 for d, q in zip(degrees, angles))):
            raise ValueError("angles_deg and angles_rad disagree.")
    if "joints" in data:
        joints = data["joints"]
        if not isinstance(joints, dict) or set(joints) != JOINT_NAMES:
            raise ValueError("Named joint data does not match joint_names.")
        for name, q in zip(names, angles):
            record = joints[name]
            value = record.get("angle_rad") if isinstance(record, dict) else None
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value - q) > 1e-6:
                raise ValueError(f"Named rad angle disagrees for {name}.")
    return dict(zip(names, map(float, angles)))


def select_start_pose(mode=None, pose_json=None):
    """Preserve legacy randomized reset unless a start pose is requested."""
    mode = mode or ("json" if pose_json else "random")
    if mode not in ("random", "default", "json"):
        raise ValueError("Unknown start pose mode.")
    if mode == "json" and not pose_json:
        raise ValueError("--start-pose json requires --pose-json <path>.")
    if mode != "json" and pose_json:
        raise ValueError("--pose-json cannot be combined with --start-pose random/default.")
    return mode, load_pose_json(pose_json) if mode == "json" else None


def configure_start_pose(env_cfg, mode, joint_angles):
    """Replace only this play instance's joint reset event (including auto resets)."""
    if mode == "random":
        return
    event = env_cfg.events.reset_robot_joints
    asset_cfg = event.params.get("asset_cfg")
    event.func = reset_selected_pose
    event.params = {"joint_angles": joint_angles}
    if asset_cfg is not None:
        event.params["asset_cfg"] = asset_cfg


def reset_selected_pose(env, env_ids, joint_angles=None, asset_cfg=None):
    """Set current state/targets only; validate against actual PhysX hard limits."""
    import torch

    robot = env.scene["robot" if asset_cfg is None else asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.long)
    else:
        env_ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
    if env_ids.numel() == 0:
        return
    positions = robot.data.default_joint_pos[env_ids].clone()
    if joint_angles is not None:
        if set(robot.joint_names) != JOINT_NAMES or set(joint_angles) != JOINT_NAMES:
            raise ValueError("Start pose does not match the robot's 12 joint names.")
        positions[:] = positions.new_tensor([joint_angles[name] for name in robot.joint_names])
    limits = robot.data.joint_pos_limits[env_ids]
    invalid = (~torch.isfinite(positions) | (positions < limits[..., 0]) | (positions > limits[..., 1]))
    if invalid.any():
        row, column = invalid.nonzero()[0].tolist()
        q = positions[row, column].item()
        lower, upper = limits[row, column].tolist()
        raise ValueError(
            f"Start pose {robot.joint_names[column]}={math.degrees(q):.2f} deg is outside "
            f"PhysX limits [{math.degrees(lower):.2f}, {math.degrees(upper):.2f}] deg. "
            "The paused pose editor's +/-180 deg range does not change play limits."
        )
    velocities = torch.zeros_like(positions)
    robot.write_joint_state_to_sim(positions, velocities, env_ids=env_ids)
    robot.set_joint_position_target(positions, env_ids=env_ids)
    robot.set_joint_velocity_target(velocities, env_ids=env_ids)
