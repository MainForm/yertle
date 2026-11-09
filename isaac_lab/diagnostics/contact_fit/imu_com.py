"""Use recorded gravity to test bounded, time-varying body CoM offsets."""

import math

import numpy as np
import torch

from mass_cases import BASELINES


UPDATE_S = 0.020
FILTER_S = 0.100
DEADBAND_DEG = 3.0
FULL_OFFSET_DEG = 25.0
HALF_INERTIA = (0.5, 0.5, 0.5)
DEFAULT_INERTIA = (1.0, 1.0, 1.0)

# Each entry is (initial offset in mm, inertia scale, motion axes, cap in mm).
FITTED = {
    "centered_default": ((0, 0, 0), DEFAULT_INERTIA, None, 0),
    "centered_half": ((0, 0, 0), HALF_INERTIA, None, 0),
    "fixed_right15_half": ((0, -15, 0), HALF_INERTIA, None, 0),
    "lateral10_half": ((0, 0, 0), HALF_INERTIA, "y", 10),
    "lateral15_half": ((0, 0, 0), HALF_INERTIA, "y", 15),
    "lateral20_half": ((0, 0, 0), HALF_INERTIA, "y", 20),
    "planar10_half": ((0, 0, 0), HALF_INERTIA, "xy", 10),
    "planar15_half": ((0, 0, 0), HALF_INERTIA, "xy", 15),
    "planar20_half": ((0, 0, 0), HALF_INERTIA, "xy", 20),
    "planar15_default": ((0, 0, 0), DEFAULT_INERTIA, "xy", 15),
}
ORIGINAL = {
    name: FITTED[name] for name in ("centered_default", "planar15_half")
}
CASES = [
    ("goal", f"{material}__{name}", BASELINES[material])
    for material, options in (("original_friction", ORIGINAL), ("fitted_friction", FITTED))
    for name in options
]
VARIANTS = {name: value[:2] for name, value in FITTED.items()}


def offset_profile(feedback, count, axes, cap_mm):
    """Sample only IMU values available at the current replay time."""
    sample_times = np.array([row["t"] for row in feedback])
    times = np.arange(count) * UPDATE_S
    indices = np.searchsorted(sample_times, times, side="right") - 1
    assert sample_times[0] == 0 and np.all(np.diff(sample_times) > 0)
    gravity = np.array([row["g"] for row in feedback])[indices]
    assert np.isfinite(gravity).all() and np.all(np.linalg.norm(gravity, axis=1) > 0)
    horizontal = gravity[:, :2].copy()
    if axes == "y":
        horizontal[:, 0] = 0.0
    radius = np.linalg.norm(horizontal, axis=1)
    angle = np.degrees(np.arctan2(radius, -gravity[:, 2]))
    fraction = np.clip((angle - DEADBAND_DEG) / (FULL_OFFSET_DEG - DEADBAND_DEG), 0, 1)
    direction = horizontal / np.maximum(radius[:, None], 1e-12)
    targets = direction * fraction[:, None] * (cap_mm / 1000.0)
    filtered = np.zeros((count, 3))
    alpha = 1.0 - math.exp(-UPDATE_S / FILTER_S)
    for tick in range(1, count):
        filtered[tick, :2] = filtered[tick - 1, :2] + alpha * (
            targets[tick] - filtered[tick - 1, :2]
        )
    assert np.max(np.linalg.norm(filtered, axis=1)) <= cap_mm / 1000.0 + 1e-9
    return filtered


class ImuComReplay:
    def __init__(self, robot, feedback, end_time, base_offsets, cases=CASES):
        self.robot = robot
        self.view = robot.root_physx_view
        self.body = robot.body_names.index("base_link")
        self.coms = self.view.get_coms().clone()
        self.nominal = self.coms[:, self.body, :3].clone()
        self.offsets = base_offsets.clone()
        count = 2 + int(end_time / UPDATE_S)
        dynamic_rows = []
        profiles = []
        for row, (_, name, _) in enumerate(cases):
            _, _, axes, cap = FITTED[name.split("__", 1)[1]]
            if axes is not None:
                dynamic_rows.append(row)
                profiles.append(offset_profile(feedback, count, axes, cap))
        self.rows = torch.tensor(dynamic_rows, dtype=torch.int32)
        self.profiles = self.coms.new_tensor(np.stack(profiles, axis=1))
        self.update_steps = round(UPDATE_S / 0.001)

    def step(self, physics_step):
        if physics_step == 0 or physics_step % self.update_steps:
            return
        from isaaclab.utils.math import quat_apply

        rows = self.rows
        previous = self.offsets[rows]
        target = self.profiles[physics_step // self.update_steps]
        if torch.equal(previous, target):
            return
        rotation = self.robot.data.root_link_quat_w[rows]
        delta_w = quat_apply(rotation, target - previous)
        # A CoM shift must not inject a step in the velocity of the body frame.
        velocity = self.view.get_root_velocities()[rows].clone()
        velocity[:, :3] += torch.linalg.cross(velocity[:, 3:], delta_w, dim=-1)
        pose = self.view.get_root_transforms()[rows].clone()
        self.coms[rows, self.body, :3] = self.nominal[rows] + target
        self.view.set_coms(self.coms, rows)
        torch.testing.assert_close(self.view.get_coms()[rows, self.body], self.coms[rows, self.body])
        torch.testing.assert_close(self.view.get_root_transforms()[rows], pose, atol=1e-6, rtol=0)
        self.robot.write_root_com_velocity_to_sim(velocity, env_ids=rows)
        self.offsets[rows] = target

    def reset_offsets(self):
        self.coms[self.rows, self.body, :3] = self.nominal[self.rows]
        self.view.set_coms(self.coms, self.rows)
