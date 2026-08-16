"""Mode-balanced velocity command scheduler for Yertle."""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand


class GaitScheduler(UniformVelocityCommand):
    """Sample walk, trot, and bound commands before sampling their velocities.

    The scheduler classifies the sampled command with the same conditions used
    here.  Selecting a mode first avoids the geometric bias of uniform command
    sampling, where the narrow bound region was only about 0.56 percent.
    """

    WALK_PROBABILITY = 0.27
    TROT_PROBABILITY = 0.46
    BOUND_PROBABILITY = 0.27

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.sampled_gait_mode = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return

        selector = torch.rand(env_ids.numel(), device=self.device)
        walk_mask = selector < self.WALK_PROBABILITY
        trot_mask = (selector >= self.WALK_PROBABILITY) & (
            selector < self.WALK_PROBABILITY + self.TROT_PROBABILITY
        )
        bound_mask = ~walk_mask & ~trot_mask

        walk_ids = env_ids[walk_mask]
        trot_ids = env_ids[trot_mask]
        bound_ids = env_ids[bound_mask]

        self._sample_walk(walk_ids)
        self._sample_trot(trot_ids)
        self._sample_bound(bound_ids)

        self.sampled_gait_mode[walk_ids] = 0
        self.sampled_gait_mode[trot_ids] = 1
        self.sampled_gait_mode[bound_ids] = 2
        # V4.2 uses direct yaw-rate commands. Heading conversion or standing
        # would alter the scheduler classification after this sampling step.
        self.is_heading_env[env_ids] = False
        self.is_standing_env[env_ids] = False
        self.heading_target[env_ids] = 0.0

    def _sample_walk(self, env_ids: torch.Tensor):
        if env_ids.numel() == 0:
            return
        # Uniform area sampling within a 0.11 m/s disk keeps planar speed
        # strictly below the scheduler's walk threshold of 0.12 m/s.
        radius = torch.sqrt(torch.rand(env_ids.numel(), device=self.device)) * 0.11
        angle = torch.rand(env_ids.numel(), device=self.device) * (2.0 * math.pi)
        self.vel_command_b[env_ids, 0] = radius * torch.cos(angle)
        self.vel_command_b[env_ids, 1] = radius * torch.sin(angle)
        self.vel_command_b[env_ids, 2] = torch.empty(env_ids.numel(), device=self.device).uniform_(
            *self.cfg.ranges.ang_vel_z
        )

    def _sample_trot(self, env_ids: torch.Tensor):
        """Sample the global range while rejecting walk and bound regions."""
        remaining = env_ids
        for _ in range(32):
            if remaining.numel() == 0:
                return
            count = remaining.numel()
            candidate = torch.empty((count, 3), device=self.device)
            candidate[:, 0].uniform_(*self.cfg.ranges.lin_vel_x)
            candidate[:, 1].uniform_(*self.cfg.ranges.lin_vel_y)
            candidate[:, 2].uniform_(*self.cfg.ranges.ang_vel_z)
            planar_speed = torch.linalg.vector_norm(candidate[:, :2], dim=1)
            is_bound = (candidate[:, 0].abs() > 0.24) & (candidate[:, 1].abs() < 0.06) & (candidate[:, 2].abs() < 0.25)
            accept = (planar_speed >= 0.12) & ~is_bound
            if accept.any():
                self.vel_command_b[remaining[accept]] = candidate[accept]
            remaining = remaining[~accept]

        # Rejection is overwhelmingly likely to finish on its first pass. This
        # deterministic fallback merely protects against an unexpected range edit.
        self.vel_command_b[remaining, 0] = -0.30
        self.vel_command_b[remaining, 1] = 0.0
        self.vel_command_b[remaining, 2] = 0.0

    def _sample_bound(self, env_ids: torch.Tensor):
        if env_ids.numel() == 0:
            return
        # The visual-forward direction is -X. Keep +X samples for the return
        # gait, but make -X bound samples the common high-speed case.
        negative_x = torch.rand(env_ids.numel(), device=self.device) < 0.75
        negative_count = int(negative_x.sum().item())
        positive_count = int((~negative_x).sum().item())
        vx = torch.empty(env_ids.numel(), device=self.device)
        vx[negative_x] = torch.empty(negative_count, device=self.device).uniform_(-0.50, -0.25)
        vx[~negative_x] = torch.empty(positive_count, device=self.device).uniform_(0.25, 0.30)
        self.vel_command_b[env_ids, 0] = vx
        self.vel_command_b[env_ids, 1] = torch.empty(env_ids.numel(), device=self.device).uniform_(-0.05, 0.05)
        self.vel_command_b[env_ids, 2] = torch.empty(env_ids.numel(), device=self.device).uniform_(-0.20, 0.20)
