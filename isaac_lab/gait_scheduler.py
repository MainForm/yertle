"""Full-range velocity commands for Yertle's fixed trot gait."""

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand


class GaitScheduler(UniformVelocityCommand):
    """Sample vx, vy, and yaw independently; every command uses trot.

    Low-speed and nearly straight commands remain in the sampling range.
    Heading conversion and automatic standing are disabled so the sampled
    velocities are used directly.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.sampled_gait_mode = torch.ones(self.num_envs, dtype=torch.long, device=self.device)

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return

        commands = torch.empty((env_ids.numel(), 3), device=self.device)
        commands[:, 0].uniform_(*self.cfg.ranges.lin_vel_x)
        commands[:, 1].uniform_(*self.cfg.ranges.lin_vel_y)
        commands[:, 2].uniform_(*self.cfg.ranges.ang_vel_z)
        self.vel_command_b[env_ids] = commands
        self.sampled_gait_mode[env_ids] = 1
        self.is_heading_env[env_ids] = False
        self.is_standing_env[env_ids] = False
        self.heading_target[env_ids] = 0.0
