"""Rolling measured servo travel balance; no action/reference/calibration edits."""
import math

import torch
from isaaclab.managers import ManagerTermBase

from .motor_operating_range import MotorRawTransform


class MotorTravelBalancePenalty(ManagerTermBase):
    """Mean per-leg squared relative difference in one-cycle travel.

    Sample actual positions at control frequency, not targets or instantaneous
    speeds. Shin servo travel uses the existing thigh+shin linkage transform.
    A rolling window permits different timing of thigh/shin movement. The
    first interval after reset is ignored (no reset teleport counted). All
    mutable history is tensor-valued for the existing checkpoint adapter.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        asset = env.scene[cfg.params['asset_cfg'].name]
        self.transform = MotorRawTransform(asset.joint_names, device=env.device,
                                           dtype=asset.data.joint_pos.dtype)
        seconds = cfg.params['window_s']
        floor = cfg.params['travel_floor_rad']
        if not math.isfinite(seconds) or seconds <= 0 or not math.isfinite(floor) or floor <= 0:
            raise ValueError('Window and travel floor must be finite and positive')
        self.window = max(1, round(seconds / env.step_dt))
        shape = (env.num_envs, 4, 2)
        self.previous = torch.zeros(shape, device=env.device)
        self.history = torch.zeros((env.num_envs, self.window, 4, 2), device=env.device)
        self.cursor = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self.valid = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self.last_step = torch.full((), -1, dtype=torch.long, device=env.device)
        self.value = torch.zeros(env.num_envs, device=env.device)

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.history[ids] = 0
        self.cursor[ids] = 0
        self.valid[ids] = False
        self.previous[ids] = 0
        self.value[ids] = 0

    def __call__(self, env, asset_cfg, command_name, window_s, travel_floor_rad):
        if self.last_step == env.common_step_counter:
            return self.value
        raw = self.transform(env.scene[asset_cfg.name].data.joint_pos)
        # RF/LF/RB/LB x (shin servo, thigh servo), absolute angle in radians.
        q = raw.reshape(-1, 4, 3)[:, :, :2] * (2 * math.pi / 4096)
        delta = (q - self.previous).abs() * self.valid[:, None, None]
        command = env.command_manager.get_command(command_name)
        moving = (torch.linalg.vector_norm(command[:, :2], dim=1) > .05) | (command[:, 2].abs() > .1)
        # Do not credit standing/zero-command motion toward later commands.
        self.history[~moving] = 0
        self.history[torch.arange(env.num_envs, device=env.device), self.cursor] = delta * moving[:, None, None]
        travel = self.history.sum(dim=1)
        shin, thigh = travel.unbind(dim=-1)
        score = ((thigh - shin) / (thigh + shin).clamp_min(travel_floor_rad)).square().mean(dim=1)
        self.value.copy_(score * moving)
        self.previous.copy_(q)
        self.valid.fill_(True)
        self.cursor.add_(1).remainder_(self.window)
        self.last_step.fill_(env.common_step_counter)
        return self.value
