"""Motor acceleration measured over the previous five milliseconds."""
import torch
from isaaclab.managers import ManagerTermBase

from .motor_drive_math import joint_to_motor_velocity


class MotorControlIntervalAcceleration(ManagerTermBase):
    """Measure motor acceleration over 5 ms at each control step."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        robot = env.scene[cfg.params['asset_cfg'].name]
        names = list(robot.joint_names)
        self.thigh = [names.index(f'{leg}_thigh') for leg in ('lf', 'rf', 'lb', 'rb')]
        self.shin = [names.index(f'{leg}_shin') for leg in ('lf', 'rf', 'lb', 'rb')]
        self.physics_steps = round(0.005 / env.physics_dt)
        if self.physics_steps < 1 or abs(self.physics_steps * env.physics_dt - 0.005) > 1e-9:
            raise ValueError('5 ms acceleration window must be an integer number of physics steps')
        self.motor_velocity_history = torch.zeros((self.physics_steps, *robot.data.joint_vel.shape),
                                                   dtype=robot.data.joint_vel.dtype, device=robot.data.joint_vel.device)
        self.history_step = torch.full((self.physics_steps, robot.data.joint_vel.shape[0]), -1,
                                       dtype=torch.long, device=robot.data.joint_vel.device)
        robot.actuators['legs']._acceleration_capture = self

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.history_step[:, ids] = -1
        self.motor_velocity_history[:, ids] = 0.0

    def record_physics_velocity(self, joint_velocity):
        # Called immediately before each physics step. The environment counter
        # advances before the write; sample n is the velocity at the start of n.
        step = self._env._sim_step_counter
        slot = step % self.physics_steps
        self.motor_velocity_history[slot].copy_(joint_to_motor_velocity(joint_velocity, self.thigh, self.shin))
        self.history_step[slot].fill_(step)

    def __call__(self, env, asset_cfg):
        robot = env.scene[asset_cfg.name]
        motor_velocity = joint_to_motor_velocity(robot.data.joint_vel, self.thigh, self.shin)
        prior_step = env._sim_step_counter - self.physics_steps + 1
        slot = prior_step % self.physics_steps
        valid = self.history_step[slot] == prior_step
        acceleration = (motor_velocity - self.motor_velocity_history[slot]) / (self.physics_steps * env.physics_dt)
        return torch.where(valid, acceleration.square().sum(dim=1), 0.0)
