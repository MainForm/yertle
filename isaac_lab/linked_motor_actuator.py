"""Opt-in explicit PD drive with ideal thigh/shin motor coupling."""
from isaaclab.actuators import IdealPDActuator, IdealPDActuatorCfg
from isaaclab.utils import configclass

from .motor_drive_math import joint_to_motor_effort, joint_to_motor_velocity, motor_to_joint_effort, limit_motor_effort


class LinkedMotorActuator(IdealPDActuator):
    def __init__(self, cfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self.thigh = [self._joint_names.index(f'{leg}_thigh') for leg in ('lf', 'rf', 'lb', 'rb')]
        self.shin = [self._joint_names.index(f'{leg}_shin') for leg in ('lf', 'rf', 'lb', 'rb')]
        if len(self._joint_names) != 12:
            raise ValueError('Linked drive requires all twelve joints in one actuator group')
        self.motor_effort = self.computed_effort.clone()

    def reset(self, env_ids):
        self.motor_effort[env_ids] = 0

    def compute(self, control_action, joint_pos, joint_vel):
        self.computed_effort = (self.stiffness * (control_action.joint_positions - joint_pos)
                                + self.damping * (control_action.joint_velocities - joint_vel)
                                + control_action.joint_efforts)
        motor_effort = joint_to_motor_effort(self.computed_effort, self.thigh, self.shin)
        motor_vel = joint_to_motor_velocity(joint_vel, self.thigh, self.shin)
        self.motor_effort = limit_motor_effort(motor_effort, motor_vel,
                                               self.cfg.motor_torque_limit, self.cfg.motor_speed_limit)
        self.applied_effort = motor_to_joint_effort(self.motor_effort, self.thigh, self.shin)
        control_action.joint_efforts = self.applied_effort
        control_action.joint_positions = None
        control_action.joint_velocities = None
        return control_action


@configclass
class LinkedMotorActuatorCfg(IdealPDActuatorCfg):
    class_type: type = LinkedMotorActuator
    motor_torque_limit: float = 1.024
    motor_speed_limit: float = 2.3


def configure_linked_motor_actuator(env_cfg):
    """Keep the joint/reference mapping; opt in to ideal coupled motor drives."""
    env_cfg.scene.robot.actuators['legs'] = LinkedMotorActuatorCfg(
        joint_names_expr=['.*_shoulder', '.*_thigh', '.*_shin'],
        stiffness=25.0, damping=0.5, effort_limit_sim=1.0e9, velocity_limit_sim=1.0e9,
        motor_torque_limit=1.024, motor_speed_limit=2.3,
    )
    if hasattr(env_cfg.rewards, 'joint_velocity_excess'):
        from .motor_space_rewards import configure_motor_space_rewards
        configure_motor_space_rewards(env_cfg)
