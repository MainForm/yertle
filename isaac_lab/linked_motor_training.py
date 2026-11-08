"""Explicit F103-style raw-position PID and approximate PWM-to-torque drive."""
import torch
from isaaclab.actuators import IdealPDActuator, IdealPDActuatorCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

from .motor_drive_math import MotorSupplySag, joint_to_motor_velocity, limit_motor_effort, motor_to_joint_effort
from .motor_operating_range import MotorRawTransform
from .firmware_pid_torch import firmware_pid_step


class LinkedMotorActuator(IdealPDActuator):
    def __init__(self, cfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self.thigh = [self._joint_names.index(f'{leg}_thigh') for leg in ('lf', 'rf', 'lb', 'rb')]
        self.shin = [self._joint_names.index(f'{leg}_shin') for leg in ('lf', 'rf', 'lb', 'rb')]
        if len(self._joint_names) != 12:
            raise ValueError('Linked drive requires all twelve joints in one actuator group')
        self.motor_effort = self.computed_effort.clone()
        # Signed motor PWM in articulation order; held between controller ticks.
        self.motor_pwm = torch.zeros_like(self.motor_effort)
        self.raw_transform = MotorRawTransform(self._joint_names, device=self.motor_effort.device, dtype=self.motor_effort.dtype)
        self.motor_supply = MotorSupplySag(device=self.motor_effort.device, dtype=self.motor_effort.dtype)
        self.motor_terminal_voltage = self.motor_supply.baseline_v.expand_as(self.motor_pwm).clone()
        # PID arrays use physical motor ID order, not articulation joint order.
        self.integral_pwm = torch.zeros_like(self.motor_effort)
        self.previous_raw = torch.zeros_like(self.motor_effort)
        self.pid_initialized = torch.zeros_like(self.motor_effort, dtype=torch.bool)
        self.pid_guard = torch.zeros(self.motor_effort.shape[0], dtype=torch.bool, device=self.motor_effort.device)
        self._base_integral = self.integral_pwm.clone()
        self._base_previous = self.previous_raw.clone()
        self._base_initialized = self.pid_initialized.clone()
        self._base_guard = self.pid_guard.clone()
        self._pid_last_step = torch.full((self.motor_effort.shape[0],), -1, dtype=torch.long, device=self.motor_effort.device)
        self._refresh_mask = torch.ones(self.motor_effort.shape[0], dtype=torch.bool, device=self.motor_effort.device)
        self._refresh_pending = True
        self._sim = SimulationContext.instance()
        if self._sim is None:
            raise RuntimeError('Linked motor actuator requires a simulation context')
        self._clock_origin = None

    def reset(self, env_ids):
        ids = slice(None) if env_ids is None else env_ids
        self.motor_effort[ids] = 0
        self.motor_pwm[ids] = 0
        self.motor_terminal_voltage[ids] = self.motor_supply.baseline_v
        self.computed_effort[ids] = 0
        self.integral_pwm[ids] = 0
        self.previous_raw[ids] = 0
        self.pid_initialized[ids] = False
        self.pid_guard[ids] = False
        self._pid_last_step[ids] = -1
        self._refresh_mask[ids] = True
        self._refresh_pending = True

    def compute(self, control_action, joint_pos, joint_vel):
        physics_step = self._sim.current_time_step_index
        acceleration_capture = getattr(self, '_acceleration_capture', None)
        if acceleration_capture is not None:
            acceleration_capture.record_physics_velocity(joint_vel)
        if self._clock_origin is None:
            self._clock_origin = physics_step
        periodic = (physics_step - self._clock_origin) % self.cfg.control_decimation == 0
        if periodic or self._refresh_pending:
            goal = torch.floor(self.raw_transform(control_action.joint_positions) + 0.5)
            measured = torch.floor(self.raw_transform(joint_pos) + 0.5)
            valid = (torch.isfinite(goal) & torch.isfinite(measured) & (goal >= 0) & (goal <= 4095)
                     & (measured >= 0) & (measured <= 4095)).all(dim=-1)
            # Invalid positions trip an environment-wide stop until episode reset.
            goal = torch.nan_to_num(goal, nan=0.0, posinf=4095.0, neginf=0.0).clamp(0, 4095)
            measured = torch.nan_to_num(measured, nan=0.0, posinf=4095.0, neginf=0.0).clamp(0, 4095)
            active = torch.ones_like(self._refresh_mask) if periodic else self._refresh_mask
            same_step = self._pid_last_step == physics_step
            mask = active.unsqueeze(-1)
            fresh = active & ~same_step
            self._base_integral = torch.where(fresh.unsqueeze(-1), self.integral_pwm, self._base_integral)
            self._base_previous = torch.where(fresh.unsqueeze(-1), self.previous_raw, self._base_previous)
            self._base_initialized = torch.where(fresh.unsqueeze(-1), self.pid_initialized, self._base_initialized)
            self._base_guard = torch.where(fresh, self.pid_guard, self._base_guard)
            guarded = self._base_guard | ~valid
            pwm, integral, previous = firmware_pid_step(
                goal, measured, self._base_previous, self._base_integral, self._base_initialized,
                self.cfg.control_period_s, self.cfg.kp_pwm, self.cfg.ki_pwm, self.cfg.kd_pwm,
                self.cfg.integral_limit_pwm, self.cfg.duty_limit_pwm)
            can_update = mask & ~guarded.unsqueeze(-1)
            self.integral_pwm = torch.where(can_update, integral, self.integral_pwm)
            self.previous_raw = torch.where(can_update, previous, self.previous_raw)
            self.pid_initialized = torch.where(can_update, torch.ones_like(self.pid_initialized), self.pid_initialized)
            self.pid_guard = torch.where(active, guarded, self.pid_guard)
            self._pid_last_step = torch.where(active, torch.full_like(self._pid_last_step, physics_step), self._pid_last_step)
            # F103 polarity is -1 for all motors. Undo that polarity into the
            # calibrated joint-coordinate direction; its sign mapping is unchanged.
            motor_pwm = torch.zeros_like(self.motor_pwm)
            motor_pwm[:, self.raw_transform.indices] = pwm * self.raw_transform.signs
            motor_pwm = torch.where(guarded.unsqueeze(-1), torch.zeros_like(motor_pwm), motor_pwm)
            if periodic:
                self.motor_pwm = motor_pwm
            else:
                mask = self._refresh_mask.unsqueeze(-1)
                self.motor_pwm = torch.where(mask, motor_pwm, self.motor_pwm)
            self._refresh_mask.zero_()
            self._refresh_pending = False
        # The command is held for 20 ms; speed-dependent motor response is
        # evaluated every 1 ms physical step. Invalid-raw guard is torque-free.
        motor_vel = joint_to_motor_velocity(joint_vel, self.thigh, self.shin)
        ids = self.raw_transform.indices
        motor_id_vel = motor_vel[:, ids]
        powered_vel = torch.where(self.pid_guard.unsqueeze(-1), 0.0, motor_id_vel)
        # The simulated drive applies this PWM directly, in place of servo duty feedback.
        self.motor_terminal_voltage, effective_voltage = self.motor_supply.solve(self.motor_pwm[:, ids], powered_vel)
        # Keep the original no-sag, standstill torque per unit duty.
        torque_id = (1000.0 * self.cfg.nm_per_pwm / self.motor_supply.baseline_v) * effective_voltage
        torque = torch.empty_like(self.motor_pwm)
        torque[:, ids] = limit_motor_effort(
            torque_id, motor_id_vel, self.cfg.motor_torque_limit, self.cfg.motor_speed_limit
        )
        self.motor_effort = torch.where(self.pid_guard.unsqueeze(-1), torch.zeros_like(torque), torque)
        self.applied_effort = motor_to_joint_effort(self.motor_effort, self.thigh, self.shin)
        self.computed_effort = self.applied_effort.clone()
        control_action.joint_efforts = self.applied_effort
        control_action.joint_positions = None
        control_action.joint_velocities = None
        return control_action


@configclass
class LinkedMotorActuatorCfg(IdealPDActuatorCfg):
    class_type: type = LinkedMotorActuator
    motor_torque_limit: float = 1.024
    motor_speed_limit: float = 4.6
    # Nominal no-load speed at 6.8 V, retained for existing diagnostic entry points.
    motor_free_speed: float = 6.8 / MotorSupplySag.back_emf_v_per_rad_s
    control_decimation: int = 20
    control_period_s: float = 0.020
    kp_pwm: float = 7.0
    ki_pwm: float = 3.0
    kd_pwm: float = 0.13
    integral_limit_pwm: float = 100.0
    duty_limit_pwm: float = 1000.0
    nm_per_pwm: float = 1.024 / 800.0


def configure_linked_motor_actuator(env_cfg):
    """Opt in to 1 ms physics, 20 ms F103-style PID and motor-space rewards."""
    if not hasattr(env_cfg.rewards, 'joint_velocity_excess'):
        raise ValueError('Linked motor drive currently supports the flat Yertle task only')
    env_cfg.sim.dt = 0.001
    env_cfg.decimation = 20
    env_cfg.sim.render_interval = env_cfg.decimation
    env_cfg.scene.contact_forces.update_period = env_cfg.sim.dt
    env_cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count = 64
    env_cfg.scene.robot.actuators['legs'] = LinkedMotorActuatorCfg(
        joint_names_expr=['.*_shoulder', '.*_thigh', '.*_shin'],
        stiffness=0.0, damping=0.0, effort_limit_sim=1.0e9, velocity_limit_sim=1.0e9,
        motor_torque_limit=1.024, motor_speed_limit=4.6, control_decimation=20,
    )
    from .motor_space_rewards import configure_motor_space_rewards
    configure_motor_space_rewards(env_cfg)
