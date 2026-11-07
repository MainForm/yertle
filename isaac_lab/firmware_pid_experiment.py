"""Isolated 20 ms F103 PID experiment; PWM-to-torque plant is hypothetical."""
import os
import torch
from isaaclab.actuators import IdealPDActuator, IdealPDActuatorCfg
from isaaclab.utils import configclass
from .motor_drive_math import joint_to_motor_velocity, motor_to_joint_effort
from .motor_operating_range import MotorRawTransform
from .firmware_pid_reference import FirmwarePID


class FirmwareExperiment(IdealPDActuator):
    def __init__(self, cfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        if self._num_envs != 1 or self._device != 'cpu':
            raise ValueError('This firmware experiment supports exactly one CPU environment')
        self.transform = MotorRawTransform(self._joint_names, device=self._device)
        self.thigh = [self._joint_names.index(f'{leg}_thigh') for leg in ('lf','rf','lb','rb')]
        self.shin = [self._joint_names.index(f'{leg}_shin') for leg in ('lf','rf','lb','rb')]
        self.states = [FirmwarePID() for _ in range(12)]
        self.tick = 0
        self.period_steps = round(0.02 / float(os.environ.get('DIAG_DT', '0.005')))
        self.pwm = torch.zeros_like(self.computed_effort)
        self.motor_effort = torch.zeros_like(self.computed_effort)
        self.guard = False
        self.hold = False
        self.update_count = 0
        self.max_speed_substep = 0.0
        self.max_torque_substep = 0.0
        self.trace = []
        self.pid_raw = [0] * 12

    def reset(self, env_ids):
        self.states = [FirmwarePID() for _ in range(12)]
        self.tick = 0
        self.pwm.zero_()
        self.guard = False

    def compute(self, action, joint_pos, joint_vel):
        if self.hold:
            self.motor_effort.zero_()
            self.applied_effort.zero_()
            self.computed_effort.zero_()
            action.joint_positions = None
            action.joint_velocities = None
            action.joint_efforts = self.applied_effort
            return action
        motor_v = joint_to_motor_velocity(joint_vel, self.thigh, self.shin)
        if self.tick % self.period_steps == 0 and not self.guard:
            measured = torch.floor(self.transform(joint_pos)[0]+0.5).to(torch.int64).tolist()
            goal = torch.floor(self.transform(action.joint_positions)[0]+0.5).to(torch.int64).tolist()
            out = []
            for i, (g, q) in enumerate(zip(goal, measured)):
                if not 0 <= g <= 4095 or not 0 <= q <= 4095:
                    self.guard = True
                    break
                out.append(self.states[i].step(g, q))
            if self.guard:
                self.pwm.zero_()
            else:
                self.pid_raw = out.copy()
                # PWM sign -1 corresponds to increasing calibrated encoder raw.
                for i, idx in enumerate(self.transform.indices):
                    self.pwm[0,idx] = -out[i] * float(self.transform.signs[i]) / 1000.0
                self.update_count += 1
        # Assumption, not identified hardware: linear DC back-EMF, no-load
        # speed 4.6 rad/s at 100% PWM; motor torque capped at 1.024 Nm.
        # Back-EMF is a model, not a hard cap on externally driven velocity.
        outward = ((motor_v >= self.cfg.motor_free_speed) & (self.pwm > 0)) | ((motor_v <= -self.cfg.motor_free_speed) & (self.pwm < 0))
        drive = torch.where(outward, torch.zeros_like(self.pwm), self.pwm)
        back_emf = motor_v/self.cfg.motor_free_speed if os.environ.get('DIAG_PLANT','dc') == 'dc' else torch.zeros_like(motor_v)
        stall_torque = float(os.environ.get('DIAG_STALL_TORQUE', '1.024'))
        self.motor_effort = (stall_torque*(drive - back_emf)).clamp(-1.024,1.024) if not self.guard else torch.zeros_like(self.pwm)
        self.applied_effort = motor_to_joint_effort(self.motor_effort,self.thigh,self.shin)
        self.computed_effort = self.applied_effort.clone()
        self.max_speed_substep = max(self.max_speed_substep,float(motor_v.abs().max()))
        self.max_torque_substep = max(self.max_torque_substep,float(self.motor_effort.abs().max()))
        self.trace.append({'tick':self.tick, 'speed':motor_v[0].tolist(),
                           'torque':self.motor_effort[0].tolist(), 'pid_pwm':self.pid_raw.copy(),
                           'p': [getattr(x,'p_term',0.) for x in self.states],
                           'i': [float(x.integral) for x in self.states],
                           'd': [getattr(x,'d_term',0.) for x in self.states],
                           'guard':self.guard})
        self.tick += 1
        action.joint_positions = None
        action.joint_velocities = None
        action.joint_efforts = self.applied_effort
        return action


@configclass
class FirmwareExperimentCfg(IdealPDActuatorCfg):
    class_type: type = FirmwareExperiment
    motor_free_speed: float = 4.6


def configure_firmware_experiment(cfg):
    cfg.scene.robot.actuators['legs'] = FirmwareExperimentCfg(joint_names_expr=['.*_shoulder','.*_thigh','.*_shin'], stiffness=0., damping=0., effort_limit_sim=1.e9, velocity_limit_sim=1.e9)
