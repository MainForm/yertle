"""Exercise the 20 ms F103-style actuator without an Isaac Sim app."""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import torch
from isaac_lab.motor_operating_range import MotorRawTransform
from isaac_lab.firmware_pid_torch import firmware_pid_step


def load_actuator():
    actuators = types.ModuleType('isaaclab.actuators')
    actuators.IdealPDActuator = type('IdealPDActuator', (), {})
    actuators.IdealPDActuatorCfg = type('IdealPDActuatorCfg', (), {})
    utils = types.ModuleType('isaaclab.utils')
    utils.configclass = lambda cls: cls
    sim = types.ModuleType('isaaclab.sim')
    sim.SimulationContext = type('SimulationContext', (), {})
    isaaclab = types.ModuleType('isaaclab')
    isaaclab.__path__ = []
    path = Path(__file__).resolve().parents[1] / 'linked_motor_training.py'
    spec = importlib.util.spec_from_file_location('isaac_lab._hold_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'isaaclab': isaaclab, 'isaaclab.actuators': actuators, 'isaaclab.utils': utils, 'isaaclab.sim': sim}):
        spec.loader.exec_module(module)
    return module.LinkedMotorActuator


class LinkedMotorHoldTests(unittest.TestCase):
    def setUp(self):
        self.drive = object.__new__(load_actuator())
        self.drive.cfg = types.SimpleNamespace(control_decimation=20, control_period_s=0.020,
            motor_torque_limit=1.024, motor_speed_limit=4.6, kp_pwm=7., ki_pwm=3., kd_pwm=.13,
            integral_limit_pwm=100., duty_limit_pwm=1000., nm_per_pwm=1.024/800.)
        self.drive.thigh = [1, 4, 7, 10]
        self.drive.shin = [2, 5, 8, 11]
        names = [f'{leg}_{part}' for leg in ('lf','rf','lb','rb') for part in ('shoulder','thigh','shin')]
        self.drive.raw_transform = MotorRawTransform(names)
        self.drive.computed_effort = torch.zeros(2, 12)
        self.drive.motor_effort = torch.zeros(2, 12)
        self.drive.integral_pwm = torch.zeros(2, 12)
        self.drive.previous_raw = torch.zeros(2, 12)
        self.drive.pid_initialized = torch.zeros(2, 12, dtype=torch.bool)
        self.drive.pid_guard = torch.zeros(2, dtype=torch.bool)
        self.drive._base_integral = self.drive.integral_pwm.clone()
        self.drive._base_previous = self.drive.previous_raw.clone()
        self.drive._base_initialized = self.drive.pid_initialized.clone()
        self.drive._base_guard = self.drive.pid_guard.clone()
        self.drive._pid_last_step = torch.full((2,), -1, dtype=torch.long)
        self.drive._refresh_mask = torch.ones(2, dtype=torch.bool)
        self.drive._refresh_pending = True
        self.drive._sim = types.SimpleNamespace(current_time_step_index=0)
        self.drive._clock_origin = None

    def step(self, goal, speed=0.0):
        action = types.SimpleNamespace(joint_positions=torch.full((2, 12), goal),
                                       joint_velocities=torch.zeros(2, 12),
                                       joint_efforts=torch.zeros(2, 12))
        self.drive.compute(action, torch.zeros(2, 12), torch.full((2, 12), speed))
        self.drive._sim.current_time_step_index += 1
        return self.drive.motor_effort.clone()

    def test_torque_is_held_for_twenty_physics_steps(self):
        first = self.step(0.02)
        for _ in range(19):
            torch.testing.assert_close(self.step(0.04), first)
        self.assertFalse(torch.equal(self.step(0.04), first))

    def test_partial_reset_updates_only_reset_environment(self):
        first = self.step(0.02)
        self.drive.reset(torch.tensor([0]))
        updated = self.step(0.04)
        self.assertFalse(torch.equal(updated[0], first[0]))
        torch.testing.assert_close(updated[1], first[1])
        self.assertTrue(self.drive.pid_initialized[0].all())

    def test_speed_cutoff_is_sampled_at_controller_update(self):
        first = self.step(0.02)
        torch.testing.assert_close(self.step(0.02, speed=10), first)
        for _ in range(18):
            self.step(0.02, speed=10)
        self.assertEqual(self.step(0.02, speed=10).abs().max().item(), 0.0)

    def test_extra_nonphysics_write_does_not_shift_or_double_integrate(self):
        self.step(0.02)
        first_i = self.drive.integral_pwm.clone()
        self.drive._sim.current_time_step_index -= 1
        self.step(0.02)
        torch.testing.assert_close(self.drive.integral_pwm, first_i)
        held = self.drive.motor_effort.clone()
        for _ in range(19):
            torch.testing.assert_close(self.step(0.04), held)
        self.assertFalse(torch.equal(self.step(0.04), held))

    def test_nonzero_simulator_start_aligns_to_first_action(self):
        self.drive._sim.current_time_step_index = 2
        first = self.step(0.02)
        for _ in range(19):
            torch.testing.assert_close(self.step(0.04), first)
        self.assertFalse(torch.equal(self.step(0.04), first))

    def test_invalid_raw_position_stops_only_affected_env_until_reset(self):
        action = types.SimpleNamespace(joint_positions=torch.zeros(2, 12),
                                       joint_velocities=torch.zeros(2, 12),
                                       joint_efforts=torch.zeros(2, 12))
        action.joint_positions[0] = 100.0
        self.drive.compute(action, torch.zeros(2, 12), torch.zeros(2, 12))
        self.assertTrue(self.drive.pid_guard[0].item())
        self.assertFalse(self.drive.pid_guard[1].item())
        self.assertEqual(self.drive.motor_effort[0].abs().max().item(), 0.0)
        self.drive.reset(torch.tensor([0]))
        self.assertFalse(self.drive.pid_guard[0].item())

    def test_calibrated_sides_and_coupled_shin_drive_direction(self):
        goal = torch.zeros(2, 12)
        goal[:, [0, 3, 6, 9]] = 0.02
        goal[:, self.drive.thigh] = 0.02
        goal[:, self.drive.shin] = -0.02  # linked shin motor target remains fixed
        action = types.SimpleNamespace(joint_positions=goal, joint_velocities=torch.zeros(2, 12),
                                       joint_efforts=torch.zeros(2, 12))
        self.drive.compute(action, torch.zeros(2, 12), torch.zeros(2, 12))
        self.assertTrue(torch.all(self.drive.motor_effort[:, [0, 3, 6, 9]] > 0))
        self.assertTrue(torch.all(self.drive.motor_effort[:, self.drive.thigh] > 0))
        torch.testing.assert_close(self.drive.motor_effort[:, self.drive.shin], torch.zeros(2, 4))


class FirmwarePidTests(unittest.TestCase):
    def test_integral_limit_antiwindup_unwind_and_first_derivative(self):
        def step(goal, measured, previous, integral, initialized=True):
            x = lambda v: torch.tensor([[v]], dtype=torch.float32)
            return firmware_pid_step(x(goal), x(measured), x(previous), x(integral),
                                     torch.tensor([[initialized]]), .02)
        pwm, integral, _ = step(2010, 2000, 1900, 0, False)
        self.assertEqual(pwm.item(), 71)
        self.assertAlmostEqual(integral.item(), .6, places=5)
        _, integral, _ = step(4000, 2000, 2000, 10)
        self.assertEqual(integral.item(), 10)  # stop integrating into saturation
        _, integral, _ = step(2010, 2000, 2000, 99.9)
        self.assertEqual(integral.item(), 100)
        _, integral, _ = step(1990, 2000, 2000, 100)
        self.assertLess(integral.item(), 100)


if __name__ == '__main__':
    unittest.main()
