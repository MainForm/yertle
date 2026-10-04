"""Exercise trot commands and phases with PyTorch, without starting Isaac Sim.

Run from the repository root: python -m unittest discover -s isaac_lab/tests
"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import torch


def load_module(filename, dependency, symbol):
    """Isolate the Isaac Lab import; the tested tensor operations use real torch."""
    dependency_stub = ModuleType(dependency)
    setattr(dependency_stub, symbol, object)
    path = Path(__file__).resolve().parents[1] / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {dependency: dependency_stub}):
        spec.loader.exec_module(module)
    return module


scheduler_module = load_module(
    "gait_scheduler.py",
    "isaaclab.envs.mdp.commands.velocity_command",
    "UniformVelocityCommand",
)
phase = load_module("phase_generator.py", "isaaclab.managers", "SceneEntityCfg")


class FixedTrotTests(unittest.TestCase):
    def make_scheduler(self, ranges, count=4096):
        scheduler = scheduler_module.GaitScheduler.__new__(scheduler_module.GaitScheduler)
        scheduler.cfg = SimpleNamespace(ranges=SimpleNamespace(**ranges))
        scheduler.num_envs = count
        scheduler.device = "cpu"
        scheduler.vel_command_b = torch.full((count, 3), 99.0)
        scheduler.sampled_gait_mode = torch.zeros(count, dtype=torch.long)
        scheduler.is_heading_env = torch.ones(count, dtype=torch.bool)
        scheduler.is_standing_env = torch.ones(count, dtype=torch.bool)
        scheduler.heading_target = torch.ones(count)
        return scheduler

    def test_full_range_includes_low_speed_and_straight_fast_commands(self):
        torch.manual_seed(0)
        ranges = {"lin_vel_x": (-0.5, 0.3), "lin_vel_y": (-0.2, 0.2), "ang_vel_z": (-1.0, 1.0)}
        scheduler = self.make_scheduler(ranges)
        scheduler._resample_command(range(scheduler.num_envs))
        command = scheduler.vel_command_b
        for column, bounds in enumerate(ranges.values()):
            self.assertTrue(torch.all((command[:, column] >= bounds[0]) & (command[:, column] <= bounds[1])))
        self.assertTrue(torch.any(torch.linalg.vector_norm(command[:, :2], dim=1) < 0.12))
        self.assertTrue(torch.any((command[:, 0].abs() > 0.24) & (command[:, 1].abs() < 0.06) & (command[:, 2].abs() < 0.25)))
        self.assertTrue(torch.all(scheduler.sampled_gait_mode == 1))
        self.assertFalse(scheduler.is_heading_env.any())
        self.assertFalse(scheduler.is_standing_env.any())

    def test_subset_reset_and_zero_width_ranges(self):
        ranges = {"lin_vel_x": (0.0, 0.0), "lin_vel_y": (0.0, 0.0), "ang_vel_z": (0.0, 0.0)}
        scheduler = self.make_scheduler(ranges, count=4)
        scheduler._resample_command([1, 3])
        torch.testing.assert_close(scheduler.vel_command_b[[1, 3]], torch.zeros(2, 3))
        torch.testing.assert_close(scheduler.vel_command_b[[0, 2]], torch.full((2, 3), 99.0))
        before = scheduler.vel_command_b.clone()
        scheduler._resample_command([])
        torch.testing.assert_close(scheduler.vel_command_b, before)

    def test_narrow_vx_override_is_respected(self):
        ranges = {"lin_vel_x": (-0.03, -0.01), "lin_vel_y": (0.0, 0.0), "ang_vel_z": (0.0, 0.0)}
        scheduler = self.make_scheduler(ranges)
        scheduler._resample_command(range(scheduler.num_envs))
        self.assertTrue(torch.all((scheduler.vel_command_b[:, 0] >= -0.03) & (scheduler.vel_command_b[:, 0] <= -0.01)))
        torch.testing.assert_close(scheduler.vel_command_b[:, 1:], torch.zeros(scheduler.num_envs, 2))

    def test_phase_is_fixed_trot_for_every_command(self):
        commands = torch.tensor([[0, 0, 0], [0.01, 0, 0], [-0.5, 0, 0], [0.3, 0, 0], [0, 0.2, 0], [0, 0, 1.0]])
        env = SimpleNamespace(
            num_envs=len(commands), step_dt=0.02,
            episode_length_buf=torch.zeros(len(commands), dtype=torch.long),
            command_manager=SimpleNamespace(get_command=lambda name: commands),
        )
        _, modes, frequencies, duties, _ = phase._gait_state(env, "base_velocity")
        self.assertTrue(torch.all(modes == 1))
        torch.testing.assert_close(frequencies, torch.full((len(commands),), 2.2))
        torch.testing.assert_close(duties, torch.full((len(commands),), 0.5))
        expected = torch.tensor([[True, False, False, True]]).repeat(len(commands), 1)
        torch.testing.assert_close(phase._desired_contact(env, "base_velocity"), expected)
        env.episode_length_buf.fill_(12)
        torch.testing.assert_close(phase._desired_contact(env, "base_velocity"), ~expected)
        one_hot = phase.gait_mode_one_hot(env)
        torch.testing.assert_close(one_hot, torch.tensor([[0.0, 1.0, 0.0]]).repeat(len(commands), 1))
        self.assertEqual(one_hot.shape[1], 3)
        self.assertEqual(phase.phase_clock_sin_cos(env).shape[1], 2)
        self.assertEqual(phase.gait_frequency(env).shape[1], 1)
        self.assertEqual(phase.gait_duty_factor(env).shape[1], 1)


if __name__ == "__main__":
    unittest.main()
