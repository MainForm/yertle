"""Numerical checks for progressively increasing velocity error penalties."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import torch


source = Path(__file__).resolve().parents[1] / "flat_env_cfg_reward.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
names = {"_command_relative_cauchy", "track_lin_vel_xy_cauchy", "track_ang_vel_z_cauchy",
         "lin_vel_xy_tracking_error_cauchy", "ang_vel_z_tracking_error_cauchy"}
functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
scope = {"torch": torch}
exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"), scope)


class VelocityTrackingPenaltyTests(unittest.TestCase):
    def test_zero_and_progressive_error(self):
        commands = torch.tensor([[0.0, 0.0, 0.0]] * 3)
        linear = torch.tensor([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]])
        angular = torch.tensor([[0.0, 0.0, 0.0], [0.0, 0.0, 0.2], [0.0, 0.0, 0.4]])
        robot = SimpleNamespace(data=SimpleNamespace(root_lin_vel_b=linear, root_ang_vel_b=angular))
        env = SimpleNamespace(scene={"robot": robot}, command_manager=SimpleNamespace(get_command=lambda _: commands))
        torch.testing.assert_close(scope["lin_vel_xy_tracking_error_cauchy"](env), torch.tensor([0.0, 0.4, 8.0 / 17.0]))
        torch.testing.assert_close(scope["ang_vel_z_tracking_error_cauchy"](env), torch.tensor([0.0, 0.8, 16.0 / 17.0]))
        torch.testing.assert_close(scope["lin_vel_xy_tracking_error_cauchy"](env) + scope["track_lin_vel_xy_cauchy"](env), torch.ones(3))

    def test_selected_linear_axis(self):
        robot = SimpleNamespace(data=SimpleNamespace(root_lin_vel_b=torch.tensor([[0.2, 0.9, 0.0]])))
        env = SimpleNamespace(scene={"robot": robot}, command_manager=SimpleNamespace(get_command=lambda _: torch.zeros(1, 3)))
        torch.testing.assert_close(scope["lin_vel_xy_tracking_error_cauchy"](env, components=(0,)), torch.tensor([16.0 / 17.0]))
        with self.assertRaises(ValueError):
            scope["lin_vel_xy_tracking_error_cauchy"](env, components=(0, 0))


if __name__ == "__main__":
    unittest.main()
