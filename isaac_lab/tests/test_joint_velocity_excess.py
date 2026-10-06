"""Test production reward tensor math without launching Isaac Sim.

AST extraction avoids importing the Kit-dependent environment configuration;
the function body and registration below come directly from production source.
"""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import torch


SOURCE = Path(__file__).resolve().parents[1] / "flat_env_cfg_reward.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


def entity(name, joint_ids=slice(None), **kwargs):
    return SimpleNamespace(name=name, joint_ids=joint_ids, **kwargs)


FUNCTION = next(
    node for node in TREE.body
    if isinstance(node, ast.FunctionDef) and node.name == "joint_velocity_excess"
)
NAMESPACE = {"torch": torch, "SceneEntityCfg": entity}
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(SOURCE), "exec"), NAMESPACE)
reward = NAMESPACE["joint_velocity_excess"]


class JointVelocityExcessTests(unittest.TestCase):
    def evaluate(self, velocities, joint_ids=slice(None), limit=2.30):
        tensor = torch.tensor(velocities, dtype=torch.float64)
        env = SimpleNamespace(scene={"robot": SimpleNamespace(data=SimpleNamespace(joint_vel=tensor))})
        return reward(env, limit, entity("robot", joint_ids))

    def test_below_and_at_threshold_are_zero(self):
        actual = self.evaluate([[0, 0.31, -0.31, 2.29, -2.29, 2.30, -2.30]])
        torch.testing.assert_close(actual, torch.zeros(1, dtype=torch.float64))

    def test_normalized_squared_excess_and_batched_sum(self):
        actual = self.evaluate([[2.53, -2.76, 0], [4.60, -4.60, 2.30]])
        torch.testing.assert_close(actual, torch.tensor([0.05, 2.0], dtype=torch.float64))

    def test_direction_symmetry(self):
        actual = self.evaluate([[3.0, 4.0], [-3.0, -4.0]])
        torch.testing.assert_close(actual[0], actual[1])

    def test_only_selected_joints_are_used(self):
        actual = self.evaluate([[4.60, 23.0, -4.60]], joint_ids=[0, 2])
        torch.testing.assert_close(actual, torch.tensor([2.0], dtype=torch.float64))

    def test_invalid_threshold_is_rejected(self):
        for limit in (0, -2.30):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                self.evaluate([[0]], limit=limit)

    def test_real_registration_uses_negative_weight_and_all_joints(self):
        configure = next(node for node in TREE.body if isinstance(node, ast.FunctionDef) and node.name == "configure_rewards")
        assignment = next(
            node for node in configure.body
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Attribute) and target.attr == "joint_velocity_excess" for target in node.targets)
        )
        rewards = SimpleNamespace()
        namespace = dict(NAMESPACE, rewards=rewards, RewTerm=lambda **kwargs: SimpleNamespace(**kwargs))
        exec(compile(ast.Module(body=[assignment], type_ignores=[]), str(SOURCE), "exec"), namespace)
        term = rewards.joint_velocity_excess
        self.assertIs(term.func, reward)
        self.assertEqual(term.weight, -0.10)
        self.assertEqual(term.params["velocity_limit_rad_s"], 2.30)
        self.assertEqual(term.params["asset_cfg"].joint_names, ".*")
        env = SimpleNamespace(scene={"robot": SimpleNamespace(data=SimpleNamespace(joint_vel=torch.tensor([[4.60]])))})
        weighted = term.func(env, **term.params) * term.weight
        torch.testing.assert_close(weighted, torch.tensor([-0.10]))


if __name__ == "__main__":
    unittest.main()
