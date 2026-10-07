"""Pose-pool reset regression with real tensors, without Kit."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import torch

spec = importlib.util.spec_from_file_location("train_start_pose", Path(__file__).resolve().parents[1] / "train_start_pose.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TrainStartPoseTests(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(42)
        self.names = [f"{leg}_{kind}" for leg in ("lf", "rf", "lb", "rb") for kind in ("shoulder", "thigh", "shin")]
        self.pose = {n: 2.7 if n.endswith("shin") else 0.0 for n in self.names}
        self.defaults = torch.tensor([0., -.6, .9]).repeat(128, 4)
        self.limits = torch.tensor([-1.6, 1.6]).repeat(128, 12, 1)
        self.robot = SimpleNamespace(joint_names=self.names,
            data=SimpleNamespace(default_joint_pos=self.defaults.clone(), joint_pos_limits=self.limits.clone()),
            write_joint_state_to_sim=Mock(), set_joint_position_target=Mock(), set_joint_velocity_target=Mock())
        def expand(limits):
            self.robot.data.joint_pos_limits[:] = limits
        self.robot.write_joint_position_limit_to_sim = Mock(side_effect=expand)
        self.env = SimpleNamespace(scene={"robot": self.robot}, num_envs=128, device="cpu")

    def test_uniform_coherent_poses_and_unchanged_offsets(self):
        module.reset_pose_pool(self.env, None, [self.pose], True)
        selection = self.env.reset_pose_ids
        self.assertGreater((selection == 0).sum().item(), 35)
        self.assertGreater((selection == 1).sum().item(), 35)
        q, dq = self.robot.write_joint_state_to_sim.call_args.args
        torch.testing.assert_close(q[selection == 0], self.defaults[selection == 0])
        desired = torch.tensor([self.pose[n] for n in self.names])
        torch.testing.assert_close(q[selection == 1], desired.expand((selection == 1).sum(), -1))
        torch.testing.assert_close(dq, torch.zeros_like(q))
        torch.testing.assert_close(self.robot.data.default_joint_pos, self.defaults)

    def test_subset_reset_extends_all_envs_once(self):
        module.reset_pose_pool(self.env, [1, 4], [self.pose], True)
        self.assertEqual(self.env.reset_pose_ids[0].item(), -1)
        self.assertEqual(self.robot.data.joint_pos_limits[0, 2, 1].item(), torch.tensor(2.7).item())
        module.reset_pose_pool(self.env, [4], [self.pose], True)
        self.robot.write_joint_position_limit_to_sim.assert_called_once()

    def test_invalid_pool_rejected_before_state_or_limit_write(self):
        for poses, extend in (([self.pose], False), ([dict(self.pose, lf_shin=float("nan"))], True),
                              ([dict(self.pose, lf_shin=4.0)], True), ([{}], True)):
            with self.subTest(poses=poses, extend=extend), self.assertRaises(ValueError):
                module.reset_pose_pool(self.env, [0], poses, extend)
        self.robot.write_joint_state_to_sim.assert_not_called()
        self.robot.write_joint_position_limit_to_sim.assert_not_called()

    def test_empty_ids_noop(self):
        module.reset_pose_pool(self.env, [], [self.pose], True)
        self.robot.write_joint_state_to_sim.assert_not_called()

    def test_configuration_and_legacy_preservation(self):
        event = SimpleNamespace(func=object(), params={"position_range": (.8, 1.2), "asset_cfg": object()})
        cfg = SimpleNamespace(events=SimpleNamespace(reset_robot_joints=event))
        old = event.func
        module.configure_reset_pose_pool(cfg, [])
        self.assertIs(event.func, old)
        with self.assertRaises(ValueError):
            module.configure_reset_pose_pool(cfg, [], True)
        asset = event.params["asset_cfg"]
        module.configure_reset_pose_pool(cfg, [self.pose], True)
        self.assertIs(event.func, module.reset_pose_pool)
        self.assertIs(event.params["asset_cfg"], asset)
        self.assertEqual(event.params["poses"], [self.pose])


if __name__ == "__main__":
    unittest.main()
