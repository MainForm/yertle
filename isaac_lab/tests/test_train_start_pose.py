"""Pose-pool reset regression with real tensors, without Kit."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import torch

spec = importlib.util.spec_from_file_location("train_start_pose", Path(__file__).resolve().parents[1] / "train_start_pose.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

pose_spec = importlib.util.spec_from_file_location("play_start_pose", Path(__file__).resolve().parents[1] / "play_start_pose.py")
pose_module = importlib.util.module_from_spec(pose_spec)
pose_spec.loader.exec_module(pose_module)


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

    def test_api_reference_profile_angles_and_sampled_states(self):
        path = Path(__file__).resolve().parents[1] / "configs" / "seated_api_reference.json"
        pose = pose_module.load_pose_json(path)
        for name, angle in pose.items():
            degrees = 0 if name.endswith("shoulder") else 75 if name.endswith("thigh") else -160
            self.assertAlmostEqual(angle, math.radians(degrees))
        module.reset_pose_pool(self.env, None, [pose], True)
        selection = self.env.reset_pose_ids
        self.assertGreater((selection == 0).sum().item(), 35)
        self.assertGreater((selection == 1).sum().item(), 35)
        q, dq = self.robot.write_joint_state_to_sim.call_args.args
        desired = torch.tensor([pose[n] for n in self.names])
        torch.testing.assert_close(q[selection == 1], desired.expand((selection == 1).sum(), -1))
        torch.testing.assert_close(q[selection == 0], self.defaults[selection == 0])
        torch.testing.assert_close(dq, torch.zeros_like(q))
        torch.testing.assert_close(self.robot.data.default_joint_pos, self.defaults)
        self.assertTrue((q >= self.robot.data.joint_pos_limits[..., 0]).all())
        self.assertTrue((q <= self.robot.data.joint_pos_limits[..., 1]).all())

    def test_api_reference_requires_explicit_limit_expansion(self):
        path = Path(__file__).resolve().parents[1] / "configs" / "seated_api_reference.json"
        pose = pose_module.load_pose_json(path)
        with self.assertRaises(ValueError):
            module.reset_pose_pool(self.env, None, [pose], False)
        self.robot.write_joint_state_to_sim.assert_not_called()
        self.robot.write_joint_position_limit_to_sim.assert_not_called()

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

    def test_requested_standing_and_seated_pool_without_old_standing(self):
        directory = Path(__file__).resolve().parents[1] / "configs"
        standing = pose_module.load_pose_json(directory / "standing_api_reference.json")
        seated = pose_module.load_pose_json(directory / "seated_api_reference.json")
        for name, angle in standing.items():
            expected = 0 if name.endswith("shoulder") else 34 if name.endswith("thigh") else -85
            self.assertAlmostEqual(angle, math.radians(expected))
        module.reset_pose_pool(self.env, None, [seated], True, standing_pose=standing)
        selected = self.env.reset_pose_ids
        q, dq = self.robot.write_joint_state_to_sim.call_args.args
        for i, candidate in enumerate((standing, seated)):
            self.assertGreater((selected == i).sum().item(), 35)
            desired = torch.tensor([candidate[n] for n in self.names])
            torch.testing.assert_close(q[selected == i], desired.expand((selected == i).sum(), -1))
        torch.testing.assert_close(self.robot.data.default_joint_pos, self.defaults)
        torch.testing.assert_close(dq, torch.zeros_like(q))

    def test_invalid_standing_override_rejected_before_writes(self):
        for standing in ({}, dict(self.pose, lf_shin=float("nan")), dict(self.pose, lf_shin=4)):
            with self.assertRaises(ValueError):
                module.reset_pose_pool(self.env, [0], [self.pose], True, standing_pose=standing)
        self.robot.write_joint_state_to_sim.assert_not_called()
        self.robot.write_joint_position_limit_to_sim.assert_not_called()

    def test_standing_only_override(self):
        module.reset_pose_pool(self.env, None, [], True, standing_pose=self.pose)
        self.assertTrue((self.env.reset_pose_ids == 0).all())
        q, _ = self.robot.write_joint_state_to_sim.call_args.args
        torch.testing.assert_close(q, torch.tensor([self.pose[n] for n in self.names]).expand(128, -1))
        torch.testing.assert_close(self.robot.data.default_joint_pos, self.defaults)


if __name__ == "__main__":
    unittest.main()
