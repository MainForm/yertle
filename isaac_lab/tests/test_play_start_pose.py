"""JSON validation and real-tensor reset regression without launching Isaac Sim."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import torch


spec = importlib.util.spec_from_file_location(
    "play_start_pose", Path(__file__).resolve().parents[1] / "play_start_pose.py"
)
pose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose)


class PlayStartPoseTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "pose.json"
        self.names = sorted(pose.JOINT_NAMES)
        self.angles = {name: 0.2 if name.endswith("shoulder") else 0.5 for name in self.names}

    def write_json(self, **changes):
        data = {"schema_version": 1, "joint_names": self.names[::-1],
                "angles_rad": [self.angles[name] for name in self.names[::-1]],
                "motor_frame": "reverse-front"}
        data.update(changes)
        self.path.write_text(json.dumps(data), encoding="utf-8")
        return self.path

    def make_env(self):
        defaults = torch.zeros(3, 12)
        positions = torch.full_like(defaults, -0.8)
        velocities = torch.ones_like(defaults)
        robot = SimpleNamespace(
            joint_names=self.names,
            data=SimpleNamespace(default_joint_pos=defaults, joint_pos=positions,
                                 joint_vel=velocities,
                                 joint_pos_limits=torch.tensor([-1.0, 1.0]).repeat(3, 12, 1)),
            set_joint_position_target=Mock(), set_joint_velocity_target=Mock(),
        )
        def write(position, velocity, env_ids):
            positions[env_ids] = position
            velocities[env_ids] = velocity
        robot.write_joint_state_to_sim = Mock(side_effect=write)
        return SimpleNamespace(scene={"robot": robot}, num_envs=3, device="cpu"), robot

    def test_no_flags_preserve_legacy(self):
        self.assertEqual(pose.select_start_pose(), ("random", None))

    def test_file_implies_json_and_maps_by_name_not_order_or_motor_frame(self):
        self.assertEqual(pose.select_start_pose(pose_json=self.write_json()), ("json", self.angles))
        self.assertEqual(pose.select_start_pose("json", self.path), ("json", self.angles))

    def test_default_selection(self):
        self.assertEqual(pose.select_start_pose("default"), ("default", None))

    def test_conflicting_or_missing_flags(self):
        for mode, path in (("json", None), ("random", "file"), ("default", "file"), ("invalid", None)):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                pose.select_start_pose(mode, path)

    def test_bad_schema_joint_names_and_values(self):
        cases = ({"schema_version": True}, {"joint_names": self.names[:-1] + [self.names[0]]},
                 {"angles_rad": [0.0] * 11}, {"angles_rad": [float("nan")] * 12},
                 {"angles_rad": [float("inf")] * 12}, {"angles_rad": [True] * 12},
                 {"angles_deg": [0.0] * 12}, {"joints": {}})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                pose.load_pose_json(self.write_json(**changes))

    def test_redundant_editor_fields(self):
        data = {"angles_deg": [math.degrees(self.angles[name]) for name in self.names[::-1]],
                "joints": {name: {"angle_rad": q} for name, q in self.angles.items()}}
        self.assertEqual(pose.load_pose_json(self.write_json(**data)), self.angles)
        data["joints"][self.names[0]]["angle_rad"] = 99.0
        with self.assertRaises(ValueError):
            pose.load_pose_json(self.write_json(**data))

    def test_random_does_not_modify_event(self):
        marker = object()
        cfg = SimpleNamespace(events=SimpleNamespace(reset_robot_joints=marker))
        pose.configure_start_pose(cfg, "random", None)
        self.assertIs(cfg.events.reset_robot_joints, marker)

    def test_selected_event_keeps_asset_and_mode(self):
        asset = SimpleNamespace(name="robot")
        event = SimpleNamespace(func=object(), mode="reset", params={"position_range": (0.8, 1.2), "asset_cfg": asset})
        cfg = SimpleNamespace(events=SimpleNamespace(reset_robot_joints=event))
        pose.configure_start_pose(cfg, "json", self.angles)
        self.assertIs(event.func, pose.reset_selected_pose)
        self.assertEqual(event.mode, "reset")
        self.assertEqual(event.params, {"joint_angles": self.angles, "asset_cfg": asset})

    def test_json_subset_reset_and_repeated_reset_preserve_defaults_and_limits(self):
        env, robot = self.make_env()
        original_defaults = robot.data.default_joint_pos.clone()
        original_limits = robot.data.joint_pos_limits.clone()
        untouched = robot.data.joint_pos[0].clone()
        ids = torch.tensor([1, 2])
        for _ in range(2):
            pose.reset_selected_pose(env, ids, self.angles)
            expected = torch.tensor([self.angles[name] for name in self.names]).repeat(2, 1)
            torch.testing.assert_close(robot.data.joint_pos[ids], expected)
            torch.testing.assert_close(robot.data.joint_vel[ids], torch.zeros(2, 12))
            torch.testing.assert_close(robot.data.joint_pos[0], untouched)
            torch.testing.assert_close(robot.data.default_joint_pos, original_defaults)
            torch.testing.assert_close(robot.data.joint_pos_limits, original_limits)
            torch.testing.assert_close(robot.set_joint_position_target.call_args.args[0], expected)
            robot.data.joint_pos[ids] = -0.3

    def test_default_all_envs_and_empty_reset(self):
        env, robot = self.make_env()
        pose.reset_selected_pose(env, None)
        torch.testing.assert_close(robot.data.joint_pos, robot.data.default_joint_pos)
        torch.testing.assert_close(robot.data.joint_vel, torch.zeros(3, 12))
        robot.write_joint_state_to_sim.reset_mock()
        pose.reset_selected_pose(env, [])
        robot.write_joint_state_to_sim.assert_not_called()

    def test_out_of_limit_pose_rejected_before_any_write(self):
        env, robot = self.make_env()
        self.angles[self.names[0]] = 2.0
        with self.assertRaisesRegex(ValueError, "outside PhysX limits"):
            pose.reset_selected_pose(env, [0, 2], self.angles)
        robot.write_joint_state_to_sim.assert_not_called()
        robot.set_joint_position_target.assert_not_called()

    def test_nan_runtime_and_mismatched_robot_rejected(self):
        env, robot = self.make_env()
        self.angles[self.names[0]] = float("nan")
        with self.assertRaises(ValueError):
            pose.reset_selected_pose(env, [0], self.angles)
        robot.joint_names[0] = "other"
        with self.assertRaises(ValueError):
            pose.reset_selected_pose(env, [0], self.angles)
        robot.write_joint_state_to_sim.assert_not_called()

    def test_extension_is_explicit_and_requires_json(self):
        event = SimpleNamespace(func=object(), params={})
        cfg = SimpleNamespace(events=SimpleNamespace(reset_robot_joints=event))
        for mode in ("random", "default"):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                pose.configure_start_pose(cfg, mode, None, extend_limits=True)
        pose.configure_start_pose(cfg, "json", self.angles, extend_limits=True)
        self.assertTrue(event.params["extend_limits"])

    def test_extension_updates_only_selected_limits_and_preserves_defaults(self):
        env, robot = self.make_env()
        original = robot.data.joint_pos_limits.clone()
        defaults = robot.data.default_joint_pos.clone()
        self.angles[self.names[0]] = 2.0
        self.angles[self.names[1]] = -2.0
        def write_limits(limits, env_ids):
            robot.data.joint_pos_limits[env_ids] = limits
        robot.write_joint_position_limit_to_sim = Mock(side_effect=write_limits)
        for _ in range(2):
            pose.reset_selected_pose(env, [1, 2], self.angles, extend_limits=True)
        expected = original.clone()
        expected[1:, 0, 1] = 2.0
        expected[1:, 1, 0] = -2.0
        torch.testing.assert_close(robot.data.joint_pos_limits, expected)
        torch.testing.assert_close(robot.data.default_joint_pos, defaults)
        self.assertEqual(robot.write_joint_position_limit_to_sim.call_count, 1)

    def test_extension_rejects_nonfinite_and_beyond_180_before_writes(self):
        for value in (float("nan"), float("inf"), math.pi + 0.01):
            env, robot = self.make_env()
            robot.write_joint_position_limit_to_sim = Mock()
            self.angles[self.names[0]] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                pose.reset_selected_pose(env, [0], self.angles, extend_limits=True)
            robot.write_joint_position_limit_to_sim.assert_not_called()
            robot.write_joint_state_to_sim.assert_not_called()


if __name__ == "__main__":
    unittest.main()
