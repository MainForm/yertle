"""Pure Python calibration tests; no Isaac, torch, or hardware required."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


path = Path(__file__).resolve().parents[1] / "pose_calibration.py"
spec = importlib.util.spec_from_file_location("pose_calibration", path)
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)


class CalibrationPoseTests(unittest.TestCase):
    def setUp(self):
        self.names = [f"{leg}_{kind}" for kind in ("shoulder", "thigh", "shin")
                      for leg in ("lb", "lf", "rb", "rf")]
        urdf = ET.parse(path.parents[1] / "simulation" / "yertle.urdf")
        hard_limits = {joint.get("name"): (float(joint.find("limit").get("lower")),
                                          float(joint.find("limit").get("upper")))
                       for joint in urdf.findall("joint") if joint.find("limit") is not None}
        self.defaults = [dict(shoulder=0, thigh=-0.6, shin=0.9)[name.rsplit("_", 1)[1]]
                         for name in self.names]
        self.pose = calibration.CalibrationPose(self.names, self.defaults,
                                                [hard_limits[name] for name in self.names])

    def test_motor_mapping_uses_names_not_fixed_simulator_order(self):
        motors = self.pose.as_dict()["motors"]
        self.assertEqual([m["simulation_joint_index"] for m in motors],
                         [7, 11, 3, 5, 9, 1, 6, 10, 2, 4, 8, 0])
        self.assertTrue(all(m["raw_reference"] is None and m["raw_direction_sign"] is None for m in motors))
        reverse = calibration.CalibrationPose(self.names[::-1], self.defaults[::-1], self.pose.limits[::-1])
        for motor in reverse.as_dict()["motors"]:
            self.assertEqual(reverse.joint_names[motor["simulation_joint_index"]], motor["joint"])

    def test_reversed_front_is_a_consistent_180_degree_robot_frame(self):
        pose = calibration.CalibrationPose(self.names, self.defaults, self.pose.limits,
                                            motor_frame="reverse-front")
        self.assertEqual([m["simulation_joint_index"] for m in pose.as_dict()["motors"]],
                         [4, 8, 0, 6, 10, 2, 5, 9, 1, 7, 11, 3])
        urdf = ET.parse(path.parents[1] / "simulation" / "yertle.urdf")
        origins = {joint.get("name"): list(map(float, joint.find("origin").get("xyz").split()))
                   for joint in urdf.findall("joint") if joint.get("name", "").endswith("_shoulder")}
        for leg, (front, right) in zip(calibration.MOTOR_FRAMES["reverse-front"],
                                      ((True, True), (True, False), (False, True), (False, False))):
            x, y, _ = origins[f"{leg}_shoulder"]
            self.assertEqual(x < 0, front)
            self.assertEqual(y > 0, right)

    def test_front_rear_only_mapping_preserves_original_left_right_labels(self):
        pose = calibration.CalibrationPose(self.names, self.defaults, self.pose.limits,
                                            motor_frame="swap-front-rear")
        self.assertEqual([m["simulation_joint_index"] for m in pose.as_dict()["motors"]],
                         [6, 10, 2, 4, 8, 0, 7, 11, 3, 5, 9, 1])

    def test_manual_range_180_is_separate_from_source_limits_and_round_trips(self):
        pose = calibration.CalibrationPose(self.names, self.defaults, self.pose.limits,
                                            manual_limit_degrees=180, motor_frame="reverse-front")
        self.assertEqual(pose.source_limits, self.pose.limits)
        for degrees in (-180, 180):
            for name in self.names:
                self.assertAlmostEqual(pose.set_degrees(name, degrees), degrees)
        self.assertEqual(self.pose.angles, self.defaults)
        with tempfile.TemporaryDirectory() as directory:
            saved = pose.save(directory)
            pose.reset()
            pose.load(saved)
            self.assertTrue(all(abs(q - math.pi) < 1e-6 for q in pose.angles))
            self.assertEqual(pose.as_dict()["joints"]["rf_thigh"]["source_usd_limits_rad"],
                             list(self.pose.limits[self.pose.indices["rf_thigh"]]))
            # A training-limit pose loader must still reject these angles.
            with self.assertRaises(ValueError):
                self.pose.load(saved)

    def test_legacy_pose_keeps_sim_angles_when_motor_frame_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            self.pose.set_degrees("rf_thigh", -60)
            data = self.pose.as_dict()
            del data["motor_frame"]  # Old schema-1 files did not record a frame.
            saved = Path(directory) / "legacy.json"
            saved.write_text(json.dumps(data), encoding="utf-8")
            pose = calibration.CalibrationPose(self.names, self.defaults, self.pose.limits,
                                                manual_limit_degrees=180, motor_frame="reverse-front")
            pose.load(saved)
            self.assertEqual(pose.angles, self.pose.angles)
            self.assertEqual(pose.loaded_motor_frame, "sim")
            self.assertEqual(pose.as_dict()["motors"][0]["joint"], "lb_thigh")

    def test_degrees_radians_and_joint_limits(self):
        for name in self.names:
            index = self.pose.indices[name]
            lower, upper = self.pose.limits[index]
            self.pose.set_degrees(name, 1e6)
            self.assertAlmostEqual(self.pose.angles[index], upper)
            self.pose.set_degrees(name, -1e6)
            self.assertAlmostEqual(self.pose.angles[index], lower)
        self.pose.set_degrees("rf_thigh", -60)
        self.assertAlmostEqual(self.pose.angles[self.pose.indices["rf_thigh"]], -math.pi / 3)

    def test_reset_restores_all_twelve_defaults(self):
        for name in self.names:
            self.pose.set_degrees(name, 30)
        self.pose.reset()
        self.assertEqual(self.pose.angles, self.defaults)

    def test_nonfinite_values_and_bad_mapping_are_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ValueError):
                self.pose.set_degrees("rf_thigh", value)
        with self.assertRaises(ValueError):
            calibration.CalibrationPose(["rf_thigh"] * 12, self.defaults, self.pose.limits)
        with self.assertRaises(ValueError):
            calibration.CalibrationPose(self.names, [float("nan")] * 12, self.pose.limits)

    def test_saved_pose_has_units_and_never_overwrites_previous_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            self.pose.set_degrees("rf_thigh", -60)
            first = self.pose.save(directory)
            old_content = first.read_text(encoding="utf-8")
            self.pose.reset()
            second = self.pose.save(directory)
            self.assertNotEqual(first, second)
            self.assertEqual(first.read_text(encoding="utf-8"), old_content)
            saved = json.loads(old_content)
            self.assertEqual(saved["angles_rad"], [saved["joints"][n]["angle_rad"] for n in self.names])
            self.assertAlmostEqual(saved["joints"]["rf_thigh"]["angle_deg"], -60)
            self.assertEqual(len(saved["motors"]), 12)

    def test_load_saved_pose_and_reset_does_not_change_default_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            self.pose.set_degrees("rf_thigh", -57.3)
            expected = self.pose.angles.copy()
            saved = self.pose.save(directory)
            self.pose.reset()
            self.pose.load(saved)
            self.assertEqual(self.pose.angles, expected)
            self.pose.reset()
            self.assertEqual(self.pose.angles, self.defaults)

    def test_load_maps_names_even_if_simulator_order_is_different(self):
        with tempfile.TemporaryDirectory() as directory:
            self.pose.set_degrees("rf_thigh", -57.3)
            saved = self.pose.save(directory)
            reverse = calibration.CalibrationPose(self.names[::-1], self.defaults[::-1], self.pose.limits[::-1])
            reverse.load(saved)
            self.assertEqual(reverse.angles, self.pose.angles[::-1])

    def assert_bad_pose_is_atomic(self, data):
        with tempfile.TemporaryDirectory() as directory:
            bad = Path(directory) / "bad.json"
            bad.write_text(json.dumps(data), encoding="utf-8")
            before = self.pose.angles.copy()
            with self.assertRaises(ValueError):
                self.pose.load(bad)
            self.assertEqual(self.pose.angles, before)

    def test_load_rejects_missing_duplicate_and_unknown_joints(self):
        for name_change in (lambda names: names[:-1],
                            lambda names: [names[0]] * 12,
                            lambda names: names[:-1] + ["unknown_joint"]):
            data = self.pose.as_dict()
            data["joint_names"] = name_change(data["joint_names"])
            self.assert_bad_pose_is_atomic(data)

    def test_load_rejects_out_of_range_nonfinite_and_non_numeric_angles(self):
        for angle in (1e6, float("nan"), float("inf"), "0.0", True):
            data = self.pose.as_dict()
            data["angles_rad"][0] = angle
            self.assert_bad_pose_is_atomic(data)

    def test_load_rejects_inconsistent_degrees_and_named_angles(self):
        data = self.pose.as_dict()
        data["angles_deg"][0] += 10
        self.assert_bad_pose_is_atomic(data)
        data = self.pose.as_dict()
        data["joints"][self.names[0]]["angle_rad"] += 0.1
        self.assert_bad_pose_is_atomic(data)

    def test_load_rejects_unsupported_schema_and_missing_file(self):
        for version in (None, 2, True, "1"):
            data = self.pose.as_dict()
            data["schema_version"] = version
            self.assert_bad_pose_is_atomic(data)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(OSError):
                self.pose.load(Path(directory) / "does_not_exist.json")

    def test_load_existing_user_seated_pose(self):
        user_pose = path.parent / "runs" / "pose_calibration" / "pose_20261006_101423_781259Z.json"
        if not user_pose.exists():
            self.skipTest("Local user calibration artifact is not part of the repository")
        self.pose.load(user_pose)
        for name, angle in zip(self.names, self.pose.angles):
            expected = dict(shoulder=0, thigh=-57.3, shin=80.2)[name.rsplit("_", 1)[1]]
            self.assertAlmostEqual(math.degrees(angle), expected, places=4)


if __name__ == "__main__":
    unittest.main()
