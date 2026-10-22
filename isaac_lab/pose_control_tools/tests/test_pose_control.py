"""Pure tests: no Isaac Sim import, model mutation, COM connection or hardware."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

ISAAC_LAB = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("pose_control", ISAAC_LAB / "pose_control.py")
pose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose)
loader_spec = importlib.util.spec_from_file_location("play_start_pose", ISAAC_LAB / "play_start_pose.py")
loader = importlib.util.module_from_spec(loader_spec)
loader_spec.loader.exec_module(loader)


class PoseControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = pose.load_targets()

    def test_complete_exact_trajectories(self):
        for name, count in (("tripod-low", 651), ("tripod-high", 951)):
            frames = self.data["trajectories"][name]["frames"]
            self.assertEqual(len(frames), count)
            self.assertEqual(frames[0]["angles_rad"], frames[-1]["angles_rad"])

    def test_every_frame_selected_once(self):
        for trajectory in self.data["trajectories"].values():
            actual = [pose.select_frame(trajectory, 3 + i*.02)[0]
                      for i in range(len(trajectory["frames"]))]
            self.assertEqual(actual, list(range(len(trajectory["frames"]))))

    def test_settle_and_tail(self):
        trajectory = self.data["trajectories"]["tripod-high"]
        self.assertEqual(pose.select_frame(trajectory, 2.99), (0, "settle", False))
        self.assertEqual(pose.select_frame(trajectory, 3)[0], 0)
        self.assertFalse(pose.select_frame(trajectory, 23.99)[2])
        self.assertEqual(pose.select_frame(trajectory, 24)[0], 950)
        self.assertTrue(pose.select_frame(trajectory, 24)[2])

    def test_invalid_angles_rejected(self):
        for values in ([0]*11, [0]*13, [float("nan")]*12,
                       [float("inf")]*12, [math.pi+.01]*12, [True]*12):
            with self.subTest(values=values), self.assertRaises(ValueError):
                pose.validate_angles(values)

    def test_corrupt_data_rejected(self):
        data = copy.deepcopy(self.data)
        data["joint_names"][0] = data["joint_names"][1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"corrupt.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(ValueError):
                pose.load_targets(path)

    def test_save_schema_compatible_with_committed_play_loader(self):
        names = self.data["joint_names"]
        angles = self.data["presets"]["standing"]["angles_rad"]
        data = {"schema_version": 1, "motor_frame": "sim", "joint_names": names,
                "angles_rad": angles, "angles_deg": [math.degrees(q) for q in angles]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"pose.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(loader.load_pose_json(path), dict(zip(names, angles)))

    def test_seated_reference(self):
        for name, angle in zip(self.data["joint_names"], self.data["presets"]["seated"]["angles_rad"]):
            degrees = 0 if name.endswith("shoulder") else 75 if name.endswith("thigh") else -160
            self.assertAlmostEqual(math.degrees(angle), degrees)


if __name__ == "__main__":
    unittest.main()
