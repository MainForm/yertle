"""No Isaac imports or real robot access required."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tripod_visual_pose import foot_body, make_half_body_pose


class VisualPoseTests(unittest.TestCase):
    def setUp(self):
        self.names = [p + "_" + k for k in ("shoulder", "thigh", "shin") for p in ("lb", "lf", "rb", "rf")]
        self.source = {"joint_names": self.names,
                       "angles_rad": [0.] * 4 + [math.radians(34)] * 4 + [math.radians(-85)] * 4,
                       "body_shift_m": [-.012, .012, 0], "raw_positions_by_motor_id_1_to_12": [2048] * 12}

    def test_height_and_fixed_other_joints(self):
        result = make_half_body_pose(self.source)
        self.assertAlmostEqual(result["expected_feet_body_m"][0][2], -.5 * result["display_body_height_m"], places=8)
        for index, name in enumerate(self.names):
            if not name.startswith("rf_"):
                self.assertEqual(self.source["angles_rad"][index], result["angles_rad"][index])
        self.assertNotIn("raw_positions_by_motor_id_1_to_12", result)
        self.assertTrue(all(abs(x) < math.pi for x in result["angles_rad"]))
        self.assertEqual(self.source["angles_rad"][7], math.radians(34))

    def test_preserve_rf_xy(self):
        result = make_half_body_pose(self.source)
        before = foot_body("rf", [0, math.radians(34), math.radians(-85)])
        np.testing.assert_allclose(result["expected_feet_body_m"][0][:2], before[:2], atol=1e-8)

    def test_ratios(self):
        for ratio in (.05, .25, .5, .75):
            result = make_half_body_pose(self.source, ratio)
            self.assertAlmostEqual(result["expected_feet_body_m"][0][2],
                                   (ratio - 1) * result["display_body_height_m"], places=8)

    def test_invalid(self):
        for ratio in (0, 1, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                make_half_body_pose(self.source, ratio)
        self.source["joint_names"][0] = self.source["joint_names"][1]
        with self.assertRaises(ValueError):
            make_half_body_pose(self.source)


if __name__ == "__main__":
    unittest.main()
