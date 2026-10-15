"""Hardware-envelope regression tests; run without Isaac Sim or hardware."""

import ast
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import torch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("motor_operating_range", ROOT / "motor_operating_range.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
NAMES = [f"{leg}_{kind}" for kind in ("shoulder", "thigh", "shin") for leg in ("lb", "lf", "rb", "rf")]


def inverse_raw(raw):
    """Independent scalar inverse, matching the production C++ linkage API."""
    q = {}
    for leg_index, leg in enumerate(("rf", "lf", "rb", "lb")):
        delta = [(raw[3 * leg_index + k] - 2048) * module.MOTOR_SIGNS[3 * leg_index + k]
                 * 2 * math.pi / 4096 for k in range(3)]
        q[leg + "_shin"] = math.radians(-160) + delta[0] - delta[1]
        q[leg + "_thigh"] = math.radians(75) + delta[1]
        q[leg + "_shoulder"] = delta[2]
    return [q[name] for name in NAMES]


class OperatingRangeTests(unittest.TestCase):
    def setUp(self):
        self.cost = module.OperatingRangeCost(dtype=torch.float64)
        self.transform = module.MotorRawTransform(NAMES, dtype=torch.float64)

    def raw(self, values):
        return torch.tensor(values, dtype=torch.float64)

    def test_complete_csv_and_measurement_units(self):
        groups = module.load_operating_ranges()
        self.assertEqual(set(groups), module.EXPECTED_KEYS)
        self.assertEqual(sum(map(len, groups.values())), 54)
        self.assertEqual(groups[(1, 2)][0], (1940, 925, 1983))
        self.assertEqual(groups[(0, 3)], [(0, 1646, 2673)])

    def test_reference_and_random_cpp_inverse_roundtrip(self):
        torch.manual_seed(10)
        raw = torch.randint(0, 4096, (100, 12)).tolist()
        raw.insert(0, [2048] * 12)
        actual = self.transform(self.raw([inverse_raw(row) for row in raw]))
        torch.testing.assert_close(actual, self.raw(raw), rtol=0, atol=2e-12)

    def test_permuted_joint_order(self):
        q = self.raw([inverse_raw([2048] * 12)])
        order = list(reversed(range(12)))
        transform = module.MotorRawTransform([NAMES[i] for i in order], dtype=torch.float64)
        torch.testing.assert_close(transform(q[:, order]), self.transform(q))

    def test_shin_thigh_coupling_and_all_signs(self):
        base = self.raw([inverse_raw([2048] * 12)])
        for leg_index, leg in enumerate(("rf", "lf", "rb", "lb")):
            for kind in ("thigh", "shin", "shoulder"):
                q = base.clone()
                q[:, NAMES.index(leg + "_" + kind)] += .1
                actual = self.transform(q) - 2048
                expected = torch.zeros_like(actual)
                indices = {"shin": [0], "thigh": [0, 1], "shoulder": [2]}[kind]
                for k in indices:
                    i = 3 * leg_index + k
                    expected[:, i] = module.MOTOR_SIGNS[i] * .1 * 4096 / (2 * math.pi)
                torch.testing.assert_close(actual, expected, atol=1e-12, rtol=0)

    def test_safe_interior_has_zero_penalty(self):
        torch.testing.assert_close(self.cost(self.raw([[2048] * 12])), self.raw([0]))

    def test_table_knots_boundaries_and_interpolation(self):
        for (anchor, sweep), points in module.load_operating_ranges().items():
            for x, low, high in points:
                for value in (low, high):
                    raw = self.raw([[2048] * 12])
                    if anchor:
                        raw[:, anchor - 1] = x
                    raw[:, sweep - 1] = value
                    torch.testing.assert_close(self.cost.excess(raw), torch.zeros_like(raw))
        raw = self.raw([[2048] * 12])
        raw[:, 0], raw[:, 1] = 1990, (925 + 948) / 2 - 100
        self.assertEqual(self.cost.excess(raw)[0, 1].item(), 100)
        self.assertEqual(self.cost(raw).item(), 2)

    def test_unmeasured_anchor_domain_is_penalized_not_extrapolated(self):
        raw = self.raw([[2048] * 12])
        raw[:, 0], raw[:, 1] = 1840, 1500
        self.assertEqual(self.cost.excess(raw)[0, 0].item(), 100)
        self.assertEqual(self.cost(raw).item(), 2)

    def test_float32_roundtrip_exact_shoulder_endpoints_not_penalized(self):
        transform = module.MotorRawTransform(NAMES)
        cost = module.OperatingRangeCost()
        for key, points in module.load_operating_ranges().items():
            if key[0] != 0:
                continue
            for bound in points[0][1:]:
                raw = [2048] * 12
                raw[key[1] - 1] = bound
                q = torch.tensor([inverse_raw(raw)], dtype=torch.float32)
                self.assertEqual(cost(transform(q)).item(), 0)

    def test_target_and_measured_each_caught_without_double_counting(self):
        safe = self.raw([[2048] * 12])
        outside = safe.clone()
        outside[:, 2] = 2773  # ID3 safe_max=2673, excess=100.
        self.assertEqual(self.cost(outside, safe).item(), 2)
        self.assertEqual(self.cost(safe, outside).item(), 2)
        self.assertEqual(self.cost(outside, outside).item(), 2)
        outside[:, 2] = 2873
        self.assertEqual(self.cost(outside).item(), 5)

    def test_encoder_violation_is_not_modulo_wrapped(self):
        raw = self.raw([[2048] * 12, [2048] * 12])
        raw[0, 6], raw[1, 6] = 4403, -100
        self.assertGreaterEqual(self.cost.excess(raw)[0, 6].item(), 308)
        self.assertGreaterEqual(self.cost.excess(raw)[1, 6].item(), 100)

    def test_historical_actor_target_remains_rejected_after_leg_rotation(self):
        actions = self.raw([[-1.12617671, -1.62459838, .456345797, -2.0064013,
                             5.77863932, 3.54853559, .66210252, .541667819,
                             -3.69301653, 3.76520419, -.373643488, -5.88253975]])
        default = self.raw([[0] * 4 + [-.6] * 4 + [.9] * 4])
        raw = self.transform(default + .25 * actions)
        # Same lf target moves from old positive-sign ID7=4403 to new
        # negative-sign ID4=-307. Both remain encoder range violations.
        self.assertEqual(round(raw[0, 3].item()), -307)
        self.assertGreater(self.cost(raw).item(), 0)

    def test_native_front_asymmetric_joint_identity(self):
        base = self.raw([inverse_raw([2048] * 12)])
        for name, motor_id in (("rf_shin", 1), ("lf_shin", 4), ("rb_shin", 7), ("lb_shin", 10)):
            q = base.clone()
            q[:, NAMES.index(name)] += math.radians(10)
            raw = self.transform(q).round()
            expected = self.raw([[2048] * 12])
            expected[:, motor_id - 1] += 114 * module.MOTOR_SIGNS[motor_id - 1]
            torch.testing.assert_close(raw, expected)

    def test_invalid_names_shape_scale_and_speed_fail(self):
        for names in (NAMES[:-1], NAMES[:-1] + [NAMES[0]]):
            with self.assertRaises(ValueError):
                module.MotorRawTransform(names)
        with self.assertRaises(ValueError):
            self.transform(torch.zeros(1, 13))
        for scale in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                module.OperatingRangeCost(excess_scale_raw=scale)
        with self.assertRaises(ValueError):
            module.load_operating_ranges(command_speed_steps_per_s=1500)

    def test_bad_missing_duplicate_and_out_of_bounds_csv_fail(self):
        lines = module.RANGE_CSV.read_text().splitlines()
        cases = ["bad,header\n1,2", "\n".join(lines[:2]),
                 "\n".join(lines + [lines[1]]),
                 "\n".join([lines[0], lines[1].replace(",1646,2673,", ",-1,2673,")] + lines[2:])]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bad.csv"
            for content in cases:
                path.write_text(content)
                with self.assertRaises(ValueError):
                    module.load_operating_ranges(path)

    def test_actual_registration_and_class_with_manager_stubs(self):
        source = ROOT / "flat_env_cfg_reward.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        class Base:
            def __init__(self, cfg, env):
                self.cfg, self._env = cfg, env
        entity = lambda name: SimpleNamespace(name=name)
        namespace = dict(ManagerTermBase=Base, MotorRawTransform=module.MotorRawTransform,
                         OperatingRangeCost=module.OperatingRangeCost, SceneEntityCfg=entity)
        cls_node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MotorOperatingRangePenalty")
        exec(compile(ast.Module(body=[cls_node], type_ignores=[]), str(source), "exec"), namespace)
        configure = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "configure_rewards")
        assignment = next(n for n in configure.body if isinstance(n, ast.Assign)
                          and any(isinstance(t, ast.Attribute) and t.attr == "motor_operating_range_penalty" for t in n.targets))
        namespace.update(rewards=SimpleNamespace(), RewTerm=lambda **kw: SimpleNamespace(**kw))
        exec(compile(ast.Module(body=[assignment], type_ignores=[]), str(source), "exec"), namespace)
        cfg = namespace["rewards"].motor_operating_range_penalty
        self.assertEqual(cfg.weight, -100)
        q = self.raw([inverse_raw([2048] * 12)])
        # Action order intentionally differs from the asset joint order.
        order = list(reversed(range(12)))
        action = SimpleNamespace(_joint_names=[NAMES[i] for i in order], processed_actions=q[:, order].clone())
        asset = SimpleNamespace(joint_names=NAMES, data=SimpleNamespace(joint_pos=q))
        env = SimpleNamespace(scene={"robot": asset}, device="cpu",
                              action_manager=SimpleNamespace(get_term=lambda name: action))
        reward = cfg.func(cfg, env)
        # Actual q is legal; target shoulder ID3 alone exceeds upper bound by 100 raw.
        target = [2048] * 12
        target[2] = 2773
        action.processed_actions = self.raw([inverse_raw(target)])[:, order]
        torch.testing.assert_close(reward(env, **cfg.params) * cfg.weight, self.raw([-200]))


if __name__ == "__main__":
    unittest.main()
