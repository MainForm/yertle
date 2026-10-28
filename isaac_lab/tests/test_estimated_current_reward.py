"""Check aggregate current threshold and ideal linked-joint virtual work."""
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch
import sys
import unittest
import ast
import torch

managers = ModuleType('isaaclab.managers')
managers.SceneEntityCfg = lambda name: NS(name=name)
path = Path(__file__).resolve().parents[1] / 'estimated_current_reward.py'
spec = importlib.util.spec_from_file_location('estimated_current_reward_test', path)
mod = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'isaaclab.managers': managers}):
    spec.loader.exec_module(mod)


class EstimatedCurrentTests(unittest.TestCase):
    def make_env(self, values):
        names = [f'{leg}_{kind}' for leg in mod.LEGS for kind in ('shoulder', 'thigh', 'shin')]
        return NS(scene={'robot': NS(joint_names=names, data=NS(applied_torque=torch.tensor(values, dtype=torch.float64)))})

    def test_threshold_and_quadratic_excess(self):
        env = self.make_env([[7,0,0] + [0]*9, [8,0,0] + [0]*9, [9,0,0] + [0]*9])
        torch.testing.assert_close(mod.estimated_total_current_excess(env, 1.0), torch.tensor([0.,1.,4.], dtype=torch.float64))

    def test_linked_thigh_shin_motor_torques(self):
        env = self.make_env([[0,2,1] + [0]*9, [0,-2,-1] + [0]*9])
        torch.testing.assert_close(mod.estimated_motor_current_sum(env, 2.0), torch.tensor([4.,4.], dtype=torch.float64))

    def test_invalid_model_or_scale_rejected(self):
        env = self.make_env([[0]*12])
        for kwargs in ({'amp_per_nm':0}, {'amp_per_nm':1,'scale_a':0}, {'amp_per_nm':1,'threshold_a':-1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                mod.estimated_total_current_excess(env, **kwargs)

    def test_registration_parameters(self):
        tree = ast.parse((path.parent / 'flat_env_cfg_reward.py').read_text(encoding='utf8'))
        configure = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'configure_rewards')
        assignment = next(n for n in configure.body if isinstance(n, ast.Assign) and
                          any(isinstance(t, ast.Attribute) and t.attr == 'estimated_total_current_excess' for t in n.targets))
        rewards = NS()
        ns = {'rewards': rewards, 'RewTerm': lambda **kw:NS(**kw), 'SceneEntityCfg': managers.SceneEntityCfg,
              'estimated_total_current_excess': mod.estimated_total_current_excess}
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),str(path),'exec'),ns)
        term = rewards.estimated_total_current_excess
        self.assertEqual(term.weight, -.1)
        self.assertEqual(term.params['threshold_a'], 7.)
        self.assertAlmostEqual(term.params['amp_per_nm'], 2.5/(19.5*.0980665))

if __name__ == '__main__':
    unittest.main()
