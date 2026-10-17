"""Tensor/registration/CLI regression tests; no Kit, GPU or cloud required."""
import ast
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import torch

ROOT = Path(__file__).resolve().parents[1]
stub = ModuleType('isaaclab.managers')
stub.SceneEntityCfg = object
spec = importlib.util.spec_from_file_location('trot_support_test_phase', ROOT/'phase_generator.py')
phase = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'isaaclab.managers': stub}):
    spec.loader.exec_module(phase)
SENSOR = NS(name='contact_forces')


def environment(contacts, step=5, command=(.1, 0., 0.), stale_contact=False):
    # Deliberately shuffle the sensor body order, including a non-foot body.
    names = ['rb_shin', 'base_link', 'lf_shin', 'lb_shin', 'rf_shin']
    forces = torch.zeros(len(contacts), len(names), 3)
    for i, name in enumerate(phase._FOOT_NAMES):
        forces[:, names.index(name), 2] = torch.tensor([row[i] for row in contacts])*2.
    history = forces.unsqueeze(1).repeat(1, 3, 1, 1)
    if stale_contact:
        history[:, :, :, 2] = 2.
    sensor = NS(body_names=names, data=NS(net_forces_w=forces, net_forces_w_history=history),
                compute_first_contact=lambda dt: forces[:, :, 2]>0)
    return NS(scene=NS(sensors={'contact_forces': sensor}), num_envs=len(contacts),
              episode_length_buf=torch.full((len(contacts),), step), step_dt=.02,
              command_manager=NS(get_command=lambda name: torch.tensor([command]*len(contacts))))


class TrotSupportTests(unittest.TestCase):
    def test_support_missing_counts_correct_diagonal_wrong_diagonal_one_missing_and_flight(self):
        env = environment([[1,0,0,1], [0,1,1,0], [1,0,0,0], [0,0,0,0], [1,1,1,1]])
        torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env, SENSOR), torch.tensor([0.,1.,.5,1.,0.]))

    def test_other_half_cycle_uses_other_diagonal(self):
        env = environment([[0,1,1,0], [1,0,0,1]], step=16)
        torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env, SENSOR), torch.tensor([0.,1.]))

    def test_no_history_maximum_hides_current_airborne(self):
        env = environment([[0,0,0,0]], stale_contact=True)
        torch.testing.assert_close(phase.all_feet_airborne_penalty(env, SENSOR), torch.ones(1))
        torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env, SENSOR), torch.ones(1))

    def test_any_one_support_foot_stops_all_air_cost(self):
        env = environment([[0,0,0,0], [1,0,0,0], [0,0,0,1], [1,1,1,1]])
        torch.testing.assert_close(phase.all_feet_airborne_penalty(env, SENSOR), torch.tensor([1.,0.,0.,0.]))

    def test_all_airborne_still_penalized_at_zero_command(self):
        env = environment([[0,0,0,0]], command=(0.,0.,0.))
        torch.testing.assert_close(phase.all_feet_airborne_penalty(env, SENSOR), torch.ones(1))
        torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env, SENSOR), torch.zeros(1))

    def test_pure_yaw_requires_support(self):
        env = environment([[0,0,0,0]], command=(0.,0.,.5))
        torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env, SENSOR), torch.ones(1))

    def test_grace_does_not_exempt_all_airborne(self):
        env = environment([[0,0,0,0]], step=0)
        torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env, SENSOR), torch.zeros(1))
        torch.testing.assert_close(phase.all_feet_airborne_penalty(env, SENSOR), torch.ones(1))
        torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env, SENSOR, transition_grace_s=0.), torch.ones(1))

    def test_double_support_grace_and_legacy_unscheduled_penalty(self):
        env = environment([[1,1,1,1]], step=0)
        torch.testing.assert_close(phase.phase_unscheduled_contact_penalty(env, SENSOR), torch.tensor([.5]))
        torch.testing.assert_close(phase.phase_unscheduled_contact_penalty(env, SENSOR, transition_grace_s=.02), torch.zeros(1))
        # Outside transition the old swing-contact penalty remains.
        env.episode_length_buf[:] = 5
        torch.testing.assert_close(phase.phase_unscheduled_contact_penalty(env, SENSOR, transition_grace_s=.02), torch.tensor([.5]))

    def test_touchdown_grace_removes_negative_not_positive_scores(self):
        env = environment([[0,1,1,0]], step=0)
        torch.testing.assert_close(phase.phase_touchdown_timing(env, SENSOR), torch.tensor([-.5]))
        torch.testing.assert_close(phase.phase_touchdown_timing(env, SENSOR, transition_grace_s=.02), torch.zeros(1))

    def test_threshold_units_and_validation(self):
        env = environment([[0,0,0,0]])
        sensor = env.scene.sensors['contact_forces']
        sensor.data.net_forces_w[:, sensor.body_names.index('lf_shin'), 2] = .1
        torch.testing.assert_close(phase.all_feet_airborne_penalty(env, SENSOR), torch.ones(1))
        sensor.data.net_forces_w[:, sensor.body_names.index('lf_shin'), 2] = .101
        torch.testing.assert_close(phase.all_feet_airborne_penalty(env, SENSOR), torch.zeros(1))
        for invalid in [-1., float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                phase.all_feet_airborne_penalty(env, SENSOR, invalid)
        with self.assertRaises(ValueError):
            phase.phase_stance_missing_contact_penalty(env, SENSOR, transition_grace_s=.2)

    def test_registration_is_opt_in_and_preserves_weights(self):
        tree = ast.parse((ROOT/'flat_env_cfg_reward.py').read_text(encoding='utf-8'))
        func = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name=='configure_trot_support_rewards')
        ns = {'phase_generator': phase, 'RewTerm': lambda **kw: NS(**kw), 'CONTACT_SENSOR': SENSOR}
        exec(compile(ast.Module(body=[func], type_ignores=[]), '<support config>', 'exec'), ns)
        cfg = NS(rewards=NS(phase_unscheduled_contact_penalty=NS(params={}), phase_touchdown_timing=NS(params={})))
        ns['configure_trot_support_rewards'](cfg)
        self.assertEqual(cfg.rewards.phase_stance_missing_contact_penalty.weight, -1.)
        self.assertEqual(cfg.rewards.all_feet_airborne_penalty.weight, -2.)
        self.assertEqual(cfg.rewards.phase_unscheduled_contact_penalty.params['transition_grace_s'], .02)
        env = environment([[0,0,0,0]])
        a, b = cfg.rewards.phase_stance_missing_contact_penalty, cfg.rewards.all_feet_airborne_penalty
        cost_per_step = (a.func(env, **a.params)*a.weight + b.func(env, **b.params)*b.weight)*.02
        torch.testing.assert_close(cost_per_step, torch.tensor([-.06]))
        baseline = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name=='configure_rewards')
        self.assertNotIn('all_feet_airborne_penalty', ast.unparse(baseline))
        self.assertNotIn('phase_stance_missing_contact_penalty', ast.unparse(baseline))

    def test_profile_and_both_entrypoints(self):
        old = json.loads((ROOT/'configs/v6_20_2_training.json').read_text())
        new = json.loads((ROOT/'configs/v6_20_3_training.json').read_text())
        self.assertEqual(new['version'], 'v6.20.3')
        self.assertTrue(new['fresh'])
        self.assertNotIn('--checkpoint', new['train_args'])
        self.assertEqual(new['train_args'].count('--trot-support-penalties'), 1)
        self.assertEqual([x for x in new['train_args'] if x!='--trot-support-penalties'], old['train_args'])
        for script in ['train.py', 'play.py']:
            source = (ROOT/script).read_text(encoding='utf-8')
            ast.parse(source)
            self.assertIn("'--trot-support-penalties'", source)
            self.assertIn('if args_cli.trot_support_penalties:', source)


if __name__ == '__main__':
    unittest.main()
