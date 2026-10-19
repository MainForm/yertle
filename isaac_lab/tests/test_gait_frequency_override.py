"""Retained gait override and removed travel reward regression tests."""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import torch

ROOT=Path(__file__).resolve().parents[1]
managers=ModuleType('isaaclab.managers')
managers.SceneEntityCfg=object
spec=importlib.util.spec_from_file_location('frequency_test_phase',ROOT/'phase_generator.py')
phase=importlib.util.module_from_spec(spec)
with patch.dict(sys.modules,{'isaaclab.managers':managers}):
    spec.loader.exec_module(phase)

class FrequencyTests(unittest.TestCase):
    def test_clocks_and_legacy_default(self):
        env=NS(num_envs=1,episode_length_buf=torch.tensor([25]),step_dt=.02,
               command_manager=NS(get_command=lambda _:torch.tensor([[.1,0,0]])),cfg=NS())
        for hz in (1.,1.2,1.7,2.2):
            env.cfg.trot_frequency_hz=hz
            p,_,f,_,_=phase._gait_state(env,'base_velocity')
            torch.testing.assert_close(f,torch.tensor([hz]))
            torch.testing.assert_close(p,torch.tensor([(.5*hz)%1]))
        del env.cfg.trot_frequency_hz
        torch.testing.assert_close(phase._gait_state(env,'base_velocity')[2],torch.tensor([2.2]))

    def test_profile_arguments_only_differ_by_frequency(self):
        old=json.loads((ROOT/'configs/v6_21_training.json').read_text())
        new=json.loads((ROOT/'configs/v6_21_2_training.json').read_text())
        args=old['train_args'].copy()
        args[args.index('--trot-frequency-hz')+1]='1.7'
        self.assertEqual(args,new['train_args'])
        self.assertTrue(new['fresh'])
        self.assertNotIn('--checkpoint',args)
        self.assertNotIn('--motor-travel-balance',args)
        self.assertFalse(any(k.startswith('motor_travel') for k in new))

    def test_reward_and_cli_implementation_removed(self):
        self.assertFalse((ROOT/'motor_travel_balance.py').exists())
        for name in ('train.py','play.py','flat_env_cfg_reward.py'):
            source=(ROOT/name).read_text()
            self.assertNotIn('motor_travel_balance',source)
            self.assertNotIn('--motor-travel-balance',source)

    def test_v6213_and_v6214_fresh_profiles_only_differ_in_clock(self):
        baseline=json.loads((ROOT/'configs/v6_21_2_training.json').read_text())
        for version,hz in (('v6_21_3','1.7'),('v6_21_4','1.2')):
            profile=json.loads((ROOT/f'configs/{version}_training.json').read_text())
            args=baseline['train_args'].copy()
            args[args.index('--trot-frequency-hz')+1]=hz
            self.assertEqual(profile['train_args'],args)
            self.assertTrue(profile['fresh'])
            self.assertNotIn('--checkpoint',args)
            self.assertNotIn('--motor-travel-balance',args)
            self.assertEqual(args[args.index('--num_envs')+1],'1024')
            self.assertEqual(args[args.index('--max_iterations')+1],'7000')

if __name__=='__main__':
    unittest.main()
