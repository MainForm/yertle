"""Double-support clock, reward, observation and CLI compatibility tests (no Kit)."""
import argparse
import ast
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import torch

ROOT=Path(__file__).resolve().parents[1]
stub=ModuleType('isaaclab.managers');stub.SceneEntityCfg=object
spec=importlib.util.spec_from_file_location('double_support_phase',ROOT/'phase_generator.py')
phase=importlib.util.module_from_spec(spec)
with patch.dict(sys.modules,{'isaaclab.managers':stub}):spec.loader.exec_module(phase)

def env_at(phases, duty=.6):
    n=len(phases)
    return NS(num_envs=n,cfg=NS(trot_frequency_hz=1.,trot_duty_factor=duty),
              episode_length_buf=torch.tensor(phases)*1000.,step_dt=.001,
              command_manager=NS(get_command=lambda _:torch.tensor([[.1,0.,0.]]*n)))

class DutyTests(unittest.TestCase):
    def test_sequence_and_diagonal_identity(self):
        env=env_at([.05,.2,.55,.7])
        expected=torch.tensor([[1,1,1,1],[1,0,0,1],[1,1,1,1],[0,1,1,0]],dtype=torch.bool)
        torch.testing.assert_close(phase._desired_contact(env,'base_velocity'),expected)

    def test_cycle_has_twenty_percent_four_support_and_no_scheduled_flight(self):
        env=env_at(((torch.arange(1000)+.25)/1000).tolist())
        contacts=phase._desired_contact(env,'base_velocity')
        self.assertEqual(contacts.all(-1).sum().item(),200)
        self.assertTrue((contacts.sum(-1)>=2).all())
        torch.testing.assert_close(contacts.float().mean(0),torch.full((4,),.6))

    def test_observation_reports_new_duty_without_shape_or_offset_change(self):
        env=env_at([.1,.2]);_,_,_,duties,offsets=phase._gait_state(env,'base_velocity')
        torch.testing.assert_close(phase.gait_duty_factor(env),torch.full((2,1),.6))
        torch.testing.assert_close(offsets,torch.tensor([[0.,.5,.5,0.]]*2))
        self.assertEqual(phase.phase_clock_sin_cos(env).shape,(2,2))

    def test_legacy_fallback_is_bitwise_equal_to_explicit_half_duty(self):
        env=env_at([.0,.05,.1,.25,.5,.55,.6,.75,.99],.5)
        expected=phase._desired_contact(env,'base_velocity').clone()
        grace=phase._trot_transition(env,'base_velocity',.02).clone()
        del env.cfg.trot_duty_factor
        self.assertTrue(torch.equal(expected,phase._desired_contact(env,'base_velocity')))
        self.assertTrue(torch.equal(grace,phase._trot_transition(env,'base_velocity',.02)))
        torch.testing.assert_close(phase.gait_duty_factor(env),torch.full((9,1),.5))

    def test_grace_covers_both_touchdown_and_liftoff(self):
        env=env_at([0.,.1,.5,.6,.25,.75])
        torch.testing.assert_close(phase._trot_transition(env,'base_velocity',.02),torch.tensor([1,1,1,1,0,0],dtype=torch.bool))

    def test_all_four_support_is_not_unscheduled_and_needs_no_clearance(self):
        env=env_at([.05,.55])
        with patch.object(phase,'_actual_contact',return_value=torch.ones(2,4,dtype=torch.bool)),patch.object(phase,'_foot_sole_heights',return_value=torch.zeros(2,4)):
            torch.testing.assert_close(phase.phase_contact_schedule(env,None),torch.ones(2))
            torch.testing.assert_close(phase.phase_unscheduled_contact_penalty(env,None),torch.zeros(2))
            torch.testing.assert_close(phase.phase_swing_clearance(env),torch.zeros(2))

    def test_airborne_penalty_remains_active_during_full_support(self):
        env=env_at([.05,.55])
        with patch.object(phase,'_instant_foot_contact',return_value=torch.zeros(2,4,dtype=torch.bool)):
            torch.testing.assert_close(phase.all_feet_airborne_penalty(env,None),torch.ones(2))
            torch.testing.assert_close(phase.phase_stance_missing_contact_penalty(env,None),torch.ones(2))

    def test_invalid_runtime_values(self):
        for duty in (.49,1.,float('nan'),float('inf')):
            with self.subTest(duty=duty),self.assertRaises(ValueError):phase.gait_duty_factor(env_at([0.],duty))

    def test_cli_registration_validation_and_assignment_in_both_entrypoints(self):
        for filename in ('train.py','play.py'):
            tree=ast.parse((ROOT/filename).read_text(encoding='utf8'))
            registration=next(n for n in tree.body if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and any(isinstance(a,ast.Constant) and a.value=='--trot-duty-factor' for a in n.value.args))
            parser=argparse.ArgumentParser();exec(compile(ast.Module(body=[registration],type_ignores=[]),filename,'exec'),{'parser':parser})
            self.assertIsNone(parser.parse_args([]).trot_duty_factor)
            self.assertEqual(parser.parse_args(['--trot-duty-factor','.6']).trot_duty_factor,.6)
            validation=next(n for n in tree.body if isinstance(n,ast.If) and 'args_cli.trot_duty_factor is not None' in ast.unparse(n.test))
            for invalid in (.49,1.,float('nan'),float('inf')):
                with patch.object(parser,'error',side_effect=ValueError),self.assertRaises(ValueError):
                    exec(compile(ast.Module(body=[validation],type_ignores=[]),filename,'exec'),{'parser':parser,'args_cli':NS(trot_duty_factor=invalid)})
            main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
            assignment=next(n for n in main.body if isinstance(n,ast.If) and ast.unparse(n.test)=='args_cli.trot_duty_factor is not None')
            cfg=NS();exec(compile(ast.Module(body=[assignment],type_ignores=[]),filename,'exec'),{'env_cfg':cfg,'args_cli':NS(trot_duty_factor=None)})
            self.assertFalse(hasattr(cfg,'trot_duty_factor'))
            exec(compile(ast.Module(body=[assignment],type_ignores=[]),filename,'exec'),{'env_cfg':cfg,'args_cli':NS(trot_duty_factor=.6)})
            self.assertEqual(cfg.trot_duty_factor,.6)

    def test_fresh_profile(self):
        profile=json.loads((ROOT/'configs/v6_21_6_training.json').read_text())
        baseline=json.loads((ROOT/'configs/v6_21_3_training.json').read_text())
        args=profile['train_args'].copy();i=args.index('--trot-duty-factor');self.assertEqual(args.pop(i+1),'0.6');args.pop(i)
        self.assertEqual(args,baseline['train_args'])
        self.assertTrue(profile['fresh']);self.assertNotIn('--checkpoint',args)

if __name__=='__main__':unittest.main()
