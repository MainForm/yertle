"""Test actual fall reward source without starting Isaac Sim."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import torch

TREE=ast.parse((Path(__file__).parents[1]/'flat_env_cfg_reward.py').read_text(encoding='utf-8'))
namespace={'torch':torch}
exec(compile(ast.Module(body=[n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='fall_termination_penalty'],type_ignores=[]),'<fall reward>','exec'),namespace)
reward=namespace['fall_termination_penalty']

class FallTerminationPenaltyTests(unittest.TestCase):
    def env(self,flags,enabled=True,timeouts=None):
        flags=torch.tensor(flags,dtype=torch.bool)
        terms={'time_out': torch.zeros_like(flags) if timeouts is None else torch.tensor(timeouts,dtype=torch.bool)}
        if enabled:terms['body_tilt_and_nonsole_contact']=flags
        return NS(num_envs=len(flags),device='cpu',termination_manager=NS(active_terms=list(terms),get_term=lambda name:terms[name]))

    def test_exact_named_flags(self):
        torch.testing.assert_close(reward(self.env([False,True,False,True])),torch.tensor([0.,1.,0.,1.]))

    def test_timeout_alone_excluded_but_coincident_fall_counts(self):
        torch.testing.assert_close(reward(self.env([False,True],timeouts=[True,True])),torch.tensor([0.,1.]))

    def test_optional_termination_absent_is_zero(self):
        torch.testing.assert_close(reward(self.env([False,False],enabled=False,timeouts=[True,False])),torch.zeros(2))

    def test_no_latched_penalty_after_reset(self):
        env=self.env([True]);self.assertEqual(reward(env).item(),1.)
        env.termination_manager.get_term('body_tilt_and_nonsole_contact').zero_()
        self.assertEqual(reward(env).item(),0.)

    def test_registration_and_effective_fall_cost(self):
        configure=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='configure_rewards')
        term=next(n for n in configure.body if isinstance(n,ast.Assign) and any(ast.unparse(t)=='rewards.fall_termination_penalty' for t in n.targets))
        values={kw.arg:kw.value for kw in term.value.keywords}
        self.assertEqual(ast.unparse(values['func']),'fall_termination_penalty')
        self.assertEqual(ast.literal_eval(values['params']),{'term_name':'body_tilt_and_nonsole_contact'})
        weight=ast.literal_eval(values['weight'])
        self.assertEqual(weight,-250.)
        torch.testing.assert_close(reward(self.env([False,True]))*weight*.02,torch.tensor([0.,-5.]))

if __name__=='__main__':unittest.main()
