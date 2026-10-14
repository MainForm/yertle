import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import torch

ROOT=Path(__file__).resolve().parents[1]
TREE=ast.parse((ROOT/'flat_env_cfg_reward.py').read_text(encoding='utf-8'))
class Base:
    def __init__(self,cfg,env):pass
node=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='ActionSecondDifferencePenalty')
ns={'torch':torch,'ManagerTermBase':Base}
exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual second-difference reward>','exec'),ns)
Term=ns['ActionSecondDifferencePenalty']

class ActionSecondDifferenceTests(unittest.TestCase):
    def setUp(self):
        self.action=NS(raw_actions=torch.zeros(2,3,dtype=torch.float64))
        self.env=NS(action_manager=NS(get_term=lambda name:self.action),common_step_counter=0)
        self.term=Term(NS(params={'action_name':'joint_pos'}),self.env)

    def step(self,values):
        self.env.common_step_counter+=1
        self.action.raw_actions.copy_(torch.tensor(values,dtype=torch.float64))
        return self.term(self.env,'joint_pos').clone()

    def test_first_two_actions_are_not_penalized(self):
        for values in ([[8,8,8],[2,2,2]],[[0,0,0],[-8,-8,-8]]):
            torch.testing.assert_close(self.step(values),torch.zeros(2,dtype=torch.float64))

    def test_constant_and_linear_ramp_have_zero_cost(self):
        for k in range(7):
            torch.testing.assert_close(self.step([[.4,.4,.4],[k*.1,k*.2,-k*.3]]),torch.zeros(2,dtype=torch.float64),atol=1e-25,rtol=0)

    def test_alternating_action_matches_second_difference_squared_sum(self):
        self.step([[.2]*3,[.1]*3]);self.step([[-.2]*3,[-.1]*3])
        torch.testing.assert_close(self.step([[.2]*3,[.1]*3]),torch.tensor([3*(4*.2)**2,3*(4*.1)**2],dtype=torch.float64))

    def test_partial_reset_does_not_mix_episodes_or_reset_other_env(self):
        self.step([[1]*3,[1]*3]);self.step([[-1]*3,[-1]*3])
        self.term.reset(torch.tensor([0]))
        torch.testing.assert_close(self.step([[100]*3,[1]*3]),torch.tensor([0.,48.],dtype=torch.float64))
        torch.testing.assert_close(self.step([[100]*3,[-1]*3]),torch.tensor([0.,48.],dtype=torch.float64))
        torch.testing.assert_close(self.step([[100]*3,[1]*3]),torch.tensor([0.,48.],dtype=torch.float64))

    def test_repeat_evaluation_is_idempotent(self):
        self.step([[1]*3,[1]*3]);self.step([[-1]*3,[-1]*3])
        expected=self.step([[1]*3,[1]*3])
        for _ in range(3):torch.testing.assert_close(self.term(self.env,'joint_pos'),expected)
        torch.testing.assert_close(self.step([[-1]*3,[-1]*3]),expected)

    def test_reset_all_and_no_input_mutation(self):
        self.step([[1]*3,[1]*3]);self.step([[-1]*3,[-1]*3]);self.step([[1]*3,[1]*3])
        before=self.action.raw_actions.clone();self.term.reset()
        torch.testing.assert_close(self.action.raw_actions,before)
        torch.testing.assert_close(self.step([[100]*3,[100]*3]),torch.zeros(2,dtype=torch.float64))

    def test_high_frequency_penalty_exceeds_trot_at_same_amplitude(self):
        def total(hz):
            self.term.reset();scores=[]
            for k in range(500):
                a=.2*torch.cos(torch.tensor(2*torch.pi*hz*k*.02)).item()
                scores.append(self.step([[a]*3,[a]*3])[0].item())
            return sum(scores[2:])
        self.assertGreater(total(25),1000*total(2.2))

    def test_registration(self):
        configure=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='configure_rewards')
        assignment=next(n for n in configure.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='rewards.action_second_difference_l2')
        values={k.arg:k.value for k in assignment.value.keywords}
        self.assertEqual(ast.unparse(values['func']),'ActionSecondDifferencePenalty')
        self.assertEqual(ast.literal_eval(values['weight']),-.05)
        self.assertEqual(ast.literal_eval(values['params']),{'action_name':'joint_pos'})

if __name__=='__main__':unittest.main()
