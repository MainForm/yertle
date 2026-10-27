"""CPU numerical/registration tests for the low-body height interval cost."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import torch

SOURCE=Path(__file__).resolve().parents[1]/'flat_env_cfg_reward.py'
TREE=ast.parse(SOURCE.read_text(encoding='utf8'))
scope={'torch':torch,'SceneEntityCfg':lambda name:NS(name=name)}
nodes=[n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name in ('_interval_barrier','body_height_barrier')]
exec(compile(ast.Module(body=nodes,type_ignores=[]),str(SOURCE),'exec'),scope)

class BodyHeightTests(unittest.TestCase):
    def test_interval_and_quadratic_excess(self):
        h=torch.tensor([.06,.08,.10,.125,.15,.17,.19],dtype=torch.float64)
        env=NS(scene={'robot':NS(data=NS(root_pos_w=torch.stack([h*0,h*0,h],dim=1)))})
        actual=scope['body_height_barrier'](env)
        torch.testing.assert_close(actual,torch.tensor([4.,1.,0.,0.,0.,1.,4.],dtype=h.dtype),atol=1e-12,rtol=1e-12)
        torch.testing.assert_close(-.08*actual*.02,torch.tensor([-.0064,-.0016,0,0,0,-.0016,-.0064],dtype=h.dtype),atol=1e-12,rtol=1e-12)

    def test_registration_and_low_body_defaults(self):
        cfg=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='configure_rewards')
        node=next(n for n in cfg.body if isinstance(n,ast.Assign) and any(ast.unparse(t)=='rewards.body_height_barrier' for t in n.targets))
        rewards=NS();env_cfg=NS(rewards=rewards)
        def term(**kw):return NS(**kw)
        ns=dict(scope,rewards=rewards,RewTerm=term,SceneEntityCfg=lambda name:NS(name=name))
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(SOURCE),'exec'),ns)
        self.assertEqual(rewards.body_height_barrier.weight,-.08)
        self.assertEqual({k:v for k,v in rewards.body_height_barrier.params.items() if k!='asset_cfg'},dict(lower_m=.10,upper_m=.15,transition_m=.02))

if __name__=='__main__':unittest.main()
