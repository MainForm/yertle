import ast,math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import torch

tree=ast.parse((Path(__file__).parents[1]/'flat_env_cfg_reward.py').read_text(encoding='utf-8'))
ns={'torch':torch,'SceneEntityCfg':lambda name,**kwargs:NS(name=name,**kwargs)}
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('_interval_barrier','shoulder_posture_barrier')],type_ignores=[]),'<shoulder reward>','exec'),ns)

class ShoulderThresholdTests(unittest.TestCase):
    def test_ten_degree_deadband_and_both_signs(self):
        angles=torch.tensor([[0,10,-10,5],[11,-11,0,0]],dtype=torch.float64)*math.pi/180
        env=NS(scene={'robot':NS(data=NS(joint_pos=angles))})
        cost=ns['shoulder_posture_barrier'](env,NS(name='robot',joint_ids=slice(None)))
        torch.testing.assert_close(cost,torch.tensor([0,2*(math.radians(1)/.12)**2],dtype=torch.float64))

    def test_registration_retains_weight_and_explicit_threshold(self):
        configure=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='configure_rewards')
        term=next(n.value for n in configure.body if isinstance(n,ast.Assign) and any(ast.unparse(t)=='rewards.shoulder_posture_barrier' for t in n.targets))
        kw={k.arg:k.value for k in term.keywords}
        self.assertEqual(ast.literal_eval(kw['weight']),-.2)
        values={k.value:v for k,v in zip(kw['params'].keys,kw['params'].values) if isinstance(k,ast.Constant)}
        self.assertEqual(ast.literal_eval(values['lower']),-math.radians(10))
        self.assertEqual(ast.literal_eval(values['upper']),math.radians(10))

if __name__=='__main__':unittest.main()
