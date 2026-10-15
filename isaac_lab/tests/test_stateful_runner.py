"""Checkpoint policy/atomicity tests without launching Kit."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import os
import tempfile
import unittest
import torch

tree=ast.parse((Path(__file__).resolve().parents[1]/'stateful_runner.py').read_text(encoding='utf-8'))
node=next(n for n in tree.body if isinstance(n,ast.ClassDef))


class Parent:
    def load(self,*args,**kwargs):return self.infos
    def save(self,path,infos=None):
        torch.save({'iter':self.current_learning_iteration,'infos':infos},path)
        if getattr(self,'fail',False):raise IOError('test failure')


ns=dict(OnPolicyRunner=Parent,torch=torch,os=os,rng_state=lambda:{'test':True})
exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual stateful runner>','exec'),ns)
Runner=ns['StatefulOnPolicyRunner']


class StatefulRunnerTests(unittest.TestCase):
    def runner(self):
        r=object.__new__(Runner)
        r.current_learning_iteration=5;r.tot_timesteps=10;r.tot_time=1.
        r.alg=NS(learning_rate=.01);r.runner_digest='same'
        r.state_adapter=NS(capture=lambda:{'test':42},restore=lambda state:None)
        r.resume_env_state='auto';r._continuation=None
        r._boundary=dict(obs=torch.ones(2,3),cur_reward_sum=torch.ones(2),cur_episode_length=torch.ones(2),rewbuffer=[],lenbuffer=[])
        return r

    def test_atomic_save_contains_next_iteration(self):
        r=self.runner()
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'model_5.pt';r.save(p)
            saved=torch.load(p,weights_only=False)
            self.assertEqual(saved['infos']['yertle_continuation']['next_iteration'],6)
            self.assertFalse(Path(str(p)+'.tmp').exists())

    def test_failed_save_preserves_old_file(self):
        r=self.runner()
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'model_5.pt';r.save(p);before=p.read_bytes();r.fail=True
            with self.assertRaises(IOError):r.save(p)
            self.assertEqual(p.read_bytes(),before)
            self.assertFalse(Path(str(p)+'.tmp').exists())

    def test_save_outside_boundary_rejected(self):
        r=self.runner();r._boundary=None
        with self.assertRaises(RuntimeError):r.save('unused.pt')

    def test_legacy_auto_and_required(self):
        r=self.runner();r.infos=None;r.load('unused')
        self.assertIsNone(r._continuation)
        r.resume_env_state='required'
        with self.assertRaises(ValueError):r.load('unused')

    def test_full_load_restores_next_iteration_and_learning_rate(self):
        r=self.runner()
        s=dict(environment={},runner_digest='same',next_iteration=6,tot_timesteps=100,tot_time=20,learning_rate=.003)
        r.infos={'yertle_continuation':s};r.load('unused')
        self.assertEqual(r.current_learning_iteration,6);self.assertEqual(r.alg.learning_rate,.003)

    def test_wrong_settings_or_iteration_rejected(self):
        for digest,iteration in [('different',6),('same',9)]:
            r=self.runner();r.infos={'yertle_continuation':dict(environment={},runner_digest=digest,next_iteration=iteration)}
            with self.assertRaises(ValueError):r.load('unused')

    def test_off_explicitly_skips_full_state(self):
        r=self.runner();r.resume_env_state='off';r.infos={'yertle_continuation':{'invalid':True}}
        r.load('unused');self.assertIsNone(r._continuation)


if __name__=='__main__':unittest.main()
