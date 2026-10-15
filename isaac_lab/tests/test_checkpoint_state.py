import importlib.util
from pathlib import Path
import random
from types import SimpleNamespace as NS
import unittest
import numpy as np
import torch

spec=importlib.util.spec_from_file_location('checkpoint_state',Path(__file__).resolve().parents[1]/'checkpoint_state.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class CheckpointStateTests(unittest.TestCase):
    def test_cpu_snapshot_is_independent(self):
        original={'a':torch.ones(2),'b':[torch.zeros(3)]}
        saved=m.cpu_tree(original);original['a'].zero_()
        self.assertEqual(saved['a'].sum().item(),2)

    def test_tensor_attributes_restore_in_place(self):
        obj=NS(a=torch.tensor([1.,2.]), nested={'x':[torch.ones(2)]}, other=object())
        saved=m.capture_attrs(obj);self.assertNotIn('other',saved)
        identity=obj.a;obj.a.zero_();obj.nested['x'][0].zero_()
        m.restore_attrs(obj,saved)
        self.assertIs(identity,obj.a);self.assertEqual(obj.a.tolist(),[1.,2.])
        self.assertEqual(obj.nested['x'][0].sum().item(),2)

    def test_shape_and_dtype_mismatch_rejected(self):
        for tensor in [torch.ones(3),torch.ones(2,dtype=torch.int64)]:
            with self.assertRaises(ValueError):m.restore_value(tensor,torch.ones(2),'test')

    def test_rng_roundtrip(self):
        prior=m.rng_state()
        try:
            random.seed(42);np.random.seed(42);torch.manual_seed(42)
            saved=m.rng_state()
            expected=(random.random(),np.random.rand(),torch.rand(3))
            m.restore_rng(saved)
            actual=(random.random(),np.random.rand(),torch.rand(3))
            self.assertEqual(expected[:2],actual[:2]);torch.testing.assert_close(expected[2],actual[2])
        finally:m.restore_rng(prior)

    def test_config_digest_deterministic_and_sensitive(self):
        a=NS(to_dict=lambda:dict(v=1, fn=m.cpu_tree, ids=slice(None)))
        b=NS(to_dict=lambda:dict(ids=slice(None), fn=m.cpu_tree, v=1))
        self.assertEqual(m.config_digest(a),m.config_digest(b))
        b.to_dict=lambda:dict(v=2,fn=m.cpu_tree,ids=slice(None))
        self.assertNotEqual(m.config_digest(a),m.config_digest(b))


if __name__=='__main__':unittest.main()
