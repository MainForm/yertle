import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import torch

spec=importlib.util.spec_from_file_location('joint_position_limits',Path(__file__).resolve().parents[1]/'joint_position_limits.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class JointPositionLimitTests(unittest.TestCase):
    def test_release_all_envs_without_changing_reference(self):
        bounds=torch.empty(2,12,2);bounds[...,0]=-torch.inf;bounds[...,1]=torch.inf
        ref=torch.ones(2,12)
        data=NS(joint_pos_limits=bounds.clone(),soft_joint_pos_limits=bounds.clone(),default_joint_pos=ref.clone())
        def set_limits(values,indices):
            self.assertEqual(indices.tolist(),[0,1]);bounds.copy_(values)
        robot=NS(joint_names=list(range(12)),device='cpu',data=data,
                 root_physx_view=NS(set_dof_limits=set_limits,get_dof_limits=lambda:bounds))
        m.release_joint_position_limits(NS(scene={'robot':robot},num_envs=2))
        self.assertTrue(torch.isneginf(data.joint_pos_limits[...,0]).all())
        self.assertTrue(torch.isposinf(data.joint_pos_limits[...,1]).all())
        self.assertFalse(torch.isnan(data.soft_joint_pos_limits).any())
        torch.testing.assert_close(data.default_joint_pos,ref)

    def test_backend_refusal_fails_explicitly(self):
        robot=NS(joint_names=list(range(12)),device='cpu',
                 root_physx_view=NS(set_dof_limits=lambda *a,**kw:None,get_dof_limits=lambda:torch.zeros(1,12,2)))
        with self.assertRaises(RuntimeError):m.release_joint_position_limits(NS(scene={'robot':robot},num_envs=1))

    def test_wrong_joint_count_rejected(self):
        with self.assertRaises(ValueError):m.release_joint_position_limits(NS(scene={'robot':NS(joint_names=['x'])},num_envs=1))


if __name__=='__main__':unittest.main()
