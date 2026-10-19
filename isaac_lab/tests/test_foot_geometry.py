import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import torch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('sole_geometry',ROOT/'foot_geometry.py')
geometry=importlib.util.module_from_spec(spec);spec.loader.exec_module(geometry)


class FootGeometryTests(unittest.TestCase):
    def test_identity_uses_surface_not_link_origin(self):
        p=torch.tensor([[[0.,0.,.14]]])
        q=torch.tensor([[[1.,0.,0.,0.]]])
        vertices=torch.tensor([[[0.,0.,-.10],[0.,0.,-.12]]])
        torch.testing.assert_close(geometry.minimum_surface_height(p,q,vertices),torch.tensor([[.02]]))

    def test_pitch_rotation_rotates_surface_offset(self):
        p=torch.tensor([[[0.,0.,.2]]],dtype=torch.float64)
        q=torch.tensor([[[2**-.5,0.,2**-.5,0.]]],dtype=torch.float64)
        vertices=torch.tensor([[[.1,0.,-.12],[-.03,0.,-.12]]],dtype=torch.float64)
        torch.testing.assert_close(geometry.minimum_surface_height(p,q,vertices),torch.tensor([[.1]],dtype=torch.float64))

    def test_yaw_does_not_change_height_and_quaternion_sign_is_equivalent(self):
        p=torch.zeros(2,1,3);q=torch.tensor([[[2**-.5,0.,0.,2**-.5]], [[-2**-.5,0.,0.,-2**-.5]]])
        vertices=torch.tensor([[[.1,.04,-.12]]])
        torch.testing.assert_close(geometry.minimum_surface_height(p,q,vertices),torch.full((2,1),-.12))

    def test_batch_translation_and_duplicate_padding(self):
        p=torch.zeros(2,4,3);p[1,:,2]=.1
        q=torch.zeros(2,4,4);q[...,0]=1
        vertices=torch.tensor([[[0.,0.,-.1],[0.,0.,-.1]]]*4)
        torch.testing.assert_close(geometry.minimum_surface_height(p,q,vertices),torch.tensor([[-.1]*4,[0.]*4]))

    def test_live_buffers_are_read_and_geometry_is_cached(self):
        calls=[];old=geometry.load_sole_points
        geometry.load_sole_points=lambda path:(calls.append(path) or [[[0,0,-.1]]]*4)
        try:
            q=torch.zeros(1,4,4);q[...,0]=1
            pos=torch.zeros(1,4,3);pos[...,2]=.14
            robot=NS(cfg=NS(spawn=NS(usd_path='test.usd')),device='cpu',body_names=list(geometry.FOOT_NAMES),data=NS(body_pos_w=pos,body_quat_w=q))
            env=NS(scene={'robot':robot})
            torch.testing.assert_close(geometry.foot_sole_heights(env),torch.full((1,4),.04))
            pos[...,2]+=.1
            torch.testing.assert_close(geometry.foot_sole_heights(env),torch.full((1,4),.14))
            self.assertEqual(calls,['test.usd'])
        finally:geometry.load_sole_points=old

    def test_phase_clearance_uses_the_surface_height(self):
        phase_tree=ast.parse((ROOT/'phase_generator.py').read_text(encoding='utf-8'))
        phase_node=next(n for n in phase_tree.body if isinstance(n,ast.FunctionDef) and n.name=='phase_swing_clearance')
        heights=torch.tensor([[0.,.1,0.,.1]])
        ns={'torch':torch,'SceneEntityCfg':object,'_foot_sole_heights':lambda env:heights,
            '_desired_contact':lambda env,name:torch.tensor([[False,False,True,True]]),
            '_moving':lambda env,name:torch.tensor([True])}
        exec(compile(ast.Module(body=[phase_node],type_ignores=[]),'<actual clearance function>','exec'),ns)
        torch.testing.assert_close(ns['phase_swing_clearance'](None),torch.tensor([.25]))
        heights[:] = .015
        torch.testing.assert_close(ns['phase_swing_clearance'](None),torch.tensor([0.]))
        heights[:] = .03
        torch.testing.assert_close(ns['phase_swing_clearance'](None),torch.tensor([0.]))
        heights[:] = .0075
        torch.testing.assert_close(ns['phase_swing_clearance'](None),torch.tensor([.125]))


if __name__=='__main__':unittest.main()
