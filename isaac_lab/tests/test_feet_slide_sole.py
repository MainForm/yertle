"""Production geometry/reward tests without Kit."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import torch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('slide_geometry',ROOT/'foot_geometry.py')
geometry=importlib.util.module_from_spec(spec);spec.loader.exec_module(geometry)
tree=ast.parse((ROOT/'flat_env_cfg_reward.py').read_text(encoding='utf-8'))
node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='feet_slide_sole')
namespace={'torch':torch,'SceneEntityCfg':object,'FOOT_NAMES':geometry.FOOT_NAMES}
exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual feet_slide_sole>','exec'),namespace)

class SoleSlideTests(unittest.TestCase):
    def speed(self,vel=(0,0,0),omega=(0,0,0),com=(0,0,0),q=(1,0,0,0),points=None,mask=None,band=.001):
        def tensor(v):return torch.tensor([[v]],dtype=torch.float64)
        return geometry.lowest_surface_point_velocity(tensor((0,0,0)),tensor(q),tensor(com),
            tensor(vel),tensor(omega),torch.tensor([points or [[0,0,-.1]]],dtype=torch.float64),
            band,mask)

    def test_translation(self):
        torch.testing.assert_close(self.speed(vel=(.2,-.3,.4)),torch.tensor([[[.2,-.3,.4]]],dtype=torch.float64))

    def test_rotating_about_stationary_sole_is_not_sliding(self):
        # COM moves +X while rotation makes the sole stationary.
        torch.testing.assert_close(self.speed(vel=(.1,0,0),omega=(0,1,0)),torch.zeros(1,1,3,dtype=torch.float64))

    def test_stationary_com_does_not_hide_sole_motion(self):
        torch.testing.assert_close(self.speed(omega=(0,1,0)),torch.tensor([[[-.1,0,0]]],dtype=torch.float64))

    def test_com_offset_is_included(self):
        torch.testing.assert_close(self.speed(omega=(0,1,0),com=(0,0,.05)),torch.tensor([[[-.15,0,0]]],dtype=torch.float64))

    def test_rotation_changes_lowest_patch_and_quaternion_sign_is_equivalent(self):
        q=(2**-.5,0,2**-.5,0);points=[[-.2,0,-.1],[.1,0,-.1]]
        expected=torch.tensor([[[-.1,0,.1]]],dtype=torch.float64)
        torch.testing.assert_close(self.speed(q=q,omega=(0,1,0),points=points),expected)
        torch.testing.assert_close(self.speed(q=tuple(-x for x in q),omega=(0,1,0),points=points),expected)

    def test_padding_does_not_bias_patch_centroid(self):
        points=[[-.1,0,-.1],[.1,0,-.1],[.1,0,-.1]]
        mask=torch.tensor([[True,True,False]])
        torch.testing.assert_close(self.speed(points=points,mask=mask,omega=(0,0,1)),torch.zeros(1,1,3,dtype=torch.float64))

    def test_high_vertices_are_excluded_and_band_validation(self):
        points=[[0,0,-.1],[.1,0,-.05]]
        torch.testing.assert_close(self.speed(points=points,omega=(0,0,1)),torch.zeros(1,1,3,dtype=torch.float64))
        for band in (-.1,float('nan'),float('inf')):
            with self.subTest(band=band),self.assertRaises(ValueError):self.speed(band=band)

    def test_contact_history_threshold_name_order_and_vertical_exclusion(self):
        names=['rb_shin','base_link','lf_shin','lb_shin','rf_shin']
        history=torch.zeros(2,3,5,3,dtype=torch.float64)
        history[0,0,0,2]=2 # RB: earlier contact is retained
        history[0,2,2,2]=1 # LF: equality does NOT pass >1 N
        history[0,2,3,2]=.99
        history[0,2,4,2]=2 # RF
        sensor=NS(body_names=names,data=NS(net_forces_w_history=history))
        env=NS(scene=NS(sensors={'contact_forces':sensor}))
        velocities=torch.tensor([[[3,4,9],[0,12,8],[.3,.4,7],[8,6,9]]]*2,dtype=torch.float64)
        namespace['foot_sole_contact_velocities']=lambda env,band:velocities
        actual=namespace['feet_slide_sole'](env,NS(name='contact_forces',body_ids=[0,2,3,4]))
        torch.testing.assert_close(actual,torch.tensor([22,0],dtype=torch.float64))

    def test_registration_preserves_weight_and_only_changes_slide(self):
        configure=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='configure_rewards')
        assignment=next(n for n in configure.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='rewards.feet_slide')
        kw={k.arg:k.value for k in assignment.value.keywords}
        self.assertEqual(ast.unparse(kw['func']),'feet_slide_sole')
        self.assertEqual(ast.literal_eval(kw['weight']),-.10)
        self.assertIn("'surface_band_m': 0.001",ast.unparse(kw['params']))

if __name__=='__main__':unittest.main()
