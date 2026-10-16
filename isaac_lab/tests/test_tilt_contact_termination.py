import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS, ModuleType
import sys
import unittest
from unittest.mock import patch
import torch

spec=importlib.util.spec_from_file_location('tilt_contact_test',Path(__file__).parents[1]/'train_terminations.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class TiltContactTerminationTests(unittest.TestCase):
    def env(self,poses,flags):
        gravity=[]
        for roll,pitch in poses:
            r,p=math.radians(roll),math.radians(pitch)
            gravity.append([math.sin(p),-math.cos(p)*math.sin(r),-math.cos(p)*math.cos(r)])
        contacts=NS(read=lambda *args:torch.tensor(flags,dtype=torch.bool))
        return NS(scene={'robot':NS(data=NS(projected_gravity_b=torch.tensor(gravity,dtype=torch.float64)))},_yertle_ground_contacts=contacts)

    def result(self,poses,flags):
        return m.body_tilt_and_nonsole_ground_contact(self.env(poses,flags),math.pi/4,NS(name='robot')).tolist()

    def test_and_truth_table(self):
        self.assertEqual(self.result([(0,0),(0,0),(60,0),(60,0)],[[False],[True],[False],[True]]),[False,False,False,True])

    def test_both_signs_and_exact_threshold(self):
        poses=[(44.9,0),(45,0),(-45,0),(0,44.9),(0,45),(0,-45),(0,45.1),(180,0),(30,30)]
        self.assertEqual(self.result(poses,[[True]]*len(poses)),[False,True,True,False,True,True,True,True,False])

    def test_any_forbidden_body_qualifies(self):
        flags=torch.zeros(13,13,dtype=torch.bool);flags.fill_diagonal_(True)
        self.assertEqual(self.result([(60,0)]*13,flags.tolist()),[True]*13)

    def test_sole_only_or_airborne_does_not_terminate(self):
        self.assertEqual(self.result([(60,0),(0,70)],[[False]*13]*2),[False,False])

    def test_tilt_without_current_contact_not_latched(self):
        e=self.env([(60,0)],[[True]])
        self.assertTrue(m.body_tilt_and_nonsole_ground_contact(e,math.pi/4,NS(name='robot')).item())
        e._yertle_ground_contacts.read=lambda *args:torch.zeros(1,13,dtype=torch.bool)
        self.assertFalse(m.body_tilt_and_nonsole_ground_contact(e,math.pi/4,NS(name='robot')).item())

    def test_invalid_tolerance(self):
        e=self.env([(0,0)],[[False]])
        for value in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):m.body_tilt_and_nonsole_ground_contact(e,math.pi/4,NS(name='robot'),sole_tolerance_m=value)

    def test_registration_disables_old_term_preserves_timeout(self):
        managers=ModuleType('isaaclab.managers');managers.SceneEntityCfg=lambda name:NS(name=name);managers.TerminationTermCfg=lambda **kw:NS(**kw)
        cfg=NS(terminations=NS(body_tilt='old',time_out='keep',base_contact=None))
        with patch.dict(sys.modules,{'isaaclab.managers':managers}):m.configure_body_tilt_termination(cfg,45)
        self.assertIsNone(cfg.terminations.body_tilt)
        self.assertEqual(cfg.terminations.time_out,'keep')
        term=cfg.terminations.body_tilt_and_nonsole_contact
        self.assertIs(term.func,m.body_tilt_and_nonsole_ground_contact)
        self.assertAlmostEqual(term.params['limit_angle'],math.pi/4)
        self.assertEqual(term.params['sole_tolerance_m'],.005)
        self.assertEqual(term.params['separation_tolerance_m'],.001)

    def test_none_and_invalid_angles(self):
        cfg=NS(terminations=NS(body_tilt='old'))
        m.configure_body_tilt_termination(cfg,None)
        self.assertEqual(cfg.terminations.body_tilt,'old')
        for value in (0,-1,91,float('nan'),float('inf')):
            with self.assertRaises(ValueError):m.configure_body_tilt_termination(cfg,value)

    def test_optional_seventy_degree_boundary_retains_and_semantics(self):
        poses=[(45,0),(69.9,0),(70,0),(-70,0),(0,70),(0,-70),(80,0),(0,0)]
        flags=[[True]]*6+[[False],[True]]
        result=m.body_tilt_and_nonsole_ground_contact(self.env(poses,flags),math.radians(70),NS(name='robot'))
        self.assertEqual(result.tolist(),[False,False,True,True,True,True,False,False])

    def test_v619_profile_fresh_and_base_contact_only(self):
        import json
        profile=json.loads((Path(__file__).parents[1]/'configs/v6_19_training.json').read_text())
        args=profile['train_args']
        self.assertTrue(profile['fresh'])
        self.assertNotIn('--checkpoint',args)
        self.assertNotIn('--body-tilt-termination-deg',args)
        self.assertIn('--base-contact-termination',args)
        self.assertEqual(profile['sole_tolerance_m'],.005)
        self.assertEqual(profile['fall_cost_at_control_dt_0p02'],-5.)

    def test_base_contact_registration_disables_tilt_and_retargets_fall_cost(self):
        managers=ModuleType('isaaclab.managers');managers.SceneEntityCfg=lambda name,**kw:NS(name=name,**kw);managers.TerminationTermCfg=lambda **kw:NS(**kw)
        mdp=NS(illegal_contact=lambda *a,**kw:None)
        envs=ModuleType('isaaclab.envs');envs.mdp=mdp
        cfg=NS(terminations=NS(body_tilt='old',body_tilt_and_nonsole_contact='old',time_out='keep'),
               rewards=NS(fall_termination_penalty=NS(weight=-250.,params={'term_name':'body_tilt_and_nonsole_contact'})))
        with patch.dict(sys.modules,{'isaaclab.managers':managers,'isaaclab.envs':envs}):m.configure_base_contact_termination(cfg)
        self.assertIsNone(cfg.terminations.body_tilt)
        self.assertIsNone(cfg.terminations.body_tilt_and_nonsole_contact)
        self.assertEqual(cfg.terminations.time_out,'keep')
        self.assertIs(cfg.terminations.base_contact.func,mdp.illegal_contact)
        self.assertEqual(cfg.terminations.base_contact.params['threshold'],1.)
        self.assertEqual(cfg.terminations.base_contact.params['sensor_cfg'].body_names,'base_link')
        self.assertEqual(cfg.rewards.fall_termination_penalty.params['term_name'],'base_contact')
        self.assertEqual(cfg.rewards.fall_termination_penalty.weight,-250.)


if __name__=='__main__':unittest.main()
