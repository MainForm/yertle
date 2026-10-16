import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import torch

spec=importlib.util.spec_from_file_location('attitude_test',Path(__file__).parents[1]/'attitude_metrics.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def gravity(angles):
    return torch.tensor([[math.sin(math.radians(p)),-math.cos(math.radians(p))*math.sin(math.radians(r)),
                          -math.cos(math.radians(p))*math.cos(math.radians(r))] for r,p in angles],dtype=torch.float64)

class AttitudeMetricsTests(unittest.TestCase):
    def env(self,angles,falls):
        data=NS(projected_gravity_b=gravity(angles))
        flags=torch.tensor(falls,dtype=torch.bool)
        class Manager:
            active_terms=['body_tilt_and_nonsole_contact']
            def compute(self):return flags
            def get_term(self,name):return flags
        return NS(scene={'robot':NS(data=data)},termination_manager=Manager())

    def test_signed_angle_units(self):
        pairs=[(0,0),(30,0),(-30,0),(0,45),(0,-45),(179,0),(30,40)]
        torch.testing.assert_close(m.roll_pitch_degrees(gravity(pairs)),torch.tensor(pairs,dtype=torch.float64))

    def test_sample_before_reset_and_original_flags_unchanged(self):
        env=self.env([(60,10),(-60,-10)],[True,False]);rec=m.AttitudeMetrics(env)
        original=env.termination_manager.compute;rec.install()
        result=env.termination_manager.compute();self.assertEqual(result.tolist(),[True,False])
        env.scene['robot'].data.projected_gravity_b.copy_(gravity([(0,0),(0,0)]))
        s=rec.scalars();self.assertAlmostEqual(s['Metrics/body_attitude/roll_mean_deg'],0)
        self.assertAlmostEqual(s['Metrics/body_attitude/roll_abs_mean_deg'],60,places=4)
        self.assertAlmostEqual(s['Metrics/body_attitude/fall_roll_mean_deg'],60,places=4)
        self.assertAlmostEqual(s['Metrics/body_attitude/fall_pitch_mean_deg'],10,places=4)
        rec.close();self.assertEqual(env.termination_manager.compute,original)
        self.assertNotIn('compute',vars(env.termination_manager))

    def test_no_falls_omits_terminal_angles_and_reset_clears(self):
        env=self.env([(20,10)],[False]);rec=m.AttitudeMetrics(env);rec.install()
        try:
            env.termination_manager.compute();s=rec.scalars()
            self.assertEqual(s['Metrics/body_attitude/fall_count'],0)
            self.assertNotIn('Metrics/body_attitude/fall_roll_mean_deg',s)
            rec.reset();self.assertEqual(rec.scalars(),{})
        finally:rec.close()

    def test_multiple_steps_equal_sample_weight_and_max(self):
        env=self.env([(20,0)],[True]);rec=m.AttitudeMetrics(env);rec.install()
        try:
            env.termination_manager.compute()
            env.scene['robot'].data.projected_gravity_b.copy_(gravity([(-40,10)]))
            env.termination_manager.compute();s=rec.scalars()
            self.assertAlmostEqual(s['Metrics/body_attitude/roll_mean_deg'],-10,places=4)
            self.assertAlmostEqual(s['Metrics/body_attitude/roll_abs_max_deg'],40,places=4)
            self.assertEqual(s['Metrics/body_attitude/fall_count'],2)
        finally:rec.close()

    def test_base_contact_failure_angles_are_recorded(self):
        env=self.env([(10,-20)],[True]);env.termination_manager.active_terms=['base_contact']
        rec=m.AttitudeMetrics(env);rec.install()
        try:
            env.termination_manager.compute();s=rec.scalars()
            self.assertEqual(s['Metrics/body_attitude/fall_count'],1)
            self.assertAlmostEqual(s['Metrics/body_attitude/fall_roll_mean_deg'],10,places=4)
            self.assertAlmostEqual(s['Metrics/body_attitude/fall_pitch_mean_deg'],-20,places=4)
        finally:rec.close()

if __name__=='__main__':unittest.main()
