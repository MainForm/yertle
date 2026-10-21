"""Profile-only torque comparison; do not mutate historical shared defaults."""
from decimal import Decimal
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]

class V622ProfileTests(unittest.TestCase):
    def profile(self,name):return json.loads((ROOT/f'configs/{name}_training.json').read_text())

    def test_v622_changes_only_physical_effort_and_velocity_from_v6216(self):
        old=self.profile('v6_21_6')['train_args'].copy()
        old[old.index('--velocity-limit-sim')+1]='2.3'
        old[old.index('--effort-limit-sim')+1]='0.64'
        self.assertEqual(old,self.profile('v6_22')['train_args'])

    def test_v6222_changes_only_effort_from_v622(self):
        old=self.profile('v6_22')['train_args'].copy()
        new=self.profile('v6_22_2')
        torque=Decimal(old[old.index('--effort-limit-sim')+1])*Decimal('1.6')
        self.assertEqual(torque,Decimal('1.024'))
        old[old.index('--effort-limit-sim')+1]=str(torque)
        self.assertEqual(old,new['train_args'])
        self.assertTrue(new['fresh']);self.assertNotIn('--checkpoint',new['train_args'])
        self.assertEqual(old[old.index('--max_iterations')+1],'7000')
        self.assertIn('not a measured',new['pwm_assumption'])

if __name__=='__main__':unittest.main()
