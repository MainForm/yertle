"""Standalone tensor tests; no Kit or GPU required."""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import torch

ROOT = Path(__file__).resolve().parents[1]
package = ModuleType('travel_test')
package.__path__ = [str(ROOT)]
managers = ModuleType('isaaclab.managers')
class Base:
    def __init__(self, cfg, env):
        pass
managers.ManagerTermBase = Base
managers.SceneEntityCfg = object
with patch.dict(sys.modules, {'travel_test': package, 'isaaclab.managers': managers}):
    spec = importlib.util.spec_from_file_location('travel_test.motor_travel_balance', ROOT/'motor_travel_balance.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    phase_spec = importlib.util.spec_from_file_location('travel_phase', ROOT/'phase_generator.py')
    phase = importlib.util.module_from_spec(phase_spec)
    phase_spec.loader.exec_module(phase)

NAMES = [f'{leg}_{kind}' for kind in ('shoulder', 'thigh', 'shin') for leg in ('lb','lf','rb','rf')]

class TravelTests(unittest.TestCase):
    def setUp(self):
        self.q = torch.zeros(2, 12)
        self.command = torch.tensor([[.1,0,0],[.1,0,0]])
        self.asset_cfg = NS(name='robot')
        self.env = NS(scene={'robot': NS(joint_names=NAMES, data=NS(joint_pos=self.q))},
                      device='cpu', num_envs=2, step_dt=.02, common_step_counter=0,
                      command_manager=NS(get_command=lambda name:self.command))
        self.params = dict(asset_cfg=self.asset_cfg, command_name='base_velocity', window_s=.04, travel_floor_rad=.01)
        self.term = module.MotorTravelBalancePenalty(NS(params=self.params), self.env)
        self.call()

    def call(self):
        return self.term(self.env, **self.params).clone()

    def step(self, thigh=0, shin=0):
        for leg in ('lf','rf','lb','rb'):
            self.q[:, NAMES.index(leg+'_thigh')] += thigh
            self.q[:, NAMES.index(leg+'_shin')] += shin
        self.env.common_step_counter += 1
        return self.call()

    def test_same_servo_travel_zero_cost_despite_stationary_anatomical_shin(self):
        torch.testing.assert_close(self.step(.1,0), torch.zeros(2), atol=1e-8, rtol=0)

    def test_linkage_cancellation_is_full_imbalance(self):
        torch.testing.assert_close(self.step(.1,-.1), torch.ones(2), atol=2e-5, rtol=0)

    def test_known_two_to_one_ratio(self):
        torch.testing.assert_close(self.step(.1,.1), torch.full((2,),1/9), atol=2e-5, rtol=0)

    def test_asynchronous_equal_travel_in_window(self):
        self.step(.1,-.1)  # thigh only servo motion
        torch.testing.assert_close(self.step(0,.1), torch.zeros(2), atol=1e-8, rtol=0)

    def test_old_travel_expires(self):
        self.step(.1,-.1)
        self.step()
        torch.testing.assert_close(self.step(), torch.zeros(2))

    def test_partial_reset_ignores_teleport_other_env_keeps_history(self):
        self.step(.1,-.1)
        self.term.reset(torch.tensor([0]))
        self.q[0] += 100
        self.env.common_step_counter += 1
        torch.testing.assert_close(self.call(), torch.tensor([0.,1.]), atol=2e-5, rtol=0)

    def test_repeated_evaluation_does_not_advance_history(self):
        expected=self.step(.1,-.1)
        cursor=self.term.cursor.clone()
        torch.testing.assert_close(self.call(), expected)
        torch.testing.assert_close(self.term.cursor, cursor)

    def test_zero_command_clears_history_and_zero_motion_no_bonus(self):
        self.step(.1,-.1)
        self.command.zero_()
        torch.testing.assert_close(self.step(), torch.zeros(2))
        self.assertEqual(self.term.history.count_nonzero().item(),0)

    def test_pure_yaw_is_active(self):
        self.command[:] = torch.tensor([0,0,.2])
        torch.testing.assert_close(self.step(.1,-.1), torch.ones(2), atol=2e-5, rtol=0)

    def test_tiny_motion_is_regularized(self):
        result=self.step(.0001,-.0001)
        self.assertTrue(torch.all(result < .001))

    def test_state_tensor_roundtrip_produces_same_next_result(self):
        self.step(.1,-.1)
        restored=module.MotorTravelBalancePenalty(NS(params=self.params), self.env)
        for key,value in vars(self.term).items():
            if isinstance(value, torch.Tensor):
                getattr(restored,key).copy_(value)
        self.step(0,.05)
        torch.testing.assert_close(restored(self.env, **self.params), self.term.value)

    def test_one_hz_clock_and_legacy_default(self):
        env=NS(num_envs=1, device='cpu', episode_length_buf=torch.tensor([25]), step_dt=.02,
               command_manager=NS(get_command=lambda name:torch.tensor([[.1,0,0]])), cfg=NS(trot_frequency_hz=1.0))
        state=phase._gait_state(env,'base_velocity')
        torch.testing.assert_close(state[0],torch.tensor([.5]))
        torch.testing.assert_close(state[2],torch.tensor([1.]))
        self.assertFalse(phase._desired_contact(env,'base_velocity')[0,0].item())
        del env.cfg.trot_frequency_hz
        torch.testing.assert_close(phase._gait_state(env,'base_velocity')[2],torch.tensor([2.2]))

if __name__ == '__main__':
    unittest.main()
