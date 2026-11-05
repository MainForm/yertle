"""Five-millisecond acceleration and per-environment reset checks."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import torch

ROOT = Path(__file__).resolve().parents[1]
package = ModuleType('acceleration_test')
package.__path__ = [str(ROOT)]
managers = ModuleType('isaaclab.managers')
class ManagerTermBase:
    def __init__(self, cfg, env):
        self.cfg, self._env = cfg, env
managers.ManagerTermBase = ManagerTermBase

with patch.dict(sys.modules, {'acceleration_test': package, 'isaaclab.managers': managers}):
    spec = importlib.util.spec_from_file_location('acceleration_test.motor_control_acceleration', ROOT/'motor_control_acceleration.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


class AccelerationTests(unittest.TestCase):
    def test_control_step_and_partial_reset(self):
        names = [f'{leg}_{joint}' for leg in ('lf','rf','lb','rb') for joint in ('shoulder','thigh','shin')]
        velocity = torch.zeros(2, 12)
        robot = NS(joint_names=names, data=NS(joint_vel=velocity), actuators={'legs': NS()})
        env = NS(scene={'robot': robot}, step_dt=0.020, physics_dt=0.001, _sim_step_counter=0)
        asset = NS(name='robot')
        term = module.MotorControlIntervalAcceleration(NS(params={'asset_cfg': asset}), env)
        for step in range(1, 6):
            env._sim_step_counter = step
            term.record_physics_velocity(velocity)
            velocity[:, 1] += 0.001
        # 0.005 rad/s change over 5 ms -> 1 rad/s² on thigh and linked shin.
        torch.testing.assert_close(term(env, asset), torch.tensor([2.0, 2.0]))
        term.reset(torch.tensor([0]))
        env._sim_step_counter = 6
        term.record_physics_velocity(velocity)
        velocity[:, 1] += 0.001
        torch.testing.assert_close(term(env, asset), torch.tensor([0.0, 2.0]))
        for step in range(7, 11):
            env._sim_step_counter = step
            term.record_physics_velocity(velocity)
            velocity[:, 1] += 0.001
        torch.testing.assert_close(term(env, asset), torch.tensor([2.0, 2.0]))


if __name__ == '__main__':
    unittest.main()
