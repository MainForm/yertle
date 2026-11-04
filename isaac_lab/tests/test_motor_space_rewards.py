import unittest
from types import SimpleNamespace as NS
import torch
from isaac_lab.motor_space_rewards import motor_torques_l2, motor_velocity_l2, motor_acc_l2, motor_velocity_excess, motor_energy_l1


class MotorRewardTests(unittest.TestCase):
    def test_linked_motor_reward_quantities(self):
        names = [f'{leg}_{part}' for leg in ('lf', 'rf', 'lb', 'rb') for part in ('shoulder', 'thigh', 'shin')]
        torque = torch.zeros(1, 12); velocity = torque.clone(); acceleration = torque.clone()
        torque[0, 1:3] = torch.tensor([0.7, -0.4])
        velocity[0, 1:3] = torch.tensor([1.0, 2.0])
        acceleration[0, 1:3] = torch.tensor([1.0, 2.0])
        env = NS(scene={'robot': NS(joint_names=names, data=NS(applied_torque=torque, joint_vel=velocity, joint_acc=acceleration))})
        asset = NS(name='robot')
        self.assertAlmostEqual(motor_torques_l2(env, asset).item(), 1.37, places=5)
        self.assertAlmostEqual(motor_velocity_l2(env, asset).item(), 10., places=5)
        self.assertAlmostEqual(motor_acc_l2(env, asset).item(), 10., places=5)
        self.assertAlmostEqual(motor_energy_l1(env, asset).item(), 2.3, places=5)
        self.assertAlmostEqual(motor_velocity_excess(env, 2., asset).item(), .25, places=5)


if __name__ == '__main__':
    unittest.main()
