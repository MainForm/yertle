import unittest
import torch
from isaac_lab.motor_drive_math import joint_to_motor_velocity, joint_to_motor_effort, motor_to_joint_effort, limit_motor_effort


class MotorDriveMathTests(unittest.TestCase):
    def test_mapping_and_virtual_work(self):
        qd = torch.tensor([[0.5, 1.0, -0.25]])
        tau = torch.tensor([[0.2, 0.7, -0.4]])
        motor_v = joint_to_motor_velocity(qd, [1], [2])
        motor_tau = joint_to_motor_effort(tau, [1], [2])
        torch.testing.assert_close(motor_v, torch.tensor([[0.5, 1.0, 0.75]]))
        torch.testing.assert_close(motor_tau, torch.tensor([[0.2, 1.1, -0.4]]))
        torch.testing.assert_close(motor_to_joint_effort(motor_tau, [1], [2]), tau)
        torch.testing.assert_close((motor_v * motor_tau).sum(), (qd * tau).sum())

    def test_torque_and_directional_speed_limit(self):
        tau = torch.tensor([[2., -2., 0.7, -0.7, -0.5, 0.5]])
        speed = torch.tensor([[0., 0., 2.3, -2.3, 2.4, -2.4]])
        expected = torch.tensor([[1.024, -1.024, 0., 0., -0.5, 0.5]])
        torch.testing.assert_close(limit_motor_effort(tau, speed, 1.024, 2.3), expected)

    def test_coupled_output_can_exceed_single_motor_torque(self):
        result = motor_to_joint_effort(torch.tensor([[1.024, 1.024]]), [0], [1])
        torch.testing.assert_close(result, torch.tensor([[2.048, 1.024]]))


if __name__ == '__main__':
    unittest.main()
