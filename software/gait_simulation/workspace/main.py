"""Run a minimal PyBullet simulation without loading URDF files."""

import pybullet as p
from pathlib import Path
import math

from simulator.simulator import Simulator

def main() -> None:
    try:
        workspace_root = Path(__file__).resolve().parent
        robot_urdf = workspace_root / "simulation" / "yertle.urdf"

        with Simulator(fps=240, use_gui=True) as simulator:

            # Load plane urdf
            simulator.load_urdf("plane.urdf", use_fixed_base=True)

            # Loadf the quadruped robot model named yertle
            robot = simulator.load_urdf(
                robot_urdf,
                base_position=(0.0, 0.0, 0.35),
            )

            print("- joints limit angles")
            for joint_index, joint_limit in robot.get_motor_angle_limits().items():
                print(f"{joint_index} : "
                      f"min({math.degrees(joint_limit[0]) : 0.2f}), "
                      f"max({math.degrees(joint_limit[1]) : 0.2f})")

            # set the init angle of each motor
            initial_angles = [
                0.0, -0.5, 1.0,  # lf
                0.0, -0.5, 1.0,  # rf
                0.0, -0.5, 1.0,  # lb
                0.0, -0.5, 1.0,  # rb
            ]

            for motor_index, motor_angle in zip(robot.get_motor_indices(), initial_angles):
                robot.reset_motor_angle(motor_index, motor_angle)

            max_init_force = 20

            robot.set_multiple_motors_angle(
                tuple(zip(
                    robot.get_motor_indices(), 
                    initial_angles,
                    [max_init_force] * len(initial_angles),
                    strict=True
                ))
            )

            print("Simulation is running. Press Ctrl+C to stop.")
            while simulator.is_running():
                simulator.step()

    except KeyboardInterrupt:
        print("Stopping the simulation.")


if __name__ == "__main__":
    main()
