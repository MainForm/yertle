"""Run a minimal PyBullet simulation without loading URDF files."""

import pybullet as p
from pathlib import Path
import math
import numpy as np

from simulator.simulator import Simulator

def legs_movement(radian : float) -> float:
    if radian <= math.pi:
        return radian

    return (2 * math.pi) - radian

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
                base_orientation=(0.0, 90.0, 0.0, 1),
            )

            print("- joints limit angles")
            for joint_index, joint_limit in robot.get_motor_angle_limits().items():
                print(f"{joint_index} : "
                      f"min({math.degrees(joint_limit[0]) : 0.2f}), "
                      f"max({math.degrees(joint_limit[1]) : 0.2f})")

            motor_limits = robot.get_motor_angle_limits()

            shoulder_min_angle = motor_limits[1][0]
            shoulder_max_angle = motor_limits[1][1]
            thigh_min_angle = motor_limits[2][0]
            thigh_max_angle = motor_limits[2][1]
            shin_min_angle = motor_limits[3][0]
            shin_max_angle = motor_limits[3][1]
            
            # unit : radian
            cur_phase = 0.0
            #  unit : second
            time_step = 1.0 / 240.0
            # unit : radian / Hz
            a_cycle = 2.0 * math.pi # (Radians per cycle)
            # unit : Hz / second
            freq = 0.5
            
            print("Simulation is running. Press Ctrl+C to stop.")
            while simulator.is_running():
                cur_angle = np.interp(legs_movement(cur_phase), [0, math.pi],[shoulder_min_angle, shoulder_max_angle])

                robot.set_motor_angle(1,cur_angle, 20)
                
                cur_phase = (cur_phase + (a_cycle * freq * time_step)) % a_cycle
                simulator.step()

    except KeyboardInterrupt:
        print("Stopping the simulation.")


if __name__ == "__main__":
    main()
