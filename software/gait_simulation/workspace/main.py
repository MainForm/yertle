"""Run a minimal PyBullet simulation without loading URDF files."""

import pybullet as p
from pathlib import Path
import math
import numpy as np


from simulator.simulator import Simulator, KeyEvent, Key
from simulator.yertle import Yertle

from simulator import gait_manager
   

def main() -> None:
    try:
        workspace_root = Path(__file__).resolve().parent
        robot_urdf = workspace_root / "simulation" / "yertle.urdf"

        with Simulator(fps=240, use_gui=True) as simulator:

            # Load plane urdf
            simulator.load_urdf("plane.urdf", use_fixed_base=True)

            # Load the quadruped robot model named yertle
            yertle = Yertle(
                simulator.load_urdf(
                    robot_urdf,
                    base_position=(0.0, 0.0, 0.35),
                    base_orientation=(0.0, 0.0, 0.0, 1),
                )
            )
            
            print("Simulation is running. Press Ctrl+C to stop.")

            walking = False

            def start_walking() -> None:
                nonlocal walking  # main() 내부에 정의한 함수인 경우
                if walking == False:
                    yertle.move(gait_manager.Direction.FORWARD)

                walking = True

            def stop_walking() -> None:
                nonlocal walking

                if walking == True:
                    yertle.stop()
                    yertle.set_leg_standing_angles()
                    
                walking = False

            simulator.register_key_callback(
                Key.UP, KeyEvent.PRESSED, start_walking,
            )
            simulator.register_key_callback(
                Key.UP, KeyEvent.RELEASED, stop_walking,
            )

            while simulator.is_running():
                yertle.update_gait(1.0 / simulator.fps)
                simulator.step()

    except KeyboardInterrupt:
        print("Stopping the simulation.")


if __name__ == "__main__":
    main()
