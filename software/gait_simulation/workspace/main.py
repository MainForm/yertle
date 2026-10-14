"""Run a PyBullet simulation with a ground plane and the Yertle robot."""

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

            # Load the ground plane as a fixed base.
            simulator.load_urdf("plane.urdf", use_fixed_base=True)

            # Load the Yertle quadruped robot model.
            yertle = Yertle(
                simulator.load_urdf(
                    robot_urdf,
                    base_position=(0.0, 0.0, 0.35),
                    base_orientation=(0.0, 0.0, 0.0, 1),
                )
            )
            
            print("Simulation is running. Press Ctrl+C to stop.")

            # Start walking when the up arrow key is pressed.
            def start_walking() -> None:

                # Start walking only if the robot is currently stopped.
                if yertle.get_gait_direction() is None:
                    yertle.move(gait_manager.Direction.FORWARD)

            # Stop walking when the up arrow key is released.
            def stop_walking() -> None:

                # Restore the standing pose only if the robot is walking.
                if yertle.get_gait_direction() is not None:
                    yertle.stop()
                    yertle.set_leg_standing_angles()

            # Register the key callbacks with the simulator.
            simulator.register_key_callback(
                Key.UP, KeyEvent.PRESSED, start_walking,
            )
            simulator.register_key_callback(
                Key.UP, KeyEvent.RELEASED, stop_walking,
            )

            while simulator.is_running():
                # Update the leg angles for the current gait.
                yertle.update_gait(1.0 / simulator.fps)

                simulator.step()

    except KeyboardInterrupt:
        print("Stopping the simulation.")


if __name__ == "__main__":
    main()
