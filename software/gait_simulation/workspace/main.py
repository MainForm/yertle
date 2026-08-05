"""Run a minimal PyBullet simulation without loading URDF files."""

import pybullet as p

from simulation.simulation import Simulation

def main() -> None:
    try:
        with Simulation(fps=240, use_gui=True) as simulation:

            print("Simulation is running. Press Ctrl+C to stop.")
            while simulation.is_running():
                simulation.step()
                
    except KeyboardInterrupt:
        print("Stopping the simulation.")


if __name__ == "__main__":
    main()
