"""Run a minimal PyBullet GUI simulation."""

import time

import pybullet as p
import pybullet_data


def main() -> None:
    client_id = p.connect(p.GUI)
    if client_id < 0:
        raise RuntimeError("Failed to connect to the PyBullet GUI")

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)

        p.loadURDF("plane.urdf")

        print("PyBullet simulation is running. Press Ctrl+C to stop.")
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("Stopping the simulation.")
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
