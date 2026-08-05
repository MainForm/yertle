"""PyBullet simulation lifecycle and public object-management API."""

import time
from pathlib import Path

import pybullet as p
import pybullet_data

from .urdf_manager import URDFManager
from .urdf_object import URDFObject

class Simulation:
    # ---------------------------------------------------------------------------
    # region Context Management

    def __init__(
        self,
        gravity: float = -9.81,
        fps: int = 240,
        use_gui: bool = True,
    ) -> None:
        if fps <= 0:
            raise ValueError("fps must be greater than 0.")

        connection_mode = p.GUI if use_gui else p.DIRECT
        self._physics_client = p.connect(connection_mode)
        if self._physics_client < 0:
            raise RuntimeError("Failed to connect to PyBullet.")

        self._fps = fps
        self._closed = False

        self._urdf_manager = URDFManager(self._physics_client)

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath(),
            physicsClientId=self._physics_client,
        )
        p.setGravity(0, 0, gravity, physicsClientId=self._physics_client)
        p.setTimeStep(1.0 / fps, physicsClientId=self._physics_client)


    # __enter__ and __exit__ support the with statement.
    def __enter__(self) -> "Simulation":
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    # endregion
    # ---------------------------------------------------------------------------


    # ---------------------------------------------------------------------------
    # region Private Methods

    def _ensure_open(self) -> None:
        if not self.is_running():
            raise RuntimeError("Simulation is already closed.")

    # endregion
    # ---------------------------------------------------------------------------


    # ---------------------------------------------------------------------------
    # region Public Methods
    @property
    def client_id(self) -> int:
        """Return the ID of the owned PyBullet connection."""
        self._ensure_open()
        return self._physics_client

    def step(self) -> None:
        """Advance the physics world by one configured time step."""
        p.stepSimulation(physicsClientId=self._physics_client)
        time.sleep(1.0 / self._fps)

    def is_running(self) -> bool:
        """Return whether the PyBullet connection is still active."""
        return not self._closed and bool(p.isConnected(self._physics_client))

    def close(self) -> None:
        """Invalidate all objects and close the owned PyBullet world."""
        if self._closed:
            return
        
        self._urdf_manager.close()

        if p.isConnected(self._physics_client):
            p.disconnect(self._physics_client)
        self._closed = True

    # URDF Management methods

    def load_urdf(
        self,
        path: str | Path,
        base_position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        base_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        use_fixed_base: bool = False,
        global_scaling: float = 1.0,
    ) -> URDFObject:
        """Load a general URDF object and transfer ownership to Simulation."""
        self._ensure_open()
        return self._urdf_manager.load(
            path,
            base_position,
            base_orientation,
            use_fixed_base,
            global_scaling,
        )

    def remove_urdf(self, obj: URDFObject) -> None:
        """Remove one object owned by this simulation."""
        self._ensure_open()
        self._urdf_manager.remove(obj)

    # endregion
    # ---------------------------------------------------------------------------