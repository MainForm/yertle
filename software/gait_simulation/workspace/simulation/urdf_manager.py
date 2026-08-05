"""Internal manager for URDF object lifetime and registration."""

from pathlib import Path
from typing import TypeVar

import pybullet as p

from .urdf_object import URDFObject

class URDFManager:
    """Create, register, and remove URDF objects for one physics client."""

    def __init__(self, physics_client: int) -> None:
        self._physics_client = physics_client
        self._objects: dict[int, URDFObject] = {}
        self._closed = False

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("URDFManager is already closed.")

    def load(
        self,
        path: str | Path,
        base_position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        base_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        use_fixed_base: bool = False,
        global_scaling: float = 1.0,
    ) -> URDFObject:
        """Load and register an instance of object_class."""
        self._ensure_open()
        if global_scaling <= 0:
            raise ValueError("global_scaling must be greater than 0.")

        body_id = p.loadURDF(
            fileName=str(path),
            basePosition=base_position,
            baseOrientation=base_orientation,
            useFixedBase=use_fixed_base,
            globalScaling=global_scaling,
            physicsClientId=self._physics_client,
        )

        try:
            obj = URDFObject(
                physics_client=self._physics_client,
                body_id=body_id,
                path=path,
            )
        except Exception:
            p.removeBody(body_id, physicsClientId=self._physics_client)
            raise

        self._objects[body_id] = obj
        return obj

    def remove(self, obj: URDFObject) -> None:
        """Remove one registered object from the PyBullet world."""
        self._ensure_open()
        body_id = obj.body_id
        if self._objects.get(body_id) is not obj:
            raise ValueError("This URDF object is not registered with this Simulation.")

        p.removeBody(body_id, physicsClientId=self._physics_client)
        del self._objects[body_id]
        obj._invalidate()
    
    def close(self) -> None:
        """Invalidate every handle before the physics client disconnects."""
        if self._closed:
            return

        for obj in self._objects.values():
            obj._invalidate()

        self._objects.clear()
        self._closed = True