"""Base class for objects loaded into a PyBullet simulation."""

from pathlib import Path

import pybullet as p


class URDFObject:
    """A handle to one URDF body owned by a Simulation."""

    def __init__(
        self,
        physics_client: int,
        body_id: int,
        path: str | Path,
    ) -> None:
        self._physics_client = physics_client
        self._body_id = body_id
        self._path = path
        self._is_valid = True

    @property
    def body_id(self) -> int:
        """Return the PyBullet body ID assigned to this object."""
        self._ensure_valid()
        return self._body_id

    @property
    def path(self) -> str | Path:
        """Return the URDF path used to create this object."""
        return self._path

    @property
    def is_valid(self) -> bool:
        """Return whether this object still belongs to a live simulation."""
        return self._is_valid

    def get_base_pose(
        self,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float, float],
    ]:
        """Return the base position and quaternion orientation."""
        self._ensure_valid()
        position, orientation = p.getBasePositionAndOrientation(
            self._body_id,
            physicsClientId=self._physics_client,
        )
        return tuple(position), tuple(orientation)

    def _invalidate(self) -> None:
        """Mark the handle invalid after its owner removes or closes it."""
        self._is_valid = False

    def _ensure_valid(self) -> None:
        if not self._is_valid:
            raise RuntimeError("This URDF object has already been removed or invalidated.")
