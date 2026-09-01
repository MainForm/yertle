"""Base class for objects loaded into a PyBullet simulation."""

from pathlib import Path
from typing import Sequence

import pybullet as p


class URDFObject:
    """A handle to one URDF body owned by a Simulator."""

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

        self._joints_info = self._get_all_joint_info()


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
        tuple[float, float, float],             # position
        tuple[float, float, float, float],      # orientation
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

    def _ensure_joint_index(self, index):
        if index not in [info[0] for info in self._joints_info]:
            raise ValueError(f"Invalid joint index: {index}")

    def _ensure_motor_index(self, index):
        if index not in [info[0] for info in self._joints_info if info[2] == p.JOINT_REVOLUTE]:
            raise ValueError(f"Invalid motor index: {index}")

    def _ensure_motor_angles(
        self,
        target_joint_angles: Sequence[tuple[int, float]],  # int : joint index, float : joint target angle
    ) -> list[tuple[int,float]]:
        self._ensure_valid()

        # Type hints are not enforced at runtime, so convert each value to float.
        input_joint_angles = [
            (int(joint_index), float(joint_angle)) 
            for joint_index, joint_angle in target_joint_angles
        ]

        joints_limits = self.get_motor_angle_limits()

        for target_index, target_angle in input_joint_angles:
            if target_index not in joints_limits:
                raise ValueError(f"Invalid motor index: {target_index}")

            min_limit, max_limit = joints_limits[target_index]

            if min_limit > max_limit:
                raise ValueError(f"min_limit({min_limit}) value is upper then max_limit({max_limit})")

            if not (min_limit <= target_angle <= max_limit):
                raise ValueError(f"target_angle({target_angle}) is not included in "
                                 f" limit_range(min : {min_limit}, max : {max_limit})")

        return input_joint_angles        
    
    # joint methods
    def _get_all_joint_info(self) -> list[Sequence]:
        self._ensure_valid()

        joints_info : list[Sequence] = []
        joints_count = p.getNumJoints(
            self._body_id,
            physicsClientId=self._physics_client,
        )

        for joint_index in range(joints_count):
            joint_info = p.getJointInfo(
                self._body_id,
                joint_index,
                physicsClientId=self._physics_client,
            )

            joints_info.append(joint_info)

        return joints_info

    def get_joint_state(self,joint_index) -> list:
        self._ensure_valid()
        self._ensure_joint_index(joint_index)

        return p.getJointState(self._body_id, joint_index)

    def get_motor_indices(self) -> tuple[int, ...]:
        self._ensure_valid()
        return tuple(
            joint[0]
            for joint in self._joints_info
            if joint[2] == p.JOINT_REVOLUTE
        )

    def get_motor_angle_limits(self) -> dict[int, tuple[float, float]]:
        # 0 : joint index, 8 : joint min limit, 9 : joint max limit
        return {
            info[0]: (info[8], info[9]) 
            for info in self._joints_info 
            if info[2] == p.JOINT_REVOLUTE
        }



    def reset_motor_angle(
        self,
        motor_index : int,
        target_angle : float
    ) -> None:
        
        self._ensure_motor_angles([(motor_index, target_angle)])

        p.resetJointState(
            self._body_id,
            motor_index,
            targetValue = target_angle,
            physicsClientId=self._physics_client,
        )

    def set_motor_angle(self, index, angle, force):
        self._ensure_valid()
        self._ensure_motor_index(index)
        self._ensure_motor_angles([(
            index, angle
        )])

        p.setJointMotorControl2(
            bodyUniqueId=self._body_id,
            jointIndex=index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=force,
            physicsClientId=self._physics_client,
        )

    def set_multiple_motors_angle(
        self, 
        motor_controls : Sequence[tuple[int,float,float]] # int : motor_index, float : target_angle, float : force
    ) -> None:
        self._ensure_valid()

        motor_indices = self.get_motor_indices()

        if len(motor_indices) != len(motor_controls):
            raise ValueError(f"motor_controls counts{len(motor_controls)} is not match with count of loaded urdf model{len(motor_indices)}")
        
        self._ensure_motor_angles([
            (motor_index, target_angle) 
            for motor_index, target_angle, _ in motor_controls
        ])

        target_angles = [
            target_angle
            for _, target_angle, _ in motor_controls
        ]

        motor_forces = [
            force
            for _, _, force in motor_controls
        ]

        p.setJointMotorControlArray(
            bodyUniqueId =      self._body_id,
            jointIndices =      motor_indices,
            controlMode =       p.POSITION_CONTROL,
            targetPositions =   target_angles,
            forces =            motor_forces,
            physicsClientId=self._physics_client,
        )

