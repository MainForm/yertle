from .phase_manager import PhaseManager

import math
from enum import Enum


# Supported walking directions.
class Direction(Enum):
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3

class GaitManager:

    def __init__(
        self, 
        frequency : float,          # required time for one step 
        x_stride : float,           # stride to move foot forward and backword
        foot_elevation : float,     # elevation of max foot height
        thigh_length : float,
        shin_length : float,
    ) -> None:
        self._x_stride = x_stride
        self._foot_elevation = foot_elevation

        self._thigh_length = thigh_length
        self._shin_length = shin_length

        self.phase_manager = PhaseManager(frequency)

        # if direction is None, that means standing
        self._direction: Direction | None = None

    def set_direction(self, direction: Direction) -> None:
        """Set the robot's walking direction."""

        self._direction = direction

    def reset(self) -> None:
        """Stop walking and reset the gait phase."""

        self._direction = None

        self.phase_manager.reset(0.0)

    def step_gait(
        self,
        delta_time
    ) -> tuple[
        tuple[float,float],         # LF/RB foot potition
        tuple[float,float]          # RF/LB foot position
    ]:                       # when occur error, return None
        if self._direction is None:
            raise RuntimeError("Direction is not set.")

        if delta_time <= 0:
            raise RuntimeError("delta time can not be lower or equal with 0")

        # Advance the gait phase by the elapsed time.
        cur_phase = self.phase_manager.step(delta_time)

        if self._direction is Direction.FORWARD:
            # Calculate the forward displacement of the left front and right rear feet.
            lf_rb_foot_forward = self._x_stride * math.cos(cur_phase)
            # Calculate their lift height, clamping it to zero during stance.
            lf_rb_foot_height = max(
                0.0,
                self._foot_elevation * math.sin(cur_phase),
            )

            rf_lb_foot_forward = -self._x_stride * math.cos(cur_phase)
            rf_lb_foot_height = max(
                0.0,
                -self._foot_elevation * math.sin(cur_phase),
            )

            return (
                (lf_rb_foot_forward, lf_rb_foot_height),
                (rf_lb_foot_forward, rf_lb_foot_height),
            )

        raise RuntimeError("Direction is set to error value")