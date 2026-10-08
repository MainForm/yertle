import math

from . import gait_manager
from .gait_manager import GaitManager

from .urdf_object import URDFObject

from enum import Enum

def convert_polar_to_cartesian(length: float, angle: float) -> tuple[float, float]:
    '''Convert length (m) and angle (rad) to an (x, y) vector. Angles start at -Y and turn toward -X when positive.'''
    return (-length * math.sin(angle), -length * math.cos(angle))

class LegType(Enum):
    LEFT_FRONT = 0
    RIGHT_FRONT = 1
    LEFT_BACK = 2
    RIGHT_BACK = 3

class LegJointType(Enum):
    SHOULDER = 0
    THIGH = 1
    SHIN = 2

class Yertle:
    # Maximum foot lift height.
    FOOT_ELEVATION = 0.015
    # Maximum foot displacement along the x-axis.
    X_STRIDE = 0.04

    # Left Front, Rright Front, Left Back, Rright Back
    LEG_COUNT = 4
    # shoulder, thigh, shin
    LEG_JOINT_COUNT = 3

    # unit : meter
    THIGH_LENGTH = 0.13     # 0.13 m == 13 cm
    SHIN_LENGTH = 0.13      # 0.13 m == 13 cm

    def __init__(self, robot_object: URDFObject, gait_frequency: float = 1.5):
        self._robot = robot_object

        self._initial_joint_angles = (
            math.radians(0.0),      # shoulder
            math.radians(-25.0),    # thigh
            math.radians(50.0),     # shin
        )

        self.gait_manager = GaitManager(
            gait_frequency,         # 1 second
            self.X_STRIDE,          # 7 cm
            self.FOOT_ELEVATION,    # 4 cm
            self.THIGH_LENGTH,      # 13 cm
            self.SHIN_LENGTH        # 13 cm
        )

        self._cur_joints_angle: list[float] = [0.0] * 12

        self.reset_leg_joints()

    def reset_leg_joints(self) -> None:
        """Reset all leg joints to the default standing pose and hold it."""

        # Joint index 0 is the fixed base-to-frame joint, so the first motor
        # starts at index 1. Each leg then occupies three consecutive indices.
        first_motor_index = 1

        for leg_index in range(self.LEG_COUNT):
            joint_start = (
                first_motor_index
                + leg_index * self.LEG_JOINT_COUNT
            )

            for joint_offset, angle in enumerate(self._initial_joint_angles):
                # Place the joint immediately in the standing pose, then enable
                # position control so that it continues to hold that angle.
                self._robot.reset_motor_angle(
                    joint_start + joint_offset,
                    angle,
                )

                self.set_leg_joint_angle(leg_index, joint_offset, angle)

    def set_leg_standing_angles(self) -> None:
        """Reset all leg joints to the default standing pose and hold it."""

        for leg_index in range(self.LEG_COUNT):
            for joint_offset, angle in enumerate(self._initial_joint_angles):
                self.set_leg_joint_angle(leg_index, joint_offset, angle)

        self.gait_manager.reset()

    def set_leg_joint_angle(
        self,
        leg_type : LegType, 
        joint_type : LegJointType, 
        angle : float,
        force : float = 20,
    ) -> None:
        leg_type = LegType(leg_type)
        joint_type = LegJointType(joint_type)

        joint_start = 1 + leg_type.value * self.LEG_JOINT_COUNT

        self._robot.set_motor_angle(
            joint_start + joint_type.value,
            angle,
            force
        )

        self._cur_joints_angle[joint_start + joint_type.value - 1] = angle

    def get_leg_joint_angle(
        self,
        leg_type : LegType, 
        joint_type : LegJointType, 
    ) -> None:
        joint_start = leg_type.value * self.LEG_JOINT_COUNT

        return self._cur_joints_angle[joint_start + joint_type.value]

    def move(self, direction: gait_manager.Direction) -> None:
        """Set the robot's walking direction."""

        self.gait_manager.reset()
        self.gait_manager.set_direction(direction)

        init_thigh_cartiesian = convert_polar_to_cartesian(
            self.THIGH_LENGTH,
            self._initial_joint_angles[LegJointType.THIGH.value]
        )

        init_shin_cartiesian = convert_polar_to_cartesian(
            self.SHIN_LENGTH,
             self._initial_joint_angles[LegJointType.THIGH.value] + self._initial_joint_angles[LegJointType.SHIN.value]
        )

        self._init_foot_pos = (
            init_thigh_cartiesian[0] + init_shin_cartiesian[0],
            init_thigh_cartiesian[1] + init_shin_cartiesian[1]
        )

    def stop(self) -> None:
        """Stop walking and reset the gait phase."""

        self.gait_manager.reset()

    def update_gait(self, delta_time) -> None:
        if self.gait_manager._direction is None:
            return

        lf_rb_next_pos, rf_lb_next_pos = self.gait_manager.step_gait(delta_time)

        lf_rb_target_pos = (
            lf_rb_next_pos[0] + self._init_foot_pos[0],
            lf_rb_next_pos[1] + self._init_foot_pos[1]
        )

        rf_lb_target_pos = (
            rf_lb_next_pos[0] + self._init_foot_pos[0],
            rf_lb_next_pos[1] + self._init_foot_pos[1]
        )

        lf_rb_angles = self._calculate_inverse_kinematics(
            lf_rb_target_pos,
            (self.THIGH_LENGTH, self.get_leg_joint_angle(
                LegType.LEFT_FRONT, 
                LegJointType.THIGH )),
            (self.SHIN_LENGTH, self.get_leg_joint_angle(
                LegType.LEFT_FRONT, 
                LegJointType.SHIN )),
        )
        
        rf_lb_angles = self._calculate_inverse_kinematics(
            rf_lb_target_pos,
            (self.THIGH_LENGTH, self.get_leg_joint_angle(
                LegType.RIGHT_FRONT, 
                LegJointType.THIGH )),
            (self.SHIN_LENGTH, self.get_leg_joint_angle(
                LegType.RIGHT_FRONT, 
                LegJointType.SHIN )),
        )

        self.set_leg_joint_angle(LegType.LEFT_FRONT, LegJointType.THIGH, lf_rb_angles[0])
        self.set_leg_joint_angle(LegType.LEFT_FRONT, LegJointType.SHIN, lf_rb_angles[1])

        self.set_leg_joint_angle(LegType.RIGHT_BACK, LegJointType.THIGH, lf_rb_angles[0])
        self.set_leg_joint_angle(LegType.RIGHT_BACK, LegJointType.SHIN, lf_rb_angles[1])

        self.set_leg_joint_angle(LegType.RIGHT_FRONT, LegJointType.THIGH, rf_lb_angles[0])
        self.set_leg_joint_angle(LegType.RIGHT_FRONT, LegJointType.SHIN, rf_lb_angles[1])

        self.set_leg_joint_angle(LegType.LEFT_BACK, LegJointType.THIGH, rf_lb_angles[0])
        self.set_leg_joint_angle(LegType.LEFT_BACK, LegJointType.SHIN, rf_lb_angles[1])

    def _calculate_inverse_kinematics(
        self,
        target: tuple[float, float],            # (x, y)
        base_thigh_polar: tuple[float, float],  # (length, radian)
        base_shin_polar: tuple[float, float],   # (length, radian)
    ) -> tuple[float, float]:                   # (thigh_radian, shin_radian)

        thigh_length, initial_thigh_radians = base_thigh_polar
        shin_length, initial_shin_radians = base_shin_polar
        target_x, target_y = target

        if not all(math.isfinite(value) for value in (
            target_x, target_y, thigh_length, shin_length, initial_thigh_radians, initial_shin_radians
        )):
            raise ValueError('Target coordinates, link lengths, and initial angles must be finite.')
        if thigh_length <= 0.0 or shin_length <= 0.0:
            raise ValueError('Link lengths must be positive.')

        # The foot must lie within the annulus reachable by the two links.
        distance = math.hypot(target_x, target_y)
        if distance > thigh_length + shin_length + 1e-12 or distance < abs(thigh_length - shin_length) - 1e-12:
            raise ValueError('Target is outside the reachable range of the leg.')

        # The cosine rule gives the relative knee angle, not the triangle's interior angle.
        cos_q2 = (distance**2 - thigh_length**2 - shin_length**2) / (2 * thigh_length * shin_length)
        cos_q2 = max(-1.0, min(1.0, cos_q2))
        bend_sign = -1.0 if math.sin(initial_shin_radians) < 0.0 else 1.0
        q2 = bend_sign * math.acos(cos_q2)

        # alpha points toward the target; beta is the offset from thigh to target.
        alpha = math.atan2(-target_x, -target_y)
        # Resolve the target vector along and perpendicular to the thigh.
        beta = math.atan2(
            shin_length * math.sin(q2),
            thigh_length + shin_length * math.cos(q2),
        )
        q1 = alpha - beta

        # Choose equivalent angles near the initial pose to avoid full-turn jumps.
        thigh_radians = initial_thigh_radians + (q1 - initial_thigh_radians + math.pi) % (2 * math.pi) - math.pi
        shin_radians = initial_shin_radians + (q2 - initial_shin_radians + math.pi) % (2 * math.pi) - math.pi
        return thigh_radians, shin_radians