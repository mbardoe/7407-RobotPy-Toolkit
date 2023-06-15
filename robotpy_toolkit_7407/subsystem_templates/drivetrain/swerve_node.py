import math
from dataclasses import dataclass

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from robotpy_toolkit_7407 import PIDMotor
from robotpy_toolkit_7407.encoder import AbsoluteEncoder
from robotpy_toolkit_7407.utils.math import bounded_angle_diff, unit_normal
from robotpy_toolkit_7407.utils.units import meters, meters_per_second, \
    radians_per_second, radians


class SwerveNodeOffset:
    motor_sensor_offset: radians = 0
    motor_reversed_start: bool = False

    def __init__(self, offset: float, motor_reversed: bool = False):
        self.set_offset(offset, motor_reversed)

    def init(self) -> None:
        """

        Args:
            An init that can be overwritten.

        Returns: None

        """
        ...

    def change_direction(self) -> None:
        """
        Change the direction of the offset, and flip the direction.

        Returns: None

        """
        if self.motor_sensor_offset < math.pi / 2:
            self.motor_sensor_offset += math.pi
        else:
            self.motor_sensor_offset -= math.pi
        self.motor_reversed_start = not self.motor_reversed_start

    def get_offset(self) -> radians:
        """
        Get the current offset.

        Returns: the current offset in radians

        """
        return self.motor_sensor_offset

    def get_motor_direction(self) -> bool:
        """
        Get the direction of the desired direction of the powering motor.
        Returns: a boolean that represents

        """
        return self.motor_reversed_start

    def set_offset(self, offset: radians, motor_reversed: bool) -> None:
        """
        Given an offset and the direction of the motor set the current offset to
        that measurement. The motor reversed is updated if the current offset would
        be different given that it is between -pi/2 and pi/2.

        Args:
            offset:
            motor_reversed:

        Returns:

        """

        new_offset = math.fmod(offset, math.pi)
        if new_offset > math.pi / 2:
            new_offset -= math.pi
        number_pis = round((offset - new_offset) / math.pi)
        if number_pis % 2 == 1:
            self.motor_reversed_start = not motor_reversed
        else:
            self.motor_reversed_start = motor_reversed


@dataclass
class SwerveNodeConfig:
    node_x: meters
    node_y: meters
    turn_motor: PIDMotor
    drive_motor: PIDMotor
    encoder: AbsoluteEncoder
    drivetrain_turn_gear_ratio: float
    drivetrain_move_gear_ratio: float  # what units here?
    absolute_encoder_zeroed_pos: radians = 0
    motor_sensor_offset: radians = 0
    motor_reversed_start: bool = False


class SwerveNode:
    """
    Extendable class for swerve node.
    """
    node_x: meters
    node_y: meters
    tangent: tuple[meters, meters]
    motor_sensor_offset: SwerveNodeOffset
    turn_motor: PIDMotor
    drive_motor: PIDMotor
    encoder: AbsoluteEncoder
    drivetrain_turn_gear_ratio: float
    drivetrain_move_gear_ratio: float  # what units here?
    absolute_encoder_zeroed_pos: radians = 0

    def init(self, config: SwerveNodeConfig):
        """
        Initialize the swerve node.
        """
        self.node_x = config.node_x
        self.node_y = config.node_y
        self.tangent = unit_normal(self.node_x, self.node_y)
        self.turn_motor = config.turn_motor
        self.drive_motor = config.drive_motor
        self.encoder = config.encoder
        self.drivetrain_turn_gear_ratio = config.drivetrain_turn_gear_ratio
        self.drivetrain_move_gear_ratio = config.drivetrain_move_gear_ratio
        self.absolute_encoder_zeroed_pos = config.absolute_encoder_zeroed_pos
        self.motor_sensor_offset = SwerveNodeOffset(offset=config.motor_sensor_offset,
                                                    motor_reversed=config.motor_reversed_start)
        self.turn_motor.init()
        self.drive_motor.init()
        self.encoder.init()
        self.motor_sensor_offset.init()

    def initial_zero(self):
        current_pos_rad = (
                math.radians(self.encoder.getAbsolutePosition())
                - self.absolute_encoder_zeroed_pos
        )

        self.turn_motor.set_sensor_position(
            current_pos_rad * self.drivetrain_turn_gear_ratio / (2 * math.pi)
        )

        self.drive_motor.set_sensor_position(0)
        self.drive_motor.set_target_position(0)

    def set(self, vel: meters_per_second, angle_radians: radians):
        """
        Set the velocity and angle of the swerve node.

        Args:
            vel (meters_per_second): velocity of the swerve node
            angle_radians (radians): turning swerve node position in radians per second

        """
        # Previously
        # self._set_angle(angle_radians, self.get_turn_motor_angle() + self.motor_sensor_offset)
        self.turn_motor.set_target_position(self.revised_set_angle(angle_radians, self.get_turn_motor_angle()))
        self.set_motor_velocity(vel if not self.motor_reversed else -vel)

    def set_motor_angle(self, pos: radians):
        """
        Set the angle of the swerve node. Must be overridden.

        Args:
            pos (radians): angle of the swerve node in radians
        """
        self.turn_motor.set_target_position(
            (pos / (2 * math.pi)) * self.drivetrain_turn_gear_ratio
        )

    def get_turn_motor_angle(self) -> radians:
        """
        Get the current angle of the swerve node. Must be overridden. Must return radians.
        """
        return (
                (self.turn_motor.get_sensor_position() / self.drivetrain_turn_gear_ratio)
                * 2
                * math.pi
        )

    def set_motor_velocity(self, vel: meters_per_second):
        """
        Set the velocity of the swerve node. Must be overridden.
        Args:
            vel (meters_per_second): velocity of the swerve node in meters per second
        """
        self.drive_motor.set_target_velocity(vel * self.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> meters_per_second:
        """
        Get the velocity of the swerve node. Must be overridden. Must return meters per second.
        """
        return (
                self.drive_motor.get_sensor_velocity()
                / self.drivetrain_move_gear_ratio
        )

    def get_drive_motor_traveled_distance(self) -> meters:
        sensor_position = -1 * self.drive_motor.get_sensor_position()

        return (
                sensor_position
                / self.drivetrain_move_gear_ratio
        )

    def get_node_position(self) -> SwerveModulePosition:
        """
        Get the position of the swerve node.

        Returns:
            SwerveModulePosition: position of the swerve node
        """
        return SwerveModulePosition(
            self.get_drive_motor_traveled_distance(),
            Rotation2d(self.get_turn_motor_angle())
        )

    def get_node_state(self) -> SwerveModuleState:
        """
        Get the state of the swerve node.
        Returns:
            SwerveModuleState: state of the swerve node
        """
        return SwerveModuleState(
            self.get_motor_velocity(),
            Rotation2d(self.get_turn_motor_angle())
        )

    def revised_set_angle(self, target_angle: radians, initial_angle: radians) -> radians:
        """
        Determines the
        Args:
            target_angle:
            initial_angle:

        Returns:

        """
        diff = bounded_angle_diff(initial_angle, target_angle)
        if abs(diff) > 0.65 * math.pi:
            # We are going to flip direction of the offset
            flip_sensor_offset = math.copysign(math.pi, diff)
            diff -= flip_sensor_offset  # we want the other part of the turn rather than the one calculated
            new_target_angle = diff + initial_angle - self.motor_sensor_offset.get_offset()
            self.motor_sensor_offset.change_direction()
        else:
            new_target_angle = diff + initial_angle - self.motor_sensor_offset.get_offset()
        # self.set_motor_angle(new_target_angle)
        return new_target_angle

    def calculate(self, dx: meters_per_second, dy: meters_per_second,
                  d_theta: radians_per_second) -> SwerveModuleState:
        """
        This is a helper method to determine the direction and speed to set the nodes
        ONLY WORKS WITH A SQUARE ROBOT!!!
        This should really be a node method.
        Args:
            dx: change in x desired
            dy: change in y desired
            d_theta: change in theta desired

        Returns:
            a tuple of the magnitude and the direction

        """

        # TODO Rewrite function so that the robot doesn't need to be square
        # tangent_x, tangent_y = unit_normal(node_x, node_y)

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * self.tangent[0]
        sy = dy + r * d_theta * self.tangent[1]

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx ** 2 + sy ** 2)
        return SwerveModuleState(magnitude, Rotation2d(theta))

    # 0 degrees is facing right | "ethan is our FRC lord and saviour" - sid
    def _set_angle(self, target_angle: radians, initial_angle: radians) -> None:
        """
        DEPRECATED. The final step in computing a new trajectory for the swerve node.
        Args:
            target_angle: Where you want the swerve node to aim
            initial_angle: Where the swerve node was previously aiming

        Returns: None

        """
        target_sensor_angle, flipped, flip_sensor_offset = SwerveNode._resolve_angles(target_angle, initial_angle)

        target_sensor_angle -= self.motor_sensor_offset

        if flipped:
            self.motor_reversed = not self.motor_sensor_offset.get_motor_direction()
            self.motor_sensor_offset += flip_sensor_offset

        self.set_motor_angle(target_sensor_angle)

    @staticmethod
    def _resolve_angles(target_angle: radians, initial_angle: radians) -> tuple[float, bool, float]:
        """
        DEPRECATED
        :param target_angle: Target node angle
        :param initial_angle: Initial node sensor angle
        :return: (target_sensor_angle, flipped, flip_sensor_offset)
        """
        # TODO This function only has one client function. Rewrite or integrate into _calculate_swerve_node
        # Actual angle difference in radians
        diff = bounded_angle_diff(initial_angle, target_angle)

        # Should we flip
        if abs(diff) > 0.65 * math.pi:  #
            flip_sensor_offset = math.pi if diff > 0 else -math.pi  # I think that this part doesn't make much sense.
            diff -= flip_sensor_offset
            return diff + initial_angle, True, flip_sensor_offset

        return diff + initial_angle, False, 0
