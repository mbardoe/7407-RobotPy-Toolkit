import math

from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from robotpy_toolkit_7407 import PIDMotor
from robotpy_toolkit_7407.encoder import AbsoluteEncoder
from robotpy_toolkit_7407.utils.math import bounded_angle_diff, unit_normal
from robotpy_toolkit_7407.utils.units import meters, meters_per_second, \
    radians_per_second, radians
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNodeConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNodeOffset

# 0 degrees is facing right | "ethan is our FRC lord and saviour" - sid
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

    def __init__(self, config: SwerveNodeConfig):
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

    def init(self) -> None:
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

    def set_for_speed_and_turn(self, x_vel: meters_per_second,
                               y_vel: meters_per_second,
                               angular_vel: radians_per_second) -> None:
        """
        Set the SwerveNode to the desired position and velocity to achieve a particular robot motion.

        Args:
            x_vel: x-Velocity for the robot
            y_vel: y-Velocity for the robot
            angular_vel: angular velocity for the robot

        Returns: None

        """
        state = self.calculate(x_vel, y_vel, angular_vel)
        self.set(state.speed, state.angle)

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
        self.set_motor_velocity(vel if not self.motor_sensor_offset.get_motor_direction() else -vel)

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

        # tangent_x, tangent_y = unit_normal(node_x, node_y)

        sx = dx + d_theta * self.tangent[0]
        sy = dy + d_theta * self.tangent[1]

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx ** 2 + sy ** 2)
        return SwerveModuleState(magnitude, Rotation2d(theta))

    def get_translation(self) -> Translation2d:
        return Translation2d(self.node_x, self.node_y)
