import math
from dataclasses import dataclass
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, \
    SwerveModulePosition

from robotpy_toolkit_7407.oi.joysticks import JoystickAxis
from robotpy_toolkit_7407.sensors.gyro.swerve_gyro import SwerveGyro
from robotpy_toolkit_7407.subsystem import Subsystem
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import rotate_vector, bounded_angle_diff, unit_normal
from robotpy_toolkit_7407.utils.units import s, m, deg, rad, hour, mile, rev, meters, meters_per_second, \
    radians_per_second, radians
from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.encoder import AbsoluteEncoder


class SwerveNodeOffset:
    motor_sensor_offset: radians = 0
    motor_reversed_start: bool = False

    def init(self, offset: radians, motor_reversed: bool = False) -> None:
        '''

        Args:
            When we start a SwerveNodeOffset we need to know what direction the offset is and the
            the direction of the motor when moving in that direction.

            offset: A radian measure indicating where the motor things zero is.
            motor_reversed: Whether the motor is reversed or not when driving in the direction of the offset

        Returns: None

        '''
        self.set_offset(offset, motor_reversed)

    def change_direction(self) -> None:
        '''
        Change the direction of the offset, and flip the direction.

        Returns: None

        '''
        if self.motor_sensor_offset < math.pi / 2:
            self.motor_sensor_offset += math.pi
        else:
            self.motor_sensor_offset -= math.pi
        self.motor_reversed_start = not self.motor_reversed_start

    def get_offset(self) -> radians:
        '''
        Get the current offset.

        Returns: the current offset in radians

        '''
        return self.motor_sensor_offset

    def get_motor_direction(self) -> bool:
        '''
        Get the direction of the desired direction of the powering motor.
        Returns: a boolean that represents

        '''
        return self.motor_reversed_start

    def set_offset(self, offset: radians, motor_reversed: bool) -> None:
        '''
        Given an offset and the direction of the motor set the current offset to
        that measurement. The motor reversed is updated if the current offset would
        be different given that it is between -pi/2 and pi/2.

        Args:
            offset:
            motor_reversed:

        Returns:

        '''

        new_offset = math.fmod(offset, math.pi)
        if new_offset > math.pi / 2:
            new_offset -= math.pi
        number_pis = round((offset - new_offset) / math.pi)
        if number_pis % 2 == 1:
            self.motor_reversed_start = not motor_reversed
        else:
            self.motor_reversed_start = motor_reversed



class SwerveDrivetrain(Subsystem):
    """
    Swerve Drivetrain Extendable class. Contains driving functions.
    """
    n_front_left: SwerveNode
    n_front_right: SwerveNode
    n_back_left: SwerveNode
    n_back_right: SwerveNode
    gyro: SwerveGyro
    axis_dx: JoystickAxis
    axis_dy: JoystickAxis
    axis_rotation: JoystickAxis
    track_width: meters = 1
    max_vel: meters_per_second = (20 * mile / hour).asNumber(m / s)  # Maximum velocity
    max_angular_vel: radians_per_second = (4 * rev / s).asNumber(rad / s)  # Maximum angular velocity
    deadzone_velocity: meters_per_second = 0.05  # Does not run within this speed
    deadzone_angular_velocity: radians_per_second = (5 * deg / s).asNumber(rad / s)  # Will not turn within this speed
    start_pose: Pose2d = Pose2d(0, 0, 0)  # Starting pose of the robot from wpilib Pose (x, y, rotation)
    gyro_start_angle: radians = 0
    gyro_offset: deg = 0

    def __init__(self):
        super().__init__()
        self.kinematics: SwerveDrive4Kinematics | None = None
        self.odometry: SwerveDrive4Odometry | None = None
        self.odometry_estimator: SwerveDrive4PoseEstimator | None = None
        self.chassis_speeds: ChassisSpeeds | None = None
        self._omega: radians_per_second = 0

        self.node_translations: tuple[Translation2d] | None = None

    def init(self):
        """
        Initialize the swerve drivetrain, kinematics, odometry, and gyro.
        """
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_front_left.init()
        self.n_front_right.init()
        self.n_back_left.init()
        self.n_back_right.init()
        self.gyro.init(self.gyro_start_angle)

        logger.info("initializing odometry", "[swerve_drivetrain]")

        self.node_translations = (
            Translation2d(.5 * self.track_width, .5 * self.track_width),
            Translation2d(.5 * self.track_width, -.5 * self.track_width),
            Translation2d(-.5 * self.track_width, .5 * self.track_width),
            Translation2d(-.5 * self.track_width, -.5 * self.track_width)
        )

        self.kinematics = SwerveDrive4Kinematics(
            *self.node_translations
        )

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading(),
            self.node_positions,
            self.start_pose
        )
        self.odometry_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_heading(),
            self.node_positions,
            self.start_pose
        )

        logger.info("initialization complete", "[swerve_drivetrain]")

    @property
    def node_positions(self) -> tuple[
        SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition
    ]:
        """
        Get the node positions.
        """
        return (
            self.n_front_left.get_node_position(),
            self.n_front_right.get_node_position(),
            self.n_back_left.get_node_position(),
            self.n_back_right.get_node_position()
        )

    @property
    def node_states(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        """
        Get the node states.
        """
        return (
            self.n_front_left.get_node_state(),
            self.n_front_right.get_node_state(),
            self.n_back_left.get_node_state(),
            self.n_back_right.get_node_state()
        )

    def set_driver_centric(self, vel: (meters_per_second, meters_per_second), angular_vel: radians_per_second):
        """
        Set the driver centric velocity and angular velocity. Driver centric runs with perspective of driver.

        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """
        vel = rotate_vector(vel[0], vel[1], -self.gyro.get_robot_heading())
        self.set_robot_centric(vel, angular_vel)

    def set_robot_centric(self, vel: (meters_per_second, meters_per_second), angular_vel: radians_per_second):
        """
        Set the robot centric velocity and angular velocity. Robot centric runs with perspective of robot.
        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """
        self._omega = angular_vel  # For simulation

        if abs(vel[0]) < self.deadzone_velocity and abs(vel[1]) < self.deadzone_velocity and \
                abs(angular_vel) < self.deadzone_angular_velocity:
            self.n_front_left.set_motor_velocity(0)
            self.n_front_right.set_motor_velocity(0)
            self.n_back_left.set_motor_velocity(0)
            self.n_back_right.set_motor_velocity(0)
        else:
            self.n_front_left.set(*self._calculate_swerve_node(
                -.5 * self.track_width, -.5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))
            self.n_front_right.set(*self._calculate_swerve_node(
                -.5 * self.track_width, .5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))
            self.n_back_left.set(*self._calculate_swerve_node(
                .5 * self.track_width, -.5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))
            self.n_back_right.set(*self._calculate_swerve_node(
                .5 * self.track_width, .5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))

        self.odometry.update(
            self.get_heading(),
            *self.node_positions
        )

        self.odometry_estimator.update(
            self.get_heading(),
            self.node_positions
        )

        self.chassis_speeds = self.kinematics.toChassisSpeeds(*self.node_states)

    def stop(self):
        """
        Stop the drivetrain and all pods.
        """
        self.n_front_left.set(0, 0)
        self.n_front_right.set(0, 0)
        self.n_back_left.set(0, 0)
        self.n_back_right.set(0, 0)

    def get_heading(self) -> Rotation2d:
        """
        Get the robot heading.

        Returns:
            Heading (Rotation2d): the robot heading
        """
        return Rotation2d(self.gyro.get_robot_heading() + self.gyro_offset)

    def reset_odometry(self, pose: Pose2d):
        """
        Reset the odometry to a given pose.

        Args:
            pose (Pose2d): The pose to reset the odometry to.
        """
        self.odometry.resetPosition(
            self.get_heading(),
            pose,
            *self.node_positions
        )
        self.odometry_estimator.resetPosition(
            gyroAngle=self.get_heading(),
            pose=pose,
            modulePositions=self.node_positions
        )

    @staticmethod
    def _calculate_swerve_node(node_x: meters, node_y: meters, dx: meters_per_second, dy: meters_per_second,
                               d_theta: radians_per_second) -> (meters_per_second, radians):
        '''
        DEPRECATED. This is a helper method to determine the direction and speed to set the nodes
        ONLY WORKS WITH A SQUARE ROBOT!!!
        This should really be a node method.
        Args:
            node_x: x Position of a node
            node_y: y Position of a node
            dx: change in x desired
            dy: change in y desired
            d_theta: change in theta desired

        Returns:
            a tuple of the magnitude and the direction

        '''
        # TODO move this function to SwerveNode
        # TODO Rewrite function so that the robot doesn't need to be square
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x ** 2 + tangent_y ** 2)
        tangent_x /= tangent_m
        tangent_y /= tangent_m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx ** 2 + sy ** 2)
        return magnitude, theta
