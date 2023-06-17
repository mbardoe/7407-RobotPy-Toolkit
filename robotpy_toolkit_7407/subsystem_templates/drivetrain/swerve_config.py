from dataclasses import dataclass

from robotpy_toolkit_7407 import PIDMotor
from robotpy_toolkit_7407.encoder import AbsoluteEncoder
from robotpy_toolkit_7407.utils.units import meters, radians


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
