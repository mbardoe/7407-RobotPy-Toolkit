from typing import Protocol
from dataclasses import dataclass

from robotpy_toolkit_7407.utils.units import radians, radians_per_second
from robotpy_toolkit_7407.encoder import Encoder, AbsoluteEncoder



class Motor(Protocol):

    def init(self) -> None:
        '''
        OVERRIDE. Init method to start all the associated code. Possibly zero.
        Returns: None

        '''
        ...

    def set_raw_output(self, x: float) -> None:
        '''
        OVERRIDE. Set the raw input speed of the motor.
        Args:
            x: The raw input speed of the motor

        Returns: None

        '''
        ...


@dataclass
class EncoderMotor(Protocol):
    encoder: Encoder

    def init(self) -> None:
        '''
        OVERRIDE. Init method to start all the associated code. Possibly zero.
        Returns: None

        '''
        ...

    def set_raw_output(self, x: float) -> None:
        '''
        OVERRIDE. Set the raw input speed of the motor.
        Args:
            x: The raw input speed of the motor

        Returns: None

        '''
        ...

    def get_sensor_position(self) -> radians:
        '''
        OVERRIDE. Get the sensor position from the encoder
        Returns: sensor position in radians

        '''
        ...

    def get_sensor_velocity(self) -> radians_per_second:
        '''
        OVERRIDE. Get the sensor velocity from the encoder
        Returns: change in sensor readings in radians per second

        '''
        ...

    def set_sensor_position(self, pos: radians) -> None:
        '''
        OVERRIDE. Set the current sensor position to the following value.
        Args:
            pos: The number of radians that the current position should be
                 assigned to.

        Returns: None

        '''
        ...


class PIDMotor(Protocol):
    encoder: AbsoluteEncoder

    def init(self) -> None:
        '''
        OVERRIDE. Init method to start all the associated code. Possibly zero.

        Returns: None

        '''
        ...

    def set_raw_output(self, x: float) -> None:
        '''
        OVERRIDE. Set the raw input speed of the motor.
        Args:
            x: The raw input speed of the motor

        Returns: None

        '''

        ...

    def get_sensor_position(self) -> radians:
        '''
        OVERRIDE. Get the sensor position from the encoder
        Returns: sensor position in radians

        '''
        ...

    def get_sensor_velocity(self) -> radians_per_second:
        '''
        OVERRIDE. Get the sensor velocity from the encoder
        Returns: change in sensor reading in radians per second

        '''
        ...

    def set_sensor_position(self, pos: radians) -> None:
        '''
        OVERRIDE. Set the current sensor position to the following value.
        Args:
            pos: The number of radians that the current position should be
                 assigned to.

        Returns: None
        '''
        ...

    def set_target_position(self, pos: radians) -> None:
        '''
        OVERRIDE. Set the target position for the PID Controller
        Args:
            pos: Position that the PIDMotor should aim for

        Returns: None

        '''
        ...

    def set_target_velocity(self, vel: radians_per_second) -> None:
        '''
        OVERRIDE. Set the target velocity for the PID Controller
        Args:
            vel: Velocity that the PIDMotor should aim for

        Returns: None

        '''
        ...
