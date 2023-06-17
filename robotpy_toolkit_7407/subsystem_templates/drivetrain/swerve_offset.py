import math

from robotpy_toolkit_7407.utils.units import radians


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
