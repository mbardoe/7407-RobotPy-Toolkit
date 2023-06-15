from typing import Protocol


class Encoder(Protocol):

    def init(self)->None:
        '''
        Called at creation
        Returns: None

        '''
        ...

    def getPosition(self) -> float:
        '''
        Where is the encoder reading.
        Returns: float the encoder reading

        '''
        ...

    def getVelocity(self) -> float:
        '''
        Gives the encoders rate of change.
        Returns: float gives how fast the encoder is changing.

        '''
        ...

class AbsoluteEncoder(Protocol):

    def init(self)->None:
        '''
        Called at creation
        Returns: None

        '''
        ...

    def getPosition(self) -> float:
        '''
        Where is the encoder reading.
        Returns: float the encoder reading

        '''
        ...

    def getVelocity(self) -> float:
        '''
        Gives the encoders rate of change.
        Returns: float gives how fast the encoder is changing.

        '''
        ...

    def getAbsolutePosition(self)->float:
        '''
        Gives the postion of the encoder relative to its absolute zero.
        Returns: a number reflecting the position of the motor relative to its
                absolute zero.
        '''
        ...