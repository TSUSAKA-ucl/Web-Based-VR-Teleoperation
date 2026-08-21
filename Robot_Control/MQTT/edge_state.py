from enum import Enum, auto


class EdgeState(Enum):
    INIT = auto()
    STARTING = auto()
    REGISTRATION = auto()
    REGISTERED = auto()
    ACQUIRED = auto()
    RUNNING = auto()
    EXITING = auto()
