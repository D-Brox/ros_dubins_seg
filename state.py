from enum import Enum

class MovementWill(Enum):
    Outward = 1
    Inward = -1
    Stay = 0

class State(Enum):
    InCircle = 1
    Transition = 0
    Arrival = 0.5
