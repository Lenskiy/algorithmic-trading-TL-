from enum import Enum


class Position(Enum):
    LONG = "Long"
    SHORT = "Short"
    EMPTY = "Empty"

    def __str__(self):
        return self.value
