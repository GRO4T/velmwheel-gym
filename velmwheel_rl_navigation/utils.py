from enum import Enum
from collections import namedtuple


Point = namedtuple("Point", "x y")
Point.__doc__ = "A point in 2D space."


def is_iterable(obj):
    try:
        iter(obj)
        return True
    except TypeError:
        return False


class Color(Enum):
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    BLACK = (0, 0, 0)
    WHITE = (254, 254, 254)

    def __eq__(self, other):
        if is_iterable(other):
            return all(self.value == other)
        raise TypeError(
            f"Comparison between {type(other)} and {type(self)} not supported"
        )
