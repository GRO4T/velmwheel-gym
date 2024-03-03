import math
from typing import NamedTuple


class Point(NamedTuple):
    x: float
    y: float

    def dist(self, other: "Point") -> float:
        return math.dist((self.x, self.y), (other.x, other.y))
