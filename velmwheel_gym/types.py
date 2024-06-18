import math
from typing import NamedTuple


class Point(NamedTuple):
    x: float
    y: float

    def dist(self, other: "Point") -> float:
        return math.dist((self.x, self.y), (other.x, other.y))


class NavigationDifficulty(NamedTuple):
    goal_reached_threshold: float
    driving_in_path_tolerance: float
    dynamic_obstacle_count: int
