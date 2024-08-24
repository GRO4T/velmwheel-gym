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
    dynamic_obstacles: list[Point]
    dynamic_obstacle_motion: bool
    extend_segment: bool
    starting_rect: tuple[Point, Point]
    goal_rect: tuple[Point, Point]
    raw_lidar_ray_count: int
