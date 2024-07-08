import copy
import logging

from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)


class GlobalGuidancePath:
    def __init__(
        self,
        robot_position: Point,
        points: list[Point],
        difficulty: NavigationDifficulty,
    ):
        self._points = points
        self._difficulty = difficulty
        # NOTE: we want to trim initial points that are already passed, so we don't reward inaction
        self.update(robot_position)
        self._original_num_points = len(points)

    @property
    def points(self) -> list[Point]:
        """List of points in the path."""
        return self._points

    @property
    def original_num_points(self) -> int:
        """Initial number of points in the path."""
        return self._original_num_points

    def update(self, robot_position: Point) -> int:
        """Update path by removing passed points and return number of points removed."""
        last_passed_point = None
        for i, point in enumerate(self._points[:-1]):
            if robot_position.dist(point) < self._difficulty.driving_in_path_tolerance:
                last_passed_point = i
        if last_passed_point is not None:
            self._points = self._points[last_passed_point + 1 :]
            return last_passed_point + 1
        return 0


def get_n_points_evenly_spaced_on_path(
    points: list[Point],
    n: int,
    default_point: list[float],
    origin: Point,
) -> list[float]:
    if not points:
        return n * default_point

    evenly_spaced_points = []
    for i in range(n):
        idx = int((i / n) * len(points))
        evenly_spaced_points.extend(
            [
                points[idx].x - origin.x,
                points[idx].y - origin.y,
            ]
        )
    return evenly_spaced_points


def next_segment(
    points: list[Point],
    current_segment: list[Point],
    robot_position: Point,
    difficulty: NavigationDifficulty,
) -> GlobalGuidancePath:
    if current_segment is not None and len(current_segment) > 1:
        min_idx = len(current_segment) - 1
        anchor = current_segment[-1]
    else:
        min_idx = 0
        anchor = robot_position

    new_segment_points = copy.deepcopy(points)
    for idx, point in enumerate(points):
        if idx >= min_idx and point.dist(anchor) > 5.0:
            new_segment_points = copy.deepcopy(points[: idx + 1])
            break
    return GlobalGuidancePath(robot_position, new_segment_points, difficulty)
