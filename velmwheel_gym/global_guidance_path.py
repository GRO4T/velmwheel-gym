import logging

from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


POINT_REACHED_THRESHOLD = 0.05  # meters


class GlobalGuidancePath:
    def __init__(self, points: list[Point]):
        self._original_num_points = len(points)
        self._points = points

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
        for i, point in enumerate(self._points):
            if robot_position.dist(point) < POINT_REACHED_THRESHOLD:
                last_passed_point = i
        if last_passed_point is not None:
            self._points = self._points[last_passed_point + 1 :]
            logger.debug(f"Passed {last_passed_point + 1} points.")
            return last_passed_point + 1
        return 0
