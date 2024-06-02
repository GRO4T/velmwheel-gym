import logging

from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


class GlobalGuidancePath:
    def __init__(self, robot_position: Point, points: list[Point]):
        self._points = points
        # NOTE: we want to trim initial points that are already passed, so we don't reward inaction
        self.update(robot_position, 1.0)
        self._original_num_points = len(points)

    @property
    def points(self) -> list[Point]:
        """List of points in the path."""
        return self._points

    @property
    def original_num_points(self) -> int:
        """Initial number of points in the path."""
        return self._original_num_points

    def update(self, robot_position: Point, point_reached_threshold: float) -> int:
        """Update path by removing passed points and return number of points removed."""
        last_passed_point = None
        for i, point in enumerate(self._points):
            if robot_position.dist(point) < point_reached_threshold:
                last_passed_point = i
        if last_passed_point is not None:
            self._points = self._points[last_passed_point + 1 :]
            return last_passed_point + 1
        return 0


def get_n_points_evenly_spaced_on_path(
    global_guidance_path: GlobalGuidancePath, n: int, default_point: list[float]
) -> list[float]:
    if not global_guidance_path.points:
        return n * default_point

    evenly_spaced_points = []
    for i in range(n):
        idx = int((i / n) * len(global_guidance_path.points))
        evenly_spaced_points.extend(
            [
                global_guidance_path.points[idx].x,
                global_guidance_path.points[idx].y,
            ]
        )
    return evenly_spaced_points
