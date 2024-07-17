import pytest

from .constants import NAVIGATION_DIFFICULTIES
from .global_guidance_path import GlobalGuidancePath, next_segment
from .types import Point


def test_global_guidance_path_when_points_empty():
    # Arrange
    robot_position = Point(0, 0)
    difficulty = NAVIGATION_DIFFICULTIES[0]
    # Act
    path = GlobalGuidancePath(robot_position, [], difficulty)
    num_passed_points = path.update(robot_position)
    # Assert
    assert path.points == []
    assert path.original_num_points == 0
    assert num_passed_points == 0


@pytest.mark.parametrize(
    "points, current_segment, expected_points, expected_segment",
    [
        (
            [Point(0, 0), Point(0, 2.5), Point(0, 5.0)],
            [],
            [],
            [Point(0, 0), Point(0, 2.5), Point(0, 5.0)],
        ),
        (
            [Point(0, 0), Point(0, 2.5), Point(0, 5.0), Point(0, 7.5)],
            [],
            [Point(0, 7.5)],
            [Point(0, 0), Point(0, 2.5), Point(0, 5.0)],
        ),
        (
            [Point(0, 7.5)],
            [
                Point(0, 0),
                Point(0, 2.5),
                Point(0, 5.0),
            ],
            [],
            [Point(0, 0), Point(0, 2.5), Point(0, 5.0), Point(0, 7.5)],
        ),
        (
            [Point(0, 7.5), Point(0, 10.0), Point(0, 12.5)],
            [
                Point(0, 0),
                Point(0, 2.5),
                Point(0, 5.0),
            ],
            [Point(0, 12.5)],
            [Point(0, 0), Point(0, 2.5), Point(0, 5.0), Point(0, 7.5), Point(0, 10.0)],
        ),
    ],
)
def test_next_segment(points, current_segment, expected_points, expected_segment):
    # Arrange
    robot_position = Point(0, -1.5)
    difficulty = NAVIGATION_DIFFICULTIES[0]
    # Act
    points, new_segment = next_segment(
        points, current_segment, robot_position, difficulty
    )
    # Assert
    assert points == expected_points
    assert new_segment.points == expected_segment
