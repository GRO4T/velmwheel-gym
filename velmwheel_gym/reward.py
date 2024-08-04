import logging

import numpy as np

from velmwheel_gym.global_guidance_path import GlobalGuidancePath
from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)


COLLISION_PENALTY = -0.7
DETOUR_PENALTY = 0.0
SUCCESS_REWARD = 0.4
PATH_FOLLOWING_REWARD = 0.4
DISTANCE_FACTOR = 0.1
MISALIGNMENT_FACTOR = 0.1
OBSTACLE_PENALTY = -0.1


# pylint: disable=too-many-arguments
def calculate_reward(
    is_final_goal: bool,
    prev_robot_position: Point,
    robot_position: Point,
    alpha: float,
    goal: Point,
    is_robot_collide: bool,
    difficulty: NavigationDifficulty,
    num_passed_points: int,
    global_guidance_path: GlobalGuidancePath,
    max_episode_steps: int,
    steps: int,
    min_obstacle_dist: float,
) -> tuple[bool, float, bool]:
    reward = 0.0

    if is_robot_collide or steps == max_episode_steps:
        reward += (
            (max_episode_steps - steps)
            * (OBSTACLE_PENALTY - MISALIGNMENT_FACTOR - DISTANCE_FACTOR)
            / max_episode_steps
        )
        reward += COLLISION_PENALTY
        return False, reward, True

    if min_obstacle_dist < 1.0:
        reward += OBSTACLE_PENALTY * (1.0 - min_obstacle_dist) / max_episode_steps

    misalignment_component = MISALIGNMENT_FACTOR * max(
        -1.0, (np.pi - abs(alpha)) / np.pi
    )

    threshold = (
        difficulty.goal_reached_threshold
        if is_final_goal
        else difficulty.driving_in_path_tolerance
    )
    if robot_position.dist(goal) < threshold:
        reward += SUCCESS_REWARD + misalignment_component
        if global_guidance_path.original_num_points > 0:
            reward += (
                (PATH_FOLLOWING_REWARD + misalignment_component)
                * len(global_guidance_path.points)
                / global_guidance_path.original_num_points
            )
        return True, reward, True

    if num_passed_points > 0:
        reward += (PATH_FOLLOWING_REWARD + misalignment_component) * (
            num_passed_points / global_guidance_path.original_num_points
        )
    else:
        d1 = prev_robot_position.dist(goal)
        d2 = robot_position.dist(goal)
        distance_component = DISTANCE_FACTOR * (d1 - d2) * 20
        reward += (distance_component + misalignment_component) / max_episode_steps

    return False, reward, False
