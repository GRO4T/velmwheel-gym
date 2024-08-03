import logging

import numpy as np

from velmwheel_gym.global_guidance_path import GlobalGuidancePath
from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)


COLLISION_PENALTY = -0.9
DETOUR_PENALTY = 0.0
SUCCESS_REWARD = 0.5
PATH_FOLLOWING_REWARD = 0.5
DISTANCE_FACTOR = 0.1
MISALIGNMENT_FACTOR = 0.0


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
) -> tuple[bool, float, bool]:
    reward = 0.0

    if is_robot_collide:
        reward += (
            (max_episode_steps - steps)
            * (-MISALIGNMENT_FACTOR - DISTANCE_FACTOR)
            / max_episode_steps
        )
        reward += COLLISION_PENALTY
        return False, reward, True

    misalignment_component = MISALIGNMENT_FACTOR * max(
        -1.0, (np.pi / 6 - abs(alpha)) / (np.pi / 6)
    )

    threshold = (
        difficulty.goal_reached_threshold
        if is_final_goal
        else difficulty.driving_in_path_tolerance
    )
    if robot_position.dist(goal) < threshold:
        reward += SUCCESS_REWARD
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
        distance_component = DISTANCE_FACTOR * (d1 - d2) * 30
        reward += (
            distance_component + min(0.0, misalignment_component)
        ) / max_episode_steps

    return False, reward, False
