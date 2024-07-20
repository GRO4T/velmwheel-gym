import logging

import numpy as np

from velmwheel_gym.global_guidance_path import GlobalGuidancePath
from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)


COLLISION_PENALTY = -0.8
DETOUR_PENALTY = -0.1
MISALIGNMENT_PENALTY = -0.1
SUCCESS_REWARD = 0.5
PATH_FOLLOWING_REWARD = 0.5


# pylint: disable=too-many-arguments
def calculate_reward(
    is_final_goal: bool,
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
            * (DETOUR_PENALTY + MISALIGNMENT_PENALTY)
            / max_episode_steps
        )
        reward += COLLISION_PENALTY
        return False, reward, True

    threshold = (
        difficulty.goal_reached_threshold
        if is_final_goal
        else difficulty.driving_in_path_tolerance
    )
    if robot_position.dist(goal) < threshold:
        reward += SUCCESS_REWARD
        if global_guidance_path.original_num_points > 0:
            reward += (
                PATH_FOLLOWING_REWARD
                * len(global_guidance_path.points)
                / global_guidance_path.original_num_points
            )
        return True, reward, True

    reward = 0
    if alpha < 0.4 or alpha > 2.7:
        reward += MISALIGNMENT_PENALTY / max_episode_steps

    if num_passed_points > 0:
        reward += PATH_FOLLOWING_REWARD * (
            num_passed_points / global_guidance_path.original_num_points
        )
    else:
        reward += DETOUR_PENALTY / max_episode_steps

    return False, reward, False
