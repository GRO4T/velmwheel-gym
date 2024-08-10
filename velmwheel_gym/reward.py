import logging
from dataclasses import dataclass

import numpy as np

from velmwheel_gym.global_guidance_path import GlobalGuidancePath
from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class RewardFuncCoeffs:
    collision_penalty: float
    timeout_penalty: float
    success_reward: float
    path_following_reward: float
    distance_factor: float
    misalignment_factor: float
    rotation_velocity_penalty: float
    obstacle_penalty: float


GLOBAL_GUIDANCE_COEFFS = RewardFuncCoeffs(
    collision_penalty=-0.6,
    timeout_penalty=-0.5,
    success_reward=0.4,
    path_following_reward=0.4,
    distance_factor=0.1,
    misalignment_factor=0.1,
    rotation_velocity_penalty=-0.1,
    obstacle_penalty=-0.1,
)

NO_GLOBAL_GUIDANCE_COEFFS = RewardFuncCoeffs(
    collision_penalty=-0.7,
    timeout_penalty=-0.5,
    success_reward=0.5,
    path_following_reward=0.0,
    distance_factor=0.1,
    misalignment_factor=0.1,
    rotation_velocity_penalty=-0.1,
    obstacle_penalty=-0.1,
)


# pylint: disable=too-many-arguments
def calculate_reward(
    variant: str,
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
    robot_w: float,
) -> tuple[bool, float, bool]:
    if variant == "NoGlobalGuidance":
        coeffs = NO_GLOBAL_GUIDANCE_COEFFS
    else:
        coeffs = GLOBAL_GUIDANCE_COEFFS

    reward = 0.0

    # Robot collided with an obstacle
    if is_robot_collide:
        # Add penalty for all other negative factors as if all steps until
        # the end of the episode were taken in the worst possible way
        reward += (
            (max_episode_steps - steps)
            * (
                coeffs.obstacle_penalty
                + coeffs.rotation_velocity_penalty
                - coeffs.misalignment_factor
                - coeffs.distance_factor
            )
            / max_episode_steps
        )
        reward += coeffs.collision_penalty
        return False, reward, True

    # Robot reached the maximum number of steps
    if steps == max_episode_steps:
        return False, coeffs.timeout_penalty, True

    # Apply penalty for driving too close to obstacles
    if min_obstacle_dist < 1.0:
        reward += (
            coeffs.obstacle_penalty * (1.0 - min_obstacle_dist) / max_episode_steps
        )

    # Apply penalty for high rotation velocity
    reward += coeffs.rotation_velocity_penalty * abs(robot_w) / max_episode_steps

    # Apply penalty for misalignment with the goal
    misalignment_component = coeffs.misalignment_factor * max(
        -1.0, (np.pi - abs(alpha)) / np.pi
    )

    if variant == "EasierFollowing":
        threshold = (
            difficulty.goal_reached_threshold
            if is_final_goal
            else difficulty.driving_in_path_tolerance
        )
    else:
        threshold = difficulty.goal_reached_threshold
    if robot_position.dist(goal) < threshold:
        # Add reward for remaining points on the global guidance path
        if (
            variant != "NoGlobalGuidance"
            and global_guidance_path.original_num_points > 0
        ):
            reward += (
                (coeffs.path_following_reward + misalignment_component)
                * len(global_guidance_path.points)
                / global_guidance_path.original_num_points
            )
        reward += coeffs.success_reward + misalignment_component
        return True, reward, True

    if variant != "NoGlobalGuidance" and num_passed_points > 0:
        # Add reward for reaching global guidance path points
        reward += (coeffs.path_following_reward + misalignment_component) * (
            num_passed_points / global_guidance_path.original_num_points
        )
    else:
        # Add reward for driving towards the goal
        d1 = prev_robot_position.dist(goal)
        d2 = robot_position.dist(goal)
        distance_component = coeffs.distance_factor * (d1 - d2) * 40
        reward += (distance_component + misalignment_component) / max_episode_steps

    return False, reward, False
