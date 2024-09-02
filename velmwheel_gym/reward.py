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


# GLOBAL_GUIDANCE_COEFFS = RewardFuncCoeffs(
#     collision_penalty=-0.6,
#     timeout_penalty=-0.5,
#     success_reward=0.4,
#     path_following_reward=0.4,
#     distance_factor=0.1,
#     misalignment_factor=0.1,
#     rotation_velocity_penalty=-0.1,
#     obstacle_penalty=-0.1,
# )
GLOBAL_GUIDANCE_COEFFS = RewardFuncCoeffs(
    collision_penalty=-20.0,
    timeout_penalty=-20.0,
    success_reward=20.0,
    path_following_reward=5.0,
    distance_factor=2.5,
    misalignment_factor=0.1,
    rotation_velocity_penalty=-0.1,
    obstacle_penalty=-0.2,
)

NO_GLOBAL_GUIDANCE_COEFFS = RewardFuncCoeffs(
    collision_penalty=-20.0,
    timeout_penalty=-20.0,
    success_reward=20.0,
    path_following_reward=0.0,
    distance_factor=2.5,
    misalignment_factor=0.6,
    rotation_velocity_penalty=-0.1,
    obstacle_penalty=-0.2,
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
    reward = 0.0
    if "NoGlobalGuidance" in variant:
        coeffs = NO_GLOBAL_GUIDANCE_COEFFS
    else:
        coeffs = GLOBAL_GUIDANCE_COEFFS

    # Robot collided with an obstacle
    if is_robot_collide:
        reward += coeffs.collision_penalty
        return False, coeffs.collision_penalty, True

    if "EasierFollowing" in variant:
        threshold = (
            difficulty.goal_reached_threshold
            if is_final_goal
            else difficulty.driving_in_path_tolerance
        )
    else:
        threshold = difficulty.goal_reached_threshold
    if robot_position.dist(goal) < threshold:
        return True, coeffs.success_reward, True

    # Robot reached the maximum number of steps
    # if steps == max_episode_steps:
    #     terminated = True
    #     reward += coeffs.timeout_penalty

    # Apply penalty for driving too close to obstacles
    if min_obstacle_dist < 1.5:
        reward += coeffs.obstacle_penalty * (1.5 - min_obstacle_dist)

    # Apply penalty for high rotation velocity
    if abs(robot_w) > 0.25:
        reward += coeffs.rotation_velocity_penalty * abs(robot_w)

    # Apply penalty for misalignment with the goal
    if abs(alpha) > np.pi / 6:
        reward += coeffs.misalignment_factor * (np.pi / 6 - abs(alpha))

    if "NoGlobalGuidance" not in variant and num_passed_points > 0:
        # Add reward for reaching global guidance path points
        reward += coeffs.path_following_reward * num_passed_points
    else:
        # Add reward for driving towards the goal
        d1 = prev_robot_position.dist(goal)
        d2 = robot_position.dist(goal)
        distance_component = coeffs.distance_factor * (d1 - d2)
        reward += distance_component

    return False, reward, False
